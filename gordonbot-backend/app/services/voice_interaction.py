from __future__ import annotations

import logging
import math
import queue
import struct
import threading
import time
import wave
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np  # type: ignore

from .audio_playback import AudioCuePlayer

log = logging.getLogger(__name__)

try:  # pragma: no cover - optional at dev time
    import webrtcvad  # type: ignore
except Exception:  # pragma: no cover
    webrtcvad = None  # type: ignore

try:  # pragma: no cover
    from pvrecorder import PvRecorder  # type: ignore
except Exception:  # pragma: no cover
    PvRecorder = None  # type: ignore

try:  # pragma: no cover
    from faster_whisper import WhisperModel  # type: ignore
except Exception:  # pragma: no cover
    WhisperModel = None  # type: ignore


def _pcm_to_bytes(frame: Sequence[int]) -> bytes:
    return struct.pack("<%dh" % len(frame), *frame)


class SpeechRecorder:
    """Captures speech segments using PvRecorder and WebRTC VAD."""

    def __init__(
        self,
        *,
        device_index: int | None,
        vad_aggressiveness: int,
        silence_ms: int,
        max_ms: int,
        pre_roll_ms: int,
    ) -> None:
        if webrtcvad is None or PvRecorder is None:
            raise RuntimeError("Speech recording unavailable: missing webrtcvad or pvrecorder")

        self._device_index = device_index if device_index is not None else -1
        self._vad = webrtcvad.Vad(max(0, min(3, vad_aggressiveness)))
        self._silence_frames = max(1, silence_ms // 20)
        self._max_frames = max(1, max_ms // 20)
        self._pre_roll_frames = max(0, pre_roll_ms // 20)

        self.sample_rate = 16_000
        self.frame_ms = 20
        self.frame_length = int(self.sample_rate * self.frame_ms / 1000)

    def record_phrase(self) -> bytes:
        recorder = PvRecorder(device_index=self._device_index, frame_length=self.frame_length)
        frames: list[bytes] = []
        ring: deque[bytes] = deque(maxlen=self._pre_roll_frames)
        silence_count = 0
        triggered = False
        total_frames = 0

        try:
            recorder.start()
        except Exception as exc:
            recorder.delete()
            raise RuntimeError(f"Failed to start audio recorder: {exc}") from exc

        try:
            while total_frames < self._max_frames:
                try:
                    pcm = recorder.read()
                except Exception as exc:
                    log.warning("Speech recorder read error: %s", exc)
                    time.sleep(0.1)
                    continue

                frame_bytes = _pcm_to_bytes(pcm)
                total_frames += 1
                is_speech = self._vad.is_speech(frame_bytes, self.sample_rate)

                if not triggered:
                    ring.append(frame_bytes)
                    if is_speech:
                        triggered = True
                        frames.extend(ring)
                        ring.clear()
                else:
                    frames.append(frame_bytes)
                    if is_speech:
                        silence_count = 0
                    else:
                        silence_count += 1
                        if silence_count >= self._silence_frames:
                            break
        finally:
            try:
                recorder.stop()
            except Exception:
                pass
            try:
                recorder.delete()
            except Exception:
                pass

        if not frames:
            return b""

        return b"".join(frames)


class VoiceInteractionController:
    """Coordinates wake-word, speech capture, and transcription."""

    def __init__(
        self,
        *,
        settings,
        wakeword_service_getter: Callable[[], Optional[object]],
        wake_player: AudioCuePlayer | None = None,
    ) -> None:
        self._wakeword_service_getter = wakeword_service_getter
        self._queue: "queue.Queue[datetime]" = queue.Queue()
        self._worker = threading.Thread(target=self._run, name="voice-interaction", daemon=True)
        self._worker.start()
        self._busy = threading.Event()
        self._model_lock = threading.Lock()
        self._model: WhisperModel | None = None
        self._settings = settings
        self._wake_player = wake_player
        self._ack_player: AudioCuePlayer | None = None

        if getattr(settings, "ack_audio_enabled", False):
            try:
                self._ack_player = AudioCuePlayer(
                    path=settings.ack_audio_path,
                    command_template=settings.ack_audio_command,
                    enabled=True,
                )
            except Exception as exc:  # pragma: no cover - best effort
                log.warning("Failed to initialise acknowledgement audio player: %s", exc)
                self._ack_player = None

        if webrtcvad is None:
            log.warning("WebRTC VAD not available; voice transcription disabled")

        if WhisperModel is None:
            log.warning("faster-whisper not available; voice transcription disabled")

    def notify_wake(self, ts: datetime) -> None:
        if self._busy.is_set():
            log.debug("Wake-word detected while transcription in progress; ignoring")
            return
        self._queue.put(ts)

    # ------------------------------------------------------------------
    def _run(self) -> None:  # pragma: no cover - background thread
        while True:
            ts = self._queue.get()
            self._busy.set()
            try:
                self._handle_interaction(ts)
            except Exception:
                log.exception("Voice interaction failed")
            finally:
                self._busy.clear()

    def _handle_interaction(self, ts: datetime) -> None:
        if webrtcvad is None or WhisperModel is None or PvRecorder is None:
            log.debug("Voice interaction skipped: missing dependencies")
            return

        log.info("Voice interaction starting (wake at %s)", ts.isoformat())

        service = self._wakeword_service_getter()
        if service is not None:
            try:
                service.stop(wait=False)  # type: ignore[attr-defined]
            except Exception as exc:
                log.warning("Failed to stop wake-word service prior to recording: %s", exc)

        if self._wake_player is not None:
            self._wake_player.play(blocking=True)

        audio_bytes = b""
        try:
            recorder = SpeechRecorder(
                device_index=self._settings.wakeword_audio_device_index,
                vad_aggressiveness=self._settings.speech_vad_aggressiveness,
                silence_ms=self._settings.speech_vad_silence_ms,
                max_ms=self._settings.speech_vad_max_ms,
                pre_roll_ms=self._settings.speech_pre_roll_ms,
            )
            audio_bytes = recorder.record_phrase()
        except Exception as exc:
            log.error("Speech recording failed: %s", exc)

        if not audio_bytes:
            log.info("No speech detected after wake word")
            if service is not None and self._settings.wakeword_enabled:
                try:
                    service.start()  # type: ignore[attr-defined]
                except Exception as exc:
                    log.error("Failed to restart wake-word service: %s", exc)
            return

        self._maybe_save_recording(audio_bytes)

        if self._ack_player is not None:
            self._ack_player.play()

        if service is not None and self._settings.wakeword_enabled:
            try:
                service.start()  # type: ignore[attr-defined]
            except Exception as exc:
                log.error("Failed to restart wake-word service: %s", exc)

        text, confidence = self._transcribe(audio_bytes)
        if text:
            msg = f"Wake transcript: '{text}' (confidence {confidence:.2f})"
        else:
            msg = "Wake transcript: <no text recognised>"

        print(msg)
        log.info(msg)
        if text:
            self._dispatch_command(text, confidence)

    # ------------------------------------------------------------------
    def _load_model(self) -> Optional[WhisperModel]:
        if WhisperModel is None:
            return None
        with self._model_lock:
            if self._model is None:
                try:
                    self._model = WhisperModel(
                        self._settings.speech_model,
                        device=self._settings.speech_device,
                        compute_type=self._settings.speech_compute_type,
                    )
                except Exception as exc:
                    log.error("Failed to load Whisper model '%s': %s", self._settings.speech_model, exc)
                    self._model = None
            return self._model

    def _transcribe(self, audio_bytes: bytes) -> tuple[str, float]:
        model = self._load_model()
        if model is None:
            return "", 0.0

        audio_np = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32) / 32768.0
        try:
            segments, info = model.transcribe(
                audio_np,
                beam_size=1,
                patience=1,
                language="en",
            )
        except Exception as exc:
            log.error("Whisper transcription failed: %s", exc)
            return "", 0.0

        texts: list[str] = []
        confidences: list[float] = []
        for seg in segments:
            if seg.text:
                texts.append(seg.text.strip())
            if seg.avg_logprob is not None:
                confidences.append(math.exp(seg.avg_logprob))

        if not texts:
            return "", 0.0

        confidence = float(sum(confidences) / len(confidences)) if confidences else 0.0
        text = " ".join(texts).strip()
        return text, max(0.0, min(1.0, confidence))

    def _dispatch_command(self, text: str, confidence: float) -> None:
        try:
            from app.services.voice_router import handle_transcript
        except Exception:
            log.exception("Failed to import voice_router for command dispatch")
            return

        try:
            handle_transcript(text=text, confidence=confidence)
        except Exception:
            log.exception("Command dispatch raised an error")

    def _maybe_save_recording(self, audio_bytes: bytes) -> None:
        if not getattr(self._settings, "speech_save_recordings", False):
            return
        try:
            target_dir = Path(self._settings.speech_recording_dir)
            target_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.utcnow().strftime("%Y%m%d-%H%M%S-%f")
            path = target_dir / f"wake-{ts}.wav"
            with wave.open(str(path), "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16_000)
                wf.writeframes(audio_bytes)
            log.info("Saved wake recording to %s", path)
        except Exception:
            log.exception("Failed to save wake recording")
