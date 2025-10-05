from __future__ import annotations

import audioop
import logging
import os
import tempfile
import threading
import wave
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import requests

from app.core.config import BACKEND_ROOT, settings
from app.services.audio_playback import AudioCuePlayer
from app.services.diagnostics import get_cpu_temperature
from app.services.transcription import Transcriber, TranscriptionError
from app.services.voice_interaction import SpeechRecorder

log = logging.getLogger(__name__)


Action = Callable[[str, float], str | None]


@dataclass
class IntentRule:
    keywords: Iterable[str]
    action: Action
    description: str

    def matches(self, text: str) -> bool:
        lowered = text.lower()
        return all(token in lowered for token in self.keywords)


_relay_audio_command = (
    getattr(settings, "relay_audio_command", None)
    or getattr(settings, "ack_audio_command", None)
    or getattr(settings, "wakeword_audio_command", "ffplay -nodisp -autoexit {path}")
)

_relay_start_audio_path = Path(BACKEND_ROOT) / "assets" / "audio" / "relay_start.mp3"
_relay_sending_audio_path = Path(BACKEND_ROOT) / "assets" / "audio" / "relay_sending.mp3"

_transcriber_lock = threading.Lock()
_transcriber_instance: Transcriber | None = None


def _play_audio_cue(path: Path) -> None:
    if not path.exists():
        log.warning("Relay audio cue missing: %s", path)
        return

    try:
        player = AudioCuePlayer(
            path=path,
            command_template=_relay_audio_command,
            enabled=True,
            throttle_seconds=0.0,
        )
    except Exception as exc:
        log.error("Failed to initialise relay audio cue player for %s: %s", path, exc)
        return

    try:
        player.play(blocking=True)
    except Exception as exc:
        log.error("Failed to play relay audio cue %s: %s", path, exc)


def _get_transcriber() -> Transcriber | None:
    global _transcriber_instance
    with _transcriber_lock:
        if _transcriber_instance is not None:
            return _transcriber_instance
        try:
            _transcriber_instance = Transcriber(settings)
        except TranscriptionError as exc:
            log.error("Relay transcription backend unavailable: %s", exc)
            return None
    return _transcriber_instance


def _record_relay_message() -> bytes:
    try:
        recorder = SpeechRecorder(
            device_index=settings.wakeword_audio_device_index,
            vad_aggressiveness=settings.speech_vad_aggressiveness,
            silence_ms=settings.speech_vad_silence_ms,
            max_ms=settings.speech_vad_max_ms,
            pre_roll_ms=settings.speech_pre_roll_ms,
        )
    except Exception as exc:
        log.error("Relay message recording unavailable: %s", exc)
        return b""

    try:
        return recorder.record_phrase()
    except Exception as exc:
        log.error("Relay message recording failed: %s", exc)
        return b""


def _apply_gain(audio_bytes: bytes, gain: float) -> bytes:
    if not audio_bytes:
        return audio_bytes

    try:
        gain = float(gain)
    except (TypeError, ValueError):
        return audio_bytes

    if abs(gain - 1.0) < 1e-3:
        return audio_bytes

    gain = max(0.1, min(gain, 4.0))

    try:
        boosted = audioop.mul(audio_bytes, 2, gain)
    except Exception as exc:
        log.warning("Failed to apply intercom gain %.2f: %s", gain, exc)
        return audio_bytes

    return boosted


@contextmanager
def _temporarily_pause_wakeword():
    service = None
    if getattr(settings, "wakeword_enabled", False):
        try:
            from app.main import wake_word_service  # type: ignore
        except Exception:
            wake_word_service = None
        else:
            service = wake_word_service

    if service is not None:
        try:
            service.stop(wait=False)  # type: ignore[attr-defined]
        except Exception as exc:
            log.warning("Failed to pause wake-word service for relay message: %s", exc)
            service = None

    try:
        yield
    finally:
        if service is not None:
            try:
                service.start()  # type: ignore[attr-defined]
            except Exception as exc:
                log.error("Failed to resume wake-word service after relay message: %s", exc)
                print("Wake-word listener may be stopped")


def _play_audio_recording(audio_bytes: bytes) -> bool:
    if not audio_bytes:
        return False

    audio_bytes = _apply_gain(audio_bytes, getattr(settings, "intercom_volume_gain", 1.0))

    try:
        fd, temp_path = tempfile.mkstemp(suffix=".wav", prefix="intercom-")
    except Exception as exc:
        log.error("Failed to allocate temporary file for intercom playback: %s", exc)
        return False

    path = Path(temp_path)
    try:
        os.close(fd)
    except Exception:
        pass

    try:
        with wave.open(temp_path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16_000)
            wf.writeframes(audio_bytes)
    except Exception as exc:
        log.error("Failed to prepare intercom audio file %s: %s", path, exc)
        path.unlink(missing_ok=True)
        return False

    try:
        player = AudioCuePlayer(
            path=path,
            command_template=_relay_audio_command,
            enabled=True,
            throttle_seconds=0.0,
        )
    except Exception as exc:
        log.error("Failed to initialise intercom playback: %s", exc)
        path.unlink(missing_ok=True)
        return False

    try:
        player.play(blocking=True)
        success = True
    except Exception as exc:
        log.error("Intercom playback failed: %s", exc)
        success = False
    finally:
        path.unlink(missing_ok=True)

    return success


def _report_cpu_temperature(_: str, __: float) -> str:
    try:
        temp = get_cpu_temperature()
    except Exception as exc:
        log.error("Failed to read CPU temperature: %s", exc)
        print("Could not read CPU temperature")
        return "Could not read CPU temperature"

    if temp is None:
        print("CPU temperature unavailable")
        return "CPU temperature unavailable"

    message = f"CPU Temperature: {temp:.1f}°C"
    print(message)
    return message


def _offload_to_chatgpt(text: str, confidence: float) -> str:
    api_key = getattr(settings, "speech_api_key", None)
    if not api_key:
        message = "I don't have an internet connection right now."
        print(message)
        return message

    print(f"ChatGPT offload requested (confidence {confidence:.2f})")

    url = f"{settings.tts_api_base}/responses"
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
    }
    org = getattr(settings, "speech_api_org", None)
    if org:
        headers["OpenAI-Organization"] = org

    model = getattr(settings, "voice_reply_model", None) or "gpt-4o-mini"
    system_prompt = getattr(settings, "voice_reply_system_prompt", "Provide concise, helpful spoken answers.")

    payload = {
        "model": model,
        "input": [
            {
                "role": "system",
                "content": system_prompt,
            },
            {
                "role": "user",
                "content": text,
            },
        ],
    }

    try:
        response = requests.post(url, json=payload, headers=headers, timeout=settings.tts_api_timeout)
    except requests.RequestException as exc:
        log.error("ChatGPT offload failed: %s", exc)
        message = "I couldn't reach the assistant service."
        print(message)
        return message

    if response.status_code >= 400:
        snippet = response.text[:200]
        log.error("ChatGPT offload error %s: %s", response.status_code, snippet)
        message = "I couldn't get an answer right now."
        print(message)
        return message

    try:
        data = response.json()
    except ValueError:
        log.error("ChatGPT offload returned non-JSON response")
        message = "I received an unexpected response."
        print(message)
        return message

    text_parts: list[str] = []
    for item in data.get("output", []) or []:
        if isinstance(item, dict):
            content = item.get("content")
            if isinstance(content, list):
                for block in content:
                    if isinstance(block, dict) and block.get("type") == "output_text":
                        text_parts.append(block.get("text", ""))
            elif isinstance(content, str):
                text_parts.append(content)

    answer = " ".join(part.strip() for part in text_parts if part).strip()
    if not answer:
        answer = "I don't have an answer for that right now."

    print(answer)
    return answer


def _relay_message(_: str, __: float) -> str:
    print("Relay message mode activated")

    with _temporarily_pause_wakeword():
        _play_audio_cue(_relay_start_audio_path)
        audio_bytes = _record_relay_message()
        _play_audio_cue(_relay_sending_audio_path)

    if not audio_bytes:
        message = "I didn't hear any message to relay."
        print(message)
        return message

    transcriber = _get_transcriber()
    if transcriber is None:
        message = "I can't transcribe messages right now."
        print(message)
        return message

    try:
        text, relay_confidence = transcriber.transcribe(audio_bytes)
    except TranscriptionError as exc:
        log.error("Relay transcription failed: %s", exc)
        message = "I couldn't transcribe that message."
        print(message)
        return message

    relay_text = text.strip()
    if not relay_text:
        message = "I didn't catch that message."
        print(message)
        return message

    print(f"Relay message transcript: '{relay_text}' (confidence {relay_confidence:.2f})")
    return relay_text


def _intercom(_: str, __: float) -> str | None:
    print("Intercom mode activated")

    with _temporarily_pause_wakeword():
        _play_audio_cue(_relay_start_audio_path)
        audio_bytes = _record_relay_message()
        _play_audio_cue(_relay_sending_audio_path)

    if not audio_bytes:
        message = "I didn't hear anything to play back."
        print(message)
        return message

    played = _play_audio_recording(audio_bytes)
    if not played:
        message = "I couldn't play back that recording."
        print(message)
        return message

    print("Intercom playback complete")
    return None


RULES: list[IntentRule] = [
    IntentRule(
        keywords=("cpu", "temperature"),
        action=_report_cpu_temperature,
        description="Report CPU temperature",
    ),
    IntentRule(
        keywords=("relay", "message"),
        action=_relay_message,
        description="Capture and replay a relay message",
    ),
    IntentRule(
        keywords=("intercom",),
        action=_intercom,
        description="Capture and play back user speech directly",
    ),
]


def handle_transcript(text: str, confidence: float) -> str | None:
    for rule in RULES:
        if rule.matches(text):
            log.debug("Voice intent matched: %s", rule.description)
            return rule.action(text, confidence)

    return _offload_to_chatgpt(text, confidence)
