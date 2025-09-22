from __future__ import annotations

import logging
import math
import os
import shlex
import subprocess
import tempfile
import threading
import wave
from pathlib import Path
from typing import Optional

import numpy as np  # type: ignore

log = logging.getLogger(__name__)

try:  # pragma: no cover - optional dependency on some dev machines
    from faster_whisper import WhisperModel  # type: ignore
except Exception:  # pragma: no cover
    WhisperModel = None  # type: ignore


class TranscriptionError(RuntimeError):
    pass


class Transcriber:
    """Backend-agnostic speech transcriber supporting faster_whisper and whisper.cpp."""

    def __init__(self, settings) -> None:
        self._settings = settings
        self._backend = (settings.speech_backend or "faster-whisper").lower()
        self._model_lock = threading.Lock()
        self._model: WhisperModel | None = None

        if self._backend not in {"faster-whisper", "whispercpp"}:
            log.warning("Unknown speech backend '%s'; defaulting to faster-whisper", self._backend)
            self._backend = "faster-whisper"

        if self._backend == "whispercpp":
            if not self._whispercpp_available():
                log.warning("whisper.cpp binary/model unavailable; falling back to faster-whisper")
                self._backend = "faster-whisper"

        if self._backend == "faster-whisper" and WhisperModel is None:
            log.warning("faster-whisper not installed; whisper.cpp required")
            if self._whispercpp_available():
                self._backend = "whispercpp"
            else:
                raise TranscriptionError("No speech backend available (faster-whisper missing, whisper.cpp unavailable)")

        log.info("Speech transcription backend: %s", self._backend)

    # ------------------------------------------------------------------
    def transcribe(self, audio_bytes: bytes) -> tuple[str, float]:
        if self._backend == "whispercpp":
            return self._transcribe_whispercpp(audio_bytes)
        return self._transcribe_faster_whisper(audio_bytes)

    # ------------------------------------------------------------------
    def _transcribe_faster_whisper(self, audio_bytes: bytes) -> tuple[str, float]:
        model = self._load_faster_whisper_model()
        if model is None:
            raise TranscriptionError("faster-whisper model unavailable")

        audio_np = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32) / 32768.0
        try:
            segments, _info = model.transcribe(
                audio_np,
                beam_size=1,
                patience=1,
                language="en",
            )
        except Exception as exc:  # pragma: no cover - library error
            raise TranscriptionError(f"Whisper transcription failed: {exc}") from exc

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

    def _load_faster_whisper_model(self) -> Optional[WhisperModel]:
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

    # ------------------------------------------------------------------
    def _transcribe_whispercpp(self, audio_bytes: bytes) -> tuple[str, float]:
        bin_path = Path(self._settings.whispercpp_bin)
        model_path = Path(self._settings.whispercpp_model)

        if not model_path.exists():
            raise TranscriptionError(f"whisper.cpp model missing: {model_path}")

        candidates: list[Path] = []
        if bin_path.exists() and os.access(bin_path, os.X_OK):
            candidates.append(bin_path)
        alt = bin_path.with_name("whisper-cli")
        if alt != bin_path and alt.exists() and os.access(alt, os.X_OK):
            candidates.append(alt)
        alt_main = bin_path.with_name("main")
        if alt_main != bin_path and alt_main.exists() and os.access(alt_main, os.X_OK):
            candidates.append(alt_main)

        if not candidates:
            raise TranscriptionError(f"whisper.cpp executable not found or not executable: {bin_path}")

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = Path(tmp.name)
        try:
            self._write_wav(tmp_path, audio_bytes)

            last_error: str | None = None
            lines: list[str] = []
            for exe in candidates:
                cmd = [
                    str(exe),
                    "-m",
                    str(model_path),
                    "-f",
                    str(tmp_path),
                    "-l",
                    "en",
                    "-t",
                    str(max(1, self._settings.whispercpp_threads)),
                    "-otxt",
                    "-of",
                    str(tmp_path.with_suffix("")),
                ]
                log.debug("Running whisper.cpp: %s", " ".join(shlex.quote(x) for x in cmd))
                result = subprocess.run(cmd, capture_output=True, text=True)
                if result.returncode == 0:
                    transcript_path = tmp_path.with_suffix(".txt")
                    if not transcript_path.exists():
                        raise TranscriptionError("whisper.cpp did not produce transcript")

                    with transcript_path.open("r", encoding="utf-8", errors="ignore") as fh:
                        lines = [line.strip() for line in fh if line.strip()]
                    break

                stderr = result.stderr.strip() or result.stdout.strip()
                last_error = stderr or f"exit code {result.returncode}"
                if "deprecated" in (stderr or "").lower():
                    log.warning("whisper.cpp binary %s reported deprecation", exe)
                    continue
            else:
                raise TranscriptionError(f"whisper.cpp failed: {last_error or 'unknown error'}")

            text_lines: list[str] = []
            for line in lines:
                if line.startswith("[") and "]]" not in line:
                    continue
                if "]" in line:
                    _, _, rest = line.partition("]")
                    if rest:
                        text_lines.append(rest.strip())
                else:
                    text_lines.append(line)

            text = " ".join(text_lines).strip()
            return text, 0.0 if not text else 0.5
        finally:
            try:
                if tmp_path.exists():
                    tmp_path.unlink()
                txt = tmp_path.with_suffix(".txt")
                if txt.exists():
                    txt.unlink()
            except Exception:
                pass

    # ------------------------------------------------------------------
    @staticmethod
    def _write_wav(path: Path, audio_bytes: bytes) -> None:
        with wave.open(str(path), "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16_000)
            wf.writeframes(audio_bytes)

    def _whispercpp_available(self) -> bool:
        bin_path = Path(self._settings.whispercpp_bin)
        model_path = Path(self._settings.whispercpp_model)
        return bin_path.exists() and os.access(bin_path, os.X_OK) and model_path.exists()
