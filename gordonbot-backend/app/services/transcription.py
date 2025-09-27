from __future__ import annotations

import io
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
import requests

log = logging.getLogger(__name__)

try:  # pragma: no cover - optional dependency on some dev machines
    from faster_whisper import WhisperModel  # type: ignore
except Exception:  # pragma: no cover
    WhisperModel = None  # type: ignore


class TranscriptionError(RuntimeError):
    pass


class TranscriptionNetworkError(TranscriptionError):
    """Raised when cloud transcription fails due to connectivity issues."""


class Transcriber:
    """Backend-agnostic speech transcriber supporting OpenAI Whisper API, faster_whisper, and whisper.cpp."""

    def __init__(self, settings) -> None:
        self._settings = settings
        self._model_lock = threading.Lock()
        self._model: WhisperModel | None = None
        backend_raw = (getattr(settings, "speech_backend", "auto") or "auto").strip().lower()
        self._auto_mode = backend_raw in {"", "auto"}
        preferred_backend = "whisper-api" if self._auto_mode else backend_raw

        allowed_backends = {"whisper-api", "whispercpp", "faster-whisper"}
        if preferred_backend not in allowed_backends:
            log.warning("Unknown speech backend '%s'; defaulting to whisper-api", preferred_backend)
            preferred_backend = "whisper-api"
            self._auto_mode = True

        self._speech_api_key = getattr(settings, "speech_api_key", None)
        self._speech_api_base = (
            getattr(settings, "speech_api_base", "https://api.openai.com/v1")
            or "https://api.openai.com/v1"
        ).rstrip("/")
        self._speech_api_model = getattr(settings, "speech_api_model", "whisper-1") or "whisper-1"
        self._speech_api_timeout = float(getattr(settings, "speech_api_timeout", 30.0) or 30.0)
        self._speech_api_org = getattr(settings, "speech_api_org", None)

        backend = preferred_backend

        if backend == "whisper-api" and not self._speech_api_key:
            if self._auto_mode:
                log.warning("Whisper API backend requires SPEECH_API_KEY; falling back to whisper.cpp")
                backend = "whispercpp"
            else:
                raise TranscriptionError(
                    "Whisper API backend selected but SPEECH_API_KEY is not configured"
                )

        if backend == "whispercpp" and not self._whispercpp_available():
            if self._auto_mode:
                log.warning("whisper.cpp binary/model unavailable; attempting faster-whisper fallback")
                backend = "faster-whisper"
            else:
                raise TranscriptionError("whisper.cpp binary/model unavailable")

        if backend == "faster-whisper" and WhisperModel is None:
            if self._whispercpp_available():
                log.warning("faster-whisper not installed; using whisper.cpp backend")
                backend = "whispercpp"
            else:
                raise TranscriptionError(
                    "No speech backend available (faster-whisper missing, whisper.cpp unavailable)"
                )

        self._backend = backend
        self._preferred_backend = preferred_backend

        log.info("Speech transcription backend: %s", self._backend)

    # ------------------------------------------------------------------
    def transcribe(self, audio_bytes: bytes) -> tuple[str, float]:
        if self._backend == "whisper-api":
            try:
                return self._transcribe_whisper_api(audio_bytes)
            except TranscriptionNetworkError as exc:
                if self._auto_mode and self._whispercpp_available():
                    log.warning("Whisper API unavailable (%s); using whisper.cpp fallback", exc)
                    self._backend = "whispercpp"
                    return self._transcribe_whispercpp(audio_bytes)
                raise

        if self._backend == "whispercpp":
            return self._transcribe_whispercpp(audio_bytes)

        if self._backend == "faster-whisper":
            return self._transcribe_faster_whisper(audio_bytes)

        raise TranscriptionError(f"Unsupported speech backend: {self._backend}")

    # ------------------------------------------------------------------
    def _transcribe_whisper_api(self, audio_bytes: bytes) -> tuple[str, float]:
        if not self._speech_api_key:
            raise TranscriptionError("SPEECH_API_KEY is required for whisper API transcription")

        with io.BytesIO() as buffer:
            with wave.open(buffer, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16_000)
                wf.writeframes(audio_bytes)
            wav_data = buffer.getvalue()

        url = f"{self._speech_api_base}/audio/transcriptions"
        headers = {
            "Authorization": f"Bearer {self._speech_api_key}",
        }
        if self._speech_api_org:
            headers["OpenAI-Organization"] = self._speech_api_org

        data = {
            "model": self._speech_api_model,
            "response_format": "json",
            "language": "en",
        }

        files = {
            "file": ("speech.wav", wav_data, "audio/wav"),
        }

        try:
            response = requests.post(
                url,
                headers=headers,
                data=data,
                files=files,
                timeout=self._speech_api_timeout,
            )
        except (requests.ConnectionError, requests.Timeout) as exc:
            message = str(exc) or exc.__class__.__name__
            raise TranscriptionNetworkError(message) from exc
        except requests.RequestException as exc:
            raise TranscriptionError(f"Whisper API request failed: {exc}") from exc

        if response.status_code >= 400:
            message: str | None = None
            try:
                payload = response.json()
            except ValueError:
                payload = None
            if isinstance(payload, dict):
                if isinstance(payload.get("error"), dict):
                    message = payload["error"].get("message")
                message = message or payload.get("message")
            if not message:
                message = response.text.strip() or "unknown error"
            raise TranscriptionError(
                f"Whisper API returned HTTP {response.status_code}: {message}"
            )

        try:
            payload = response.json()
        except ValueError as exc:
            raise TranscriptionError("Whisper API returned non-JSON response") from exc

        if not isinstance(payload, dict):
            raise TranscriptionError("Unexpected Whisper API response format")

        text = str(payload.get("text", "")).strip()
        segments_obj = payload.get("segments")
        if not text and isinstance(segments_obj, list):
            collected: list[str] = []
            for segment in segments_obj:
                if isinstance(segment, dict) and segment.get("text"):
                    collected.append(str(segment["text"]).strip())
            text = " ".join(collected).strip()

        confidence = float(payload.get("confidence", 0.0)) if "confidence" in payload else 0.0
        return text, max(0.0, min(1.0, confidence))

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
