from __future__ import annotations

import logging
import shlex
import shutil
import subprocess
from typing import Optional

import requests

log = logging.getLogger(__name__)


class TextToSpeechError(RuntimeError):
    pass


class TextToSpeech:
    """Speech synthesis with cloud-first (OpenAI) and local fallbacks."""

    def __init__(self, settings) -> None:
        self._settings = settings

        # Preferred backend selection
        preferred_raw = (getattr(settings, "tts_backend", "auto") or "auto").strip().lower()
        if preferred_raw in {"", "auto"}:
            preferred = "auto"
        elif preferred_raw in {"openai", "gpt", "gpt-4o"}:
            preferred = "openai"
        elif preferred_raw in {"espeak", "espeakng"}:
            preferred = "espeakng"
        elif preferred_raw == "none":
            preferred = "none"
        else:
            log.warning("Unknown TTS backend '%s'; defaulting to auto", preferred_raw)
            preferred = "auto"
        self._preferred_backend = preferred

        # SoX availability and configuration (shared by all backends)
        sox_name = getattr(settings, "sox_bin", "sox") or "sox"
        self._sox_bin: Optional[str] = shutil.which(sox_name)
        if self._sox_bin is None:
            log.warning("sox executable '%s' not found; audio filtering/playback via SoX disabled", sox_name)

        destination_raw = getattr(settings, "tts_sox_destination", "-d") or "-d"
        destination = shlex.split(destination_raw)
        self._sox_destination = destination if destination else ["-d"]

        effects_raw = getattr(settings, "tts_sox_effects", "") or ""
        self._sox_effects = shlex.split(effects_raw)
        self._effects_enabled = bool(getattr(settings, "tts_use_filter", False) and self._sox_effects)

        # Local espeak backend setup
        espeak_name = getattr(settings, "espeak_ng_bin", "espeak-ng")
        self._espeak_bin: Optional[str] = shutil.which(espeak_name)
        self._espeak_available = self._espeak_bin is not None
        if not self._espeak_available:
            log.warning("espeak-ng executable '%s' not found; local TTS fallback disabled", espeak_name)

        # Cloud (OpenAI) backend setup
        self._openai_available = bool(getattr(settings, "speech_api_key", None))
        self._tts_api_base = (getattr(settings, "tts_api_base", "https://api.openai.com/v1") or "https://api.openai.com/v1").rstrip("/")
        self._tts_api_timeout = float(getattr(settings, "tts_api_timeout", 30.0) or 30.0)
        self._tts_model = getattr(settings, "tts_openai_model", "gpt-4o-mini-tts") or "gpt-4o-mini-tts"
        self._tts_voice = getattr(settings, "tts_openai_voice", "onyx") or "onyx"

        # Resolve primary backend with fallbacks
        if preferred == "none":
            self._backend = "none"
            log.info("TTS disabled via configuration")
            return

        if preferred == "auto":
            if self._openai_available and self._sox_bin is not None:
                self._backend = "openai"
            elif self._espeak_available:
                self._backend = "espeakng"
            else:
                raise TextToSpeechError("No text-to-speech backend available (missing OpenAI access and espeak-ng)")
        elif preferred == "openai":
            if not self._openai_available:
                log.warning("OpenAI TTS requested but SPEECH_API_KEY is missing; falling back to espeak-ng")
                preferred = "espeakng"
            elif self._sox_bin is None:
                log.warning("OpenAI TTS requires SoX playback; falling back to espeak-ng")
                preferred = "espeakng"
            self._backend = "openai" if preferred == "openai" else "espeakng"
        else:  # espeakng
            self._backend = "espeakng"

        if self._backend == "openai" and not self._openai_available:
            raise TextToSpeechError("OpenAI TTS selected but SPEECH_API_KEY is not configured")
        if self._backend != "openai" and not self._espeak_available:
            raise TextToSpeechError("espeak-ng executable not available for local TTS")

        self._available = self._backend != "none"

    def speak(self, text: str) -> None:
        if not text or not text.strip():
            return
        text = text.strip()

        if getattr(self, "_backend", "none") == "none":
            log.debug("TTS disabled; skipping speech output: %s", text)
            return

        if self._backend == "openai":
            print("[tts] backend=openai")
            try:
                self._speak_openai(text)
                return
            except TextToSpeechError as exc:
                log.warning("OpenAI TTS failed (%s); falling back to espeak-ng", exc)
                if not self._espeak_available:
                    log.error("No TTS fallback available")
                    return
                print("[tts] backend=espeakng (fallback)")
                self._speak_espeak(text)
                return

        # Default to espeak-ng backend
        print(f"[tts] backend={self._backend}")
        if not self._espeak_available:
            log.error("espeak-ng not available for TTS")
            return
        self._speak_espeak(text)

    # ------------------------------------------------------------------
    def _speak_openai(self, text: str) -> None:
        if self._sox_bin is None:
            raise TextToSpeechError("SoX is required for OpenAI playback but is not available")

        key = getattr(self._settings, "speech_api_key", None)
        if not key:
            raise TextToSpeechError("OpenAI API key not configured")

        url = f"{self._tts_api_base}/audio/speech"
        headers = {
            "Authorization": f"Bearer {key}",
            "Content-Type": "application/json",
        }
        org = getattr(self._settings, "speech_api_org", None)
        if org:
            headers["OpenAI-Organization"] = org

        payload = {
            "model": self._tts_model,
            "voice": self._tts_voice,
            "input": text,
            "response_format": "wav",
        }

        try:
            response = requests.post(
                url,
                json=payload,
                headers=headers,
                timeout=self._tts_api_timeout,
            )
        except requests.RequestException as exc:
            raise TextToSpeechError(f"OpenAI request failed: {exc}") from exc

        if response.status_code >= 400:
            snippet = response.text[:200]
            raise TextToSpeechError(f"OpenAI TTS error {response.status_code}: {snippet}")

        audio_bytes = response.content
        if not audio_bytes:
            raise TextToSpeechError("OpenAI TTS returned empty audio")

        try:
            self._play_wave_bytes(audio_bytes, apply_effects=True)
        except TextToSpeechError as exc:
            raise TextToSpeechError(f"SoX playback failed for OpenAI audio: {exc}") from exc

    # ------------------------------------------------------------------
    def _speak_espeak(self, text: str) -> None:
        if not self._espeak_bin:
            raise TextToSpeechError("espeak-ng binary missing")

        voice = getattr(self._settings, "espeak_voice", "en-US")
        rate = int(getattr(self._settings, "espeak_rate", 170))
        pitch = int(getattr(self._settings, "espeak_pitch", 50))
        amplitude = int(getattr(self._settings, "espeak_amplitude", 200))
        amplitude = max(0, min(200, amplitude))

        base_cmd = [
            self._espeak_bin,
            "-v",
            voice,
            "-s",
            str(rate),
            "-p",
            str(pitch),
            "-a",
            str(amplitude),
        ]

        if self._effects_enabled and self._sox_bin is not None:
            stdout_cmd = base_cmd[:1] + ["--stdout"] + base_cmd[1:] + [text]
            try:
                result = subprocess.run(
                    stdout_cmd,
                    check=False,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                )
            except Exception as exc:  # pragma: no cover - subprocess failure
                raise TextToSpeechError(f"espeak-ng failed: {exc}") from exc

            if result.returncode != 0 or result.stdout is None:
                log.warning("espeak-ng returned code %s; falling back to direct playback", result.returncode)
                self._play_direct(base_cmd, text)
                return

            try:
                self._play_wave_bytes(result.stdout, apply_effects=True)
                return
            except TextToSpeechError as exc:
                log.warning("SoX filter playback failed (%s); using direct espeak output", exc)
                self._play_direct(base_cmd, text)
                return

        self._play_direct(base_cmd, text)

    def _play_wave_bytes(self, audio_bytes: bytes, apply_effects: bool) -> None:
        if self._sox_bin is None:
            raise TextToSpeechError("SoX executable not available")

        cmd = [
            self._sox_bin,
            "-q",
            "-t",
            "wav",
            "-",
        ] + self._sox_destination

        if apply_effects and self._effects_enabled and self._sox_effects:
            cmd += self._sox_effects

        log.debug("Running SoX playback: %s", " ".join(shlex.quote(part) for part in cmd))
        try:
            result = subprocess.run(
                cmd,
                input=audio_bytes,
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as exc:  # pragma: no cover - subprocess failure
            raise TextToSpeechError(f"SoX playback failed: {exc}") from exc

        if result.returncode != 0:
            raise TextToSpeechError(f"SoX exited with code {result.returncode}")

    def _play_direct(self, espeak_cmd: list[str], text: str) -> None:
        cmd = espeak_cmd + [text]
        log.debug("Running TTS (direct espeak): %s", " ".join(shlex.quote(part) for part in cmd))
        try:
            subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:  # pragma: no cover - subprocess failure
            raise TextToSpeechError(f"Failed to invoke espeak-ng: {exc}") from exc


def build_text_to_speech(settings) -> Optional[TextToSpeech]:
    try:
        tts = TextToSpeech(settings)
    except Exception as exc:
        log.error("Failed to initialise TTS: %s", exc)
        return None
    return tts
