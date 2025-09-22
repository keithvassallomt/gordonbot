from __future__ import annotations

import logging
import os
import shlex
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Sequence

log = logging.getLogger(__name__)


class AudioCuePlayer:
    """Plays a short audio cue using an external command (e.g., ffplay)."""

    def __init__(
        self,
        path: str | os.PathLike[str],
        command_template: str,
        *,
        enabled: bool = True,
        throttle_seconds: float = 0.75,
    ) -> None:
        self._path = Path(path)
        self._command_template = command_template.strip()
        self._enabled = enabled
        self._throttle = max(0.0, throttle_seconds)
        self._last_play = 0.0
        self._cmd_missing_logged = False
        self._path_missing_logged = False

    def play(self, *, blocking: bool = False) -> None:
        if not self._enabled:
            return
        if not self._path.exists():
            if not self._path_missing_logged:
                log.warning("Wake-word audio path does not exist: %s", self._path)
                self._path_missing_logged = True
            return

        now = time.monotonic()
        if self._throttle and now - self._last_play < self._throttle:
            return
        self._last_play = now

        cmd = self._build_command()
        if not cmd:
            return

        if blocking:
            self._run_command(cmd)
        else:
            threading.Thread(target=self._run_command, args=(cmd,), daemon=True).start()

    # ------------------------------------------------------------------
    def _build_command(self) -> Sequence[str] | None:
        if "{path}" in self._command_template:
            cmd_str = self._command_template.format(path=str(self._path))
        else:
            cmd_str = f"{self._command_template} {self._path}"

        try:
            cmd = shlex.split(cmd_str)
        except ValueError as exc:
            log.error("Invalid wake-word audio command '%s': %s", self._command_template, exc)
            return None

        if not cmd:
            log.error("Wake-word audio command is empty after parsing")
            return None

        executable = cmd[0]
        if not shutil.which(executable):
            if not self._cmd_missing_logged:
                log.warning("Wake-word audio command not found: %s", executable)
                self._cmd_missing_logged = True
            return None

        return cmd

    @staticmethod
    def _run_command(cmd: Sequence[str]) -> None:  # pragma: no cover - subprocess
        try:
            subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:
            log.error("Wake-word audio command failed: %s", exc)
