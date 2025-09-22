from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, Iterable

from app.services.diagnostics import get_cpu_temperature

log = logging.getLogger(__name__)


Action = Callable[[str, float], None]


@dataclass
class IntentRule:
    keywords: Iterable[str]
    action: Action
    description: str

    def matches(self, text: str) -> bool:
        lowered = text.lower()
        return all(token in lowered for token in self.keywords)


def _report_cpu_temperature(_: str, __: float) -> None:
    try:
        temp = get_cpu_temperature()
    except Exception as exc:
        log.error("Failed to read CPU temperature: %s", exc)
        print("Could not read CPU temperature")
        return

    if temp is None:
        print("CPU temperature unavailable")
        return

    print(f"CPU Temperature: {temp:.1f}°C")


def _offload_to_chatgpt(text: str, confidence: float) -> None:
    print(f"ChatGPT offload needed (confidence {confidence:.2f}): {text}")


RULES: list[IntentRule] = [
    IntentRule(
        keywords=("cpu", "temperature"),
        action=_report_cpu_temperature,
        description="Report CPU temperature",
    ),
]


def handle_transcript(text: str, confidence: float) -> None:
    for rule in RULES:
        if rule.matches(text):
            log.debug("Voice intent matched: %s", rule.description)
            rule.action(text, confidence)
            return

    _offload_to_chatgpt(text, confidence)
