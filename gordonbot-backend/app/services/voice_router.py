from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, Iterable

import requests

from app.services.diagnostics import get_cpu_temperature
from app.core.config import settings

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


RULES: list[IntentRule] = [
    IntentRule(
        keywords=("cpu", "temperature"),
        action=_report_cpu_temperature,
        description="Report CPU temperature",
    ),
]


def handle_transcript(text: str, confidence: float) -> str | None:
    for rule in RULES:
        if rule.matches(text):
            log.debug("Voice intent matched: %s", rule.description)
            return rule.action(text, confidence)

    return _offload_to_chatgpt(text, confidence)
