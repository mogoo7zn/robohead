"""SkillResult — the single unified result object of all Skills."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from core.model.enums import SkillStatus


@dataclass
class SkillResult:
    status: SkillStatus = SkillStatus.RUNNING
    message: str = ""
    data: dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def running(message: str = "", **data: Any) -> "SkillResult":
        return SkillResult(SkillStatus.RUNNING, message, dict(data))

    @staticmethod
    def success(message: str = "", **data: Any) -> "SkillResult":
        return SkillResult(SkillStatus.SUCCESS, message, dict(data))

    @staticmethod
    def failure(message: str = "", **data: Any) -> "SkillResult":
        return SkillResult(SkillStatus.FAILURE, message, dict(data))

    @staticmethod
    def timeout(message: str = "", **data: Any) -> "SkillResult":
        return SkillResult(SkillStatus.TIMEOUT, message, dict(data))

    @staticmethod
    def cancelled(message: str = "", **data: Any) -> "SkillResult":
        return SkillResult(SkillStatus.CANCELLED, message, dict(data))

    @property
    def is_done(self) -> bool:
        return self.status is not SkillStatus.RUNNING

    def __str__(self) -> str:
        return f"SkillResult({self.status.value}: {self.message})"
