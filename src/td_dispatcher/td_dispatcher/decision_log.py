"""Журнал решений диспетчера.

Каждая команда, каждый результат и каждая смена фазы записываются строкой
JSON. Это одновременно инструмент разбора неудачной попытки и материал для
технической защиты: по журналу видно, почему система поступила именно так.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

__all__ = ["DecisionLog"]


class DecisionLog:
    """Пишет события в JSONL и держит хвост в памяти для быстрых запросов."""

    def __init__(self, path: str | Path | None = None, keep_last: int = 200) -> None:
        self.path = Path(path) if path else None
        self.keep_last = keep_last
        self._records: list[dict[str, Any]] = []
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, **fields: Any) -> dict[str, Any]:
        record = {
            "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "event": event,
            **fields,
        }
        self._records.append(record)
        del self._records[: -self.keep_last]

        if self.path is not None:
            try:
                with self.path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            except OSError:
                # Журнал полезен, но миссия важнее: сбой записи не должен
                # прерывать выполнение.
                pass
        return record

    @property
    def records(self) -> list[dict[str, Any]]:
        return list(self._records)

    def summary(self, limit: int = 10) -> list[str]:
        """Короткая выжимка последних событий для показа в интерфейсе."""
        out = []
        for record in self._records[-limit:]:
            event = record.get("event", "")
            if event == "command":
                out.append(f"{record.get('agent')}: {record.get('skill')} — {record.get('reason')}")
            elif event == "outcome":
                status = "успех" if record.get("success") else "неудача"
                out.append(f"{record.get('agent')}: {record.get('skill')} — {status}")
            elif event == "phase":
                out.append(f"фаза {record.get('phase')}: {record.get('reason', '')}")
            else:
                out.append(f"{event}: {record.get('message', '')}")
        return out
