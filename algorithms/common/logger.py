"""Durable, privacy-redacted per-run logging."""

from __future__ import annotations

import json
import logging
import os
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REDACTED = "[REDACTED]"


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def redact(value: Any) -> Any:
    if isinstance(value, dict):
        return {
            key: (REDACTED if key.lower() in {"robot_id", "password", "token", "secret"} else redact(item))
            for key, item in value.items()
        }
    if isinstance(value, list):
        return [redact(item) for item in value]
    if isinstance(value, tuple):
        return [redact(item) for item in value]
    if isinstance(value, str):
        runtime_id = os.environ.get("CUMCM_Q4_ROBOT_ID")
        return value.replace(runtime_id, REDACTED) if runtime_id else value
    return value


class PrivateFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        # Exception tracebacks may echo a server error containing the runtime ID.
        return redact(super().format(record))


def dump_json(value: Any, *, indent: int | None = None) -> str:
    return json.dumps(redact(value), ensure_ascii=False, allow_nan=False, sort_keys=True, indent=indent)


class RunLogger:
    def __init__(self, output_base: Path, strategy: str, level: str = "INFO") -> None:
        root = output_base.resolve() / strategy
        root.mkdir(parents=True, exist_ok=True)
        label = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + "-" + uuid.uuid4().hex[:10]
        self.run_dir = root / label
        self.run_dir.mkdir(parents=False, exist_ok=False)
        self.actions_path = self.run_dir / "actions.jsonl"
        self.decisions_path = self.run_dir / "decisions.jsonl"
        self._actions_file = self.actions_path.open("a", encoding="utf-8", newline="\n")
        self._decisions_file = self.decisions_path.open("a", encoding="utf-8", newline="\n")
        self.actions: list[dict[str, Any]] = []
        self.strategy = strategy
        self.console_path = self.run_dir / "console.log"
        self.console = self._build_console(level)

    def _build_console(self, level_name: str) -> logging.Logger:
        level = getattr(logging, level_name.upper(), None)
        if not isinstance(level, int):
            raise ValueError(f"invalid log level: {level_name}")
        logger = logging.getLogger(f"q4.{self.strategy}.{os.getpid()}.{self.run_dir.name}")
        logger.setLevel(level)
        logger.propagate = False
        formatter = PrivateFormatter("%(asctime)sZ %(levelname)s %(message)s", "%Y-%m-%dT%H:%M:%S")
        formatter.converter = time.gmtime
        for handler in (logging.StreamHandler(sys.stdout), logging.FileHandler(self.console_path, encoding="utf-8")):
            handler.setLevel(level)
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        return logger

    def record_action(self, record: dict[str, Any]) -> None:
        clean = redact(record)
        self.actions.append(clean)
        self._actions_file.write(dump_json(clean) + "\n")
        self._actions_file.flush()

    def decision(self, event: str, **fields: Any) -> None:
        clean = {"wall_time": utc_now_iso(), "strategy": self.strategy, "event": event, **fields}
        self._decisions_file.write(dump_json(clean) + "\n")
        self._decisions_file.flush()

    def write_json(self, name: str, value: Any) -> Path:
        path = self.run_dir / name
        path.write_text(dump_json(value, indent=2) + "\n", encoding="utf-8")
        return path

    def close(self) -> None:
        if not self._actions_file.closed:
            self._actions_file.close()
        if not self._decisions_file.closed:
            self._decisions_file.close()
        for handler in list(self.console.handlers):
            handler.flush()
            handler.close()
            self.console.removeHandler(handler)

    def __enter__(self) -> "RunLogger":
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()
