"""Small cross-process PID lock used to prevent concurrent Q4 controllers."""

from __future__ import annotations

import os
from pathlib import Path


class AlreadyRunningError(RuntimeError):
    pass


class PidLock:
    def __init__(self, path: Path) -> None:
        self.path = path.resolve()
        self.owned = False

    @staticmethod
    def _alive(pid: int) -> bool:
        if pid <= 0:
            return False
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

    def acquire(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        for _ in range(2):
            try:
                descriptor = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            except FileExistsError:
                try:
                    pid = int(self.path.read_text(encoding="ascii").strip())
                except (OSError, ValueError):
                    pid = -1
                if self._alive(pid):
                    raise AlreadyRunningError(f"another Q4 process owns {self.path} (pid={pid})")
                try:
                    self.path.unlink()
                except FileNotFoundError:
                    pass
                continue
            with os.fdopen(descriptor, "w", encoding="ascii") as stream:
                stream.write(str(os.getpid()))
            self.owned = True
            return
        raise AlreadyRunningError(f"could not acquire {self.path}")

    def release(self) -> None:
        if not self.owned:
            return
        try:
            if self.path.read_text(encoding="ascii").strip() == str(os.getpid()):
                self.path.unlink()
        except FileNotFoundError:
            pass
        finally:
            self.owned = False

