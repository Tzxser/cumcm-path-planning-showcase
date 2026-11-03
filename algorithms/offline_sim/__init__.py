"""Offline Q4 simulator; its case distribution is an explicit modeling assumption."""

from .simulator import OfflineSession, Source, generate_case

__all__ = ["OfflineSession", "Source", "generate_case"]
