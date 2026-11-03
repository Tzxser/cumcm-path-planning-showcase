"""Constants stated by the official problem and communication appendix."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class OfficialRules:
    arena_id: str = "default"
    arena_radius_m: float = 1800.0
    channel_min: int = 1
    channel_max: int = 20
    source_count_min: int = 10
    source_count_max: int = 16
    signal_radius_min_m: float = 1000.0
    signal_radius_max_m: float = 1500.0
    direction_half_angle_deg: float = 90.0
    bearing_error_deg: float = 1.0
    near_radius_m: float = 5.0
    clear_radius_m: float = 20.0
    speed_mps: float = 5.0
    measure_time_s: float = 5.0
    switch_time_s: float = 1.0
    clear_success_time_s: float = 5.0
    clear_failure_time_s: float = 3.0
    coordinate_limit: float = 2_000_000.0
    virtual_time_limit_s: float = 360_000.0
    real_time_guard_s: float = 30.0
    request_timeout_s: float = 8.0
    network_retries: int = 3
    retry_backoff_s: float = 0.25


DEFAULT_RULES = OfficialRules()
