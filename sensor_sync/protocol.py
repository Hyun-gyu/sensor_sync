"""Helpers for the Teensy <-> ROS sync control protocol."""

from __future__ import annotations

from typing import Dict, Optional


VALID_SYNC_MODES = (
    "gps_pps",
    "internal_timer",
    "external_trigger",
)

_MODE_ALIASES = {
    "g": "gps_pps",
    "gps": "gps_pps",
    "gps_pps": "gps_pps",
    "gpspps": "gps_pps",
    "pps": "gps_pps",
    "i": "internal_timer",
    "internal": "internal_timer",
    "internal_timer": "internal_timer",
    "timer": "internal_timer",
    "e": "external_trigger",
    "external": "external_trigger",
    "external_trigger": "external_trigger",
}

_MODE_TO_DEVICE_TOKEN = {
    "gps_pps": "GPS_PPS",
    "internal_timer": "INTERNAL",
    "external_trigger": "EXTERNAL",
}

_BOOL_FIELDS = {"running", "pps_locked"}
_INT_FIELDS = {"pps_age_ms", "stereo_frames", "pps_count"}
_FLOAT_FIELDS = {"fps"}


def normalize_sync_mode(mode: str) -> str:
    """Normalize user/device sync mode strings."""
    key = (mode or "").strip().lower()
    if key not in _MODE_ALIASES:
        raise ValueError(
            f"Unsupported sync mode '{mode}'. Valid modes: {', '.join(VALID_SYNC_MODES)}"
        )
    return _MODE_ALIASES[key]


def build_mode_command(mode: str) -> str:
    """Build the command string expected by the Teensy firmware."""
    canonical_mode = normalize_sync_mode(mode)
    return f"MODE {_MODE_TO_DEVICE_TOKEN[canonical_mode]}"


def parse_state_line(line: str) -> Optional[Dict[str, object]]:
    """Parse a structured firmware STATE line into typed values."""
    if not line.startswith("STATE "):
        return None

    parsed: Dict[str, object] = {}
    for item in line[len("STATE ") :].split():
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        if key == "mode":
            parsed[key] = normalize_sync_mode(value)
        elif key in _BOOL_FIELDS:
            parsed[key] = value == "1"
        elif key in _INT_FIELDS:
            parsed[key] = int(value)
        elif key in _FLOAT_FIELDS:
            parsed[key] = float(value)
        else:
            parsed[key] = value
    return parsed


def build_status_summary(
    *,
    connected: bool,
    running: bool,
    mode: str,
    fps: float,
    pps_locked: bool,
    pps_age_ms: int,
) -> str:
    """Build a compact human-readable status string."""
    state_bits = ["connected" if connected else "disconnected"]
    if connected:
        state_bits.append("running" if running else "stopped")
        state_bits.append(f"mode={normalize_sync_mode(mode)}")
        state_bits.append(f"fps={fps:.2f}")
        state_bits.append(f"pps_locked={int(bool(pps_locked))}")
        state_bits.append(f"pps_age_ms={pps_age_ms}")
    return ", ".join(state_bits)
