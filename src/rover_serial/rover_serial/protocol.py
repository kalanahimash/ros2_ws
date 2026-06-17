"""Serial packet encoder/decoder matching the Arduino firmware protocol."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class Packet:
    command: str
    value: str


def encode(command: str, value: str) -> bytes:
    """Encode a command/value pair as a serial packet."""
    return f"{command}:{value}\n".encode("ascii")


def decode(line: str) -> Optional[Packet]:
    """Decode a raw serial line into a Packet. Returns None on malformed input."""
    line = line.strip()
    if not line or ":" not in line:
        return None
    sep = line.index(":")
    cmd = line[:sep].strip()
    val = line[sep + 1:].strip()
    if not cmd:
        return None
    return Packet(command=cmd, value=val)


def encode_motor(left: int, right: int) -> bytes:
    """Encode a MOT command clamping speeds to [-255, 255]."""
    left  = max(-255, min(255, int(left)))
    right = max(-255, min(255, int(right)))
    return encode("MOT", f"{left},{right}")


def encode_pan(degrees: float) -> bytes:
    return encode("PAN", str(int(degrees)))


def encode_tilt(degrees: float) -> bytes:
    return encode("TILT", str(int(degrees)))


def encode_stop() -> bytes:
    return encode("STOP", "0")


def encode_estop() -> bytes:
    return encode("ESTOP", "0")


def encode_reset() -> bytes:
    return encode("RESET", "0")


def encode_heartbeat() -> bytes:
    return encode("HB", "1")


def parse_stat(value: str) -> Optional[dict]:
    """Parse a STAT packet value string into a dict."""
    parts = value.split(",")
    if len(parts) < 6:
        return None
    try:
        return {
            "mode":          parts[0],
            "left_speed":    int(parts[1]),
            "right_speed":   int(parts[2]),
            "pan_angle":     int(parts[3]),
            "tilt_angle":    int(parts[4]),
            "battery_raw":   int(parts[5]),
        }
    except (ValueError, IndexError):
        return None


def battery_raw_to_voltage(raw: int) -> float:
    """Convert ADC raw (0–1023) to voltage assuming 3:1 divider, 5V reference."""
    return raw * (15.0 / 1023.0)
