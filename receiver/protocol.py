"""Wire protocol shared by the station firmware and the receiver.

One ASCII line per sample, 115200 8N1:

    $WS,<seq>,<temp_cC>,<hum_cPct>,<press_Pa>,<temp2_cC>,<status>*<XOR>\\r\\n

All values are integers (fixed point) so the firmware never needs printf("%f").
The checksum is the XOR of every byte between '$' and '*', as two hex digits,
the same scheme NMEA GPS sentences use. Lines starting with '#' are comments.

See station/Src/main.c for the sending side.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from typing import Optional

FAULT_HTS221 = 1 << 0
FAULT_LPS22HH = 1 << 1
FAULT_STTS751 = 1 << 2

SENSOR_NAMES = {
    FAULT_HTS221: "HTS221",
    FAULT_LPS22HH: "LPS22HH",
    FAULT_STTS751: "STTS751",
}

# Datasheet operating ranges. A frame can pass the checksum and still carry
# nonsense (an uninitialised sensor, a register read half-way through a
# conversion), so values outside these ranges are treated as a sensor fault.
TEMP_RANGE_C = (-40.0, 85.0)
HUMIDITY_RANGE_PCT = (0.0, 100.0)
PRESSURE_RANGE_HPA = (260.0, 1260.0)

HEADER = "WS"
FIELD_COUNT = 7  # WS + six values


class FrameError(ValueError):
    """Raised for a line that is not a valid frame (noise, truncation, bad checksum)."""


@dataclass(frozen=True)
class Reading:
    seq: int
    temperature_c: Optional[float]
    humidity_pct: Optional[float]
    pressure_hpa: Optional[float]
    temperature2_c: Optional[float]
    status: int
    received_at: float

    @property
    def faulted_sensors(self) -> list[str]:
        return [name for bit, name in SENSOR_NAMES.items() if self.status & bit]

    def to_dict(self) -> dict:
        d = asdict(self)
        d["faulted_sensors"] = self.faulted_sensors
        return d


def checksum(body: str) -> int:
    value = 0
    for byte in body.encode("ascii"):
        value ^= byte
    return value


def encode_frame(
    seq: int,
    temperature_c: float,
    humidity_pct: float,
    pressure_hpa: float,
    temperature2_c: float,
    status: int = 0,
) -> str:
    """Build a frame exactly as the firmware does. Used by the simulator and tests."""
    body = ",".join(
        str(v)
        for v in (
            HEADER,
            seq,
            round(temperature_c * 100),
            round(humidity_pct * 100),
            round(pressure_hpa * 100),
            round(temperature2_c * 100),
            status,
        )
    )
    return f"${body}*{checksum(body):02X}\r\n"


def _in_range(value: float, bounds: tuple[float, float]) -> bool:
    return bounds[0] <= value <= bounds[1]


def parse_frame(line: str, received_at: Optional[float] = None) -> Optional[Reading]:
    """Parse one line from the station.

    Returns None for blank and comment lines, a Reading for a valid frame, and
    raises FrameError for anything else.
    """
    text = line.strip()
    if not text or text.startswith("#"):
        return None
    if not text.startswith("$" + HEADER + ","):
        raise FrameError("no $WS header")

    body, sep, cs_text = text[1:].rpartition("*")
    if not sep or len(cs_text) != 2:
        raise FrameError("missing checksum")
    try:
        expected = int(cs_text, 16)
    except ValueError:
        raise FrameError(f"checksum {cs_text!r} is not hex") from None
    actual = checksum(body) if body.isascii() else -1
    if actual != expected:
        raise FrameError(f"checksum mismatch (got {actual:02X}, frame says {expected:02X})")

    fields = body.split(",")
    if len(fields) != FIELD_COUNT:
        raise FrameError(f"expected {FIELD_COUNT} fields, got {len(fields)}")
    try:
        seq, t_cc, h_cp, p_pa, t2_cc, status = (int(f) for f in fields[1:])
    except ValueError:
        raise FrameError("non-integer field") from None

    temp = t_cc / 100
    hum = h_cp / 100
    press = p_pa / 100
    temp2 = t2_cc / 100

    if not status & FAULT_HTS221 and not (
        _in_range(temp, TEMP_RANGE_C) and _in_range(hum, HUMIDITY_RANGE_PCT)
    ):
        status |= FAULT_HTS221
    if not status & FAULT_LPS22HH and not _in_range(press, PRESSURE_RANGE_HPA):
        status |= FAULT_LPS22HH
    if not status & FAULT_STTS751 and not _in_range(temp2, TEMP_RANGE_C):
        status |= FAULT_STTS751

    return Reading(
        seq=seq,
        temperature_c=None if status & FAULT_HTS221 else temp,
        humidity_pct=None if status & FAULT_HTS221 else hum,
        pressure_hpa=None if status & FAULT_LPS22HH else press,
        temperature2_c=None if status & FAULT_STTS751 else temp2,
        status=status,
        received_at=time.time() if received_at is None else received_at,
    )
