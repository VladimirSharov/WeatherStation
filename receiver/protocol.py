"""Wire protocol shared by the station firmware and the receiver.

One ASCII line per sample, 115200 8N1:

    $WS,<station>,<seq>,<temp_cC>,<hum_cPct>,<press_Pa>,<temp2_cC>,<status>*<XOR>\\r\\n

All values are integers (fixed point), so the firmware never needs printf("%f").
An empty value field means the station has no such sensor fitted. The IKS01A3
station fills all four; a station with only a temperature/humidity sensor
sends e.g. ``$WS,2,17,1234,6550,,,0*2E``.

The checksum is the XOR of every byte between '$' and '*', as two hex digits,
the same scheme NMEA GPS sentences use. Lines starting with '#' are comments.

See station/Src/main.c for the sending side.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from typing import Optional

# Status bits: one per sensor slot, set when a fitted sensor failed to read.
FAULT_TEMP_HUM = 1 << 0
FAULT_PRESSURE = 1 << 1
FAULT_TEMP2 = 1 << 2

SLOT_NAMES = {
    FAULT_TEMP_HUM: "temperature/humidity",
    FAULT_PRESSURE: "pressure",
    FAULT_TEMP2: "second temperature",
}

# Sensor operating ranges. A frame can pass the checksum and still carry
# nonsense (an uninitialised sensor, a register read mid-conversion), so values
# outside these ranges are treated as a sensor fault.
TEMP_RANGE_C = (-40.0, 85.0)
HUMIDITY_RANGE_PCT = (0.0, 100.0)
PRESSURE_RANGE_HPA = (260.0, 1260.0)

HEADER = "WS"
FIELD_COUNT = 8  # WS, station, seq, four values, status


class FrameError(ValueError):
    """Raised for a line that is not a valid frame (noise, truncation, bad checksum)."""


@dataclass(frozen=True)
class Reading:
    station: int
    seq: int
    temperature_c: Optional[float]
    humidity_pct: Optional[float]
    pressure_hpa: Optional[float]
    temperature2_c: Optional[float]
    status: int
    received_at: float

    @property
    def faults(self) -> list[str]:
        return [name for bit, name in SLOT_NAMES.items() if self.status & bit]

    def to_dict(self) -> dict:
        d = asdict(self)
        d["faults"] = self.faults
        return d


def checksum(body: str) -> int:
    value = 0
    for byte in body.encode("ascii"):
        value ^= byte
    return value


def _fixed(value: Optional[float], scale: int) -> str:
    return "" if value is None else str(round(value * scale))


def encode_frame(
    station: int,
    seq: int,
    temperature_c: Optional[float],
    humidity_pct: Optional[float],
    pressure_hpa: Optional[float] = None,
    temperature2_c: Optional[float] = None,
    status: int = 0,
) -> str:
    """Build a frame exactly as the firmware does. Used by the simulator and tests."""
    body = ",".join(
        (
            HEADER,
            str(station),
            str(seq),
            _fixed(temperature_c, 100),
            _fixed(humidity_pct, 100),
            _fixed(pressure_hpa, 100),
            _fixed(temperature2_c, 100),
            str(status),
        )
    )
    return f"${body}*{checksum(body):02X}\r\n"


def _in_range(value: Optional[float], bounds: tuple[float, float]) -> bool:
    return value is None or bounds[0] <= value <= bounds[1]


def _int_or_none(field: str) -> Optional[int]:
    return None if field == "" else int(field)


def parse_frame(line: str, received_at: Optional[float] = None) -> Optional[Reading]:
    """Parse one line from a station.

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
        station, seq, status = int(fields[1]), int(fields[2]), int(fields[7])
        t_cc, h_cp, p_pa, t2_cc = (_int_or_none(f) for f in fields[3:7])
    except ValueError:
        raise FrameError("non-integer field") from None

    temp = None if t_cc is None else t_cc / 100
    hum = None if h_cp is None else h_cp / 100
    press = None if p_pa is None else p_pa / 100
    temp2 = None if t2_cc is None else t2_cc / 100

    if not (_in_range(temp, TEMP_RANGE_C) and _in_range(hum, HUMIDITY_RANGE_PCT)):
        status |= FAULT_TEMP_HUM
    if not _in_range(press, PRESSURE_RANGE_HPA):
        status |= FAULT_PRESSURE
    if not _in_range(temp2, TEMP_RANGE_C):
        status |= FAULT_TEMP2

    return Reading(
        station=station,
        seq=seq,
        temperature_c=None if status & FAULT_TEMP_HUM else temp,
        humidity_pct=None if status & FAULT_TEMP_HUM else hum,
        pressure_hpa=None if status & FAULT_PRESSURE else press,
        temperature2_c=None if status & FAULT_TEMP2 else temp2,
        status=status,
        received_at=time.time() if received_at is None else received_at,
    )
