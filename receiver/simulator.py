"""A stand-in for the stations, for running the receiver without hardware.

Simulates two stations sending the same $WS frames the firmware does:

  1  indoor   the IKS01A3 board: temperature, humidity, pressure, 2nd temperature
  2  outdoor  a temperature/humidity-only station, like the 2021 discrete-sensor build

The weather is a simple model: a daily temperature cycle outdoors, a heated room
indoors that follows it weakly, relative humidity that falls as air warms, and
pressure that drifts over a few days. With corrupt_rate > 0 some lines get
mangled the way a noisy serial link mangles them, to exercise the receiver's
checksum rejection.

Run it on its own to see the frames:

    python simulator.py
"""

from __future__ import annotations

import math
import random
import time
from typing import Iterator, Optional

from protocol import encode_frame

INDOOR, OUTDOOR = 1, 2
DEFAULT_NAMES = {INDOOR: "Indoor", OUTDOOR: "Outdoor"}


class WeatherModel:
    """Only the noise is random. The slow curves depend on wall-clock time
    alone, so backfilled history joins up with live frames."""

    def __init__(self, seed: Optional[int] = None):
        self._rng = random.Random(seed)

    def outdoor(self, t: float) -> tuple[float, float]:
        lt = time.localtime(t)
        hour = lt.tm_hour + lt.tm_min / 60 + lt.tm_sec / 3600
        # Coolest around 05:00, warmest around 15:00
        temp = 9.0 + 4.5 * math.sin(2 * math.pi * (hour - 9) / 24) + self._rng.gauss(0, 0.1)
        # Warmer air holds more water, so relative humidity drops in the afternoon
        hum = 82.0 - 3.2 * (temp - 9.0) + self._rng.gauss(0, 0.6)
        return temp, min(max(hum, 20.0), 99.0)

    def indoor(self, t: float) -> tuple[float, float]:
        lt = time.localtime(t)
        hour = lt.tm_hour + lt.tm_min / 60 + lt.tm_sec / 3600
        # Heated room: warmer in the evening, setback overnight
        temp = 21.5 + 1.2 * math.sin(2 * math.pi * (hour - 12) / 24) + self._rng.gauss(0, 0.05)
        hum = 41.0 - 1.5 * (temp - 21.5) + self._rng.gauss(0, 0.3)
        return temp, hum

    def pressure(self, t: float) -> float:
        # Weather systems: about a 3.5 day swing of +/- 7 hPa
        return 1012.0 + 7.0 * math.sin(2 * math.pi * (t / 86400) / 3.5) + self._rng.gauss(0, 0.05)

    def frame(self, station: int, seq: int, t: float) -> str:
        if station == INDOOR:
            temp, hum = self.indoor(t)
            # The STTS751 sits next to the other parts and reads a touch warm
            temp2 = temp + 0.35 + self._rng.gauss(0, 0.06)
            return encode_frame(INDOOR, seq, temp, hum, self.pressure(t), temp2)
        temp, hum = self.outdoor(t)
        return encode_frame(OUTDOOR, seq, temp, hum)


def _corrupt(line: str, rng: random.Random) -> str:
    kind = rng.choice(("flip", "truncate", "noise"))
    if kind == "flip":
        i = rng.randrange(1, len(line) - 5)
        chars = list(line)
        chars[i] = chr(ord(chars[i]) ^ (1 << rng.randrange(7)))
        return "".join(chars)
    if kind == "truncate":
        return line[: rng.randrange(4, len(line) - 4)] + "\r\n"
    return "".join(chr(rng.randrange(0x80, 0x100)) for _ in range(rng.randrange(4, 16))) + "\r\n"


def frames(
    period: float = 2.0,
    corrupt_rate: float = 0.0,
    seed: Optional[int] = None,
    realtime: bool = True,
    start: Optional[float] = None,
    stations: tuple[int, ...] = (INDOOR, OUTDOOR),
) -> Iterator[tuple[float, str]]:
    """Yield (timestamp, line) pairs forever: a boot banner, then one frame per
    station every period, each station with its own sequence counter."""
    model = WeatherModel(seed)
    rng = random.Random(seed)
    t = time.time() if start is None else start
    yield t, "# weather-station up (simulated)\r\n"
    seq = 0
    while True:
        for station in stations:
            line = model.frame(station, seq, t)
            if corrupt_rate and rng.random() < corrupt_rate:
                line = _corrupt(line, rng)
            yield t, line
        seq += 1
        if realtime:
            time.sleep(period)
            t = time.time()
        else:
            t += period


if __name__ == "__main__":
    try:
        for _, line in frames(corrupt_rate=0.1):
            print(line, end="", flush=True)
    except KeyboardInterrupt:
        pass
