"""A stand-in for the STM32 station, for running the receiver without hardware.

Produces the same $WS frames the firmware sends, driven by a simple weather
model: a daily temperature cycle, humidity that falls as it warms up, and
pressure that drifts over a few days. With corrupt_rate > 0 it also mangles
some lines the way a noisy serial link does, to exercise the receiver's
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


class WeatherModel:
    def __init__(self, seed: Optional[int] = None):
        # Only the noise is random; the slow curves depend on wall-clock time
        # alone, so backfilled history joins up with live frames.
        self._rng = random.Random(seed)

    def sample(self, t: float) -> tuple[float, float, float, float]:
        """Return (temperature C, humidity %, pressure hPa, second temperature C) at unix time t."""
        lt = time.localtime(t)
        hour = lt.tm_hour + lt.tm_min / 60 + lt.tm_sec / 3600
        rng = self._rng

        # Coolest around 05:00, warmest around 15:00
        temp = 17.0 + 5.5 * math.sin(2 * math.pi * (hour - 9) / 24) + rng.gauss(0, 0.08)
        # Warmer air holds more water, so relative humidity drops in the afternoon
        hum = 58.0 - 2.6 * (temp - 17.0) + rng.gauss(0, 0.4)
        hum = min(max(hum, 12.0), 98.0)
        # Weather systems: about a 3.5 day swing of +/- 7 hPa
        days = t / 86400
        press = 1012.0 + 7.0 * math.sin(2 * math.pi * days / 3.5)
        press += rng.gauss(0, 0.05)
        # The STTS751 sits next to the other parts and reads a touch warm
        temp2 = temp + 0.35 + rng.gauss(0, 0.06)
        return temp, hum, press, temp2


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
) -> Iterator[tuple[float, str]]:
    """Yield (timestamp, line) pairs forever, starting with the firmware's boot banner."""
    model = WeatherModel(seed)
    rng = random.Random(seed)
    t = time.time() if start is None else start
    yield t, "# weather-station up (simulated)\r\n"
    seq = 0
    while True:
        line = encode_frame(seq, *model.sample(t))
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
