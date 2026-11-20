"""Weather station receiver: serial -> SQLite -> web dashboard.

Runs on the Raspberry Pi with the Nucleo board plugged in over USB (the
ST-LINK exposes the station's UART as /dev/ttyACM0):

    python receiver.py --serial /dev/ttyACM0

Or with no hardware at all, using the built-in station simulator:

    python receiver.py --simulate --backfill-hours 24 --corrupt-rate 0.03

Then open http://<pi-address>:8080/.
"""

from __future__ import annotations

import argparse
import collections
import logging
import sqlite3
import threading
import time
from pathlib import Path
from typing import Iterator, Optional

from flask import Flask, jsonify, request, send_from_directory

import simulator
from protocol import FrameError, Reading, parse_frame

log = logging.getLogger("receiver")

STATIC_DIR = Path(__file__).resolve().parent / "static"
STALE_AFTER_S = 10.0
MAX_HISTORY_POINTS = 400


# --------------------------------------------------------------------------- #
# Storage
# --------------------------------------------------------------------------- #
class Store:
    def __init__(self, path: str):
        self._db = sqlite3.connect(path, check_same_thread=False)
        self._lock = threading.Lock()
        with self._lock:
            self._db.executescript(
                """
                CREATE TABLE IF NOT EXISTS readings (
                    ts      REAL NOT NULL,
                    seq     INTEGER NOT NULL,
                    temp    REAL,
                    hum     REAL,
                    press   REAL,
                    temp2   REAL,
                    status  INTEGER NOT NULL
                );
                CREATE INDEX IF NOT EXISTS readings_ts ON readings(ts);
                """
            )

    def insert(self, r: Reading) -> None:
        with self._lock:
            self._db.execute(
                "INSERT INTO readings VALUES (?, ?, ?, ?, ?, ?, ?)",
                (r.received_at, r.seq, r.temperature_c, r.humidity_pct,
                 r.pressure_hpa, r.temperature2_c, r.status),
            )
            self._db.commit()

    def insert_many(self, readings: list[Reading]) -> None:
        with self._lock:
            self._db.executemany(
                "INSERT INTO readings VALUES (?, ?, ?, ?, ?, ?, ?)",
                [(r.received_at, r.seq, r.temperature_c, r.humidity_pct,
                  r.pressure_hpa, r.temperature2_c, r.status) for r in readings],
            )
            self._db.commit()

    def latest(self) -> Optional[dict]:
        with self._lock:
            row = self._db.execute(
                "SELECT ts, seq, temp, hum, press, temp2, status FROM readings "
                "ORDER BY ts DESC LIMIT 1"
            ).fetchone()
        if row is None:
            return None
        keys = ("ts", "seq", "temp", "hum", "press", "temp2", "status")
        return dict(zip(keys, row))

    def history(self, since: float, until: float) -> list[dict]:
        """Averaged into at most MAX_HISTORY_POINTS buckets so the payload stays small."""
        bucket = max((until - since) / MAX_HISTORY_POINTS, 1.0)
        with self._lock:
            rows = self._db.execute(
                """
                SELECT CAST(ts / :b AS INTEGER) * :b + :b / 2 AS t,
                       AVG(temp), AVG(hum), AVG(press), AVG(temp2)
                FROM readings WHERE ts >= :since AND ts <= :until
                GROUP BY CAST(ts / :b AS INTEGER) ORDER BY t
                """,
                {"b": bucket, "since": since, "until": until},
            ).fetchall()
        return [
            {"ts": t, "temp": _r(a), "hum": _r(b), "press": _r(c), "temp2": _r(d)}
            for t, a, b, c, d in rows
        ]

    def pressure_near(self, ts: float, window: float = 600.0) -> Optional[float]:
        with self._lock:
            row = self._db.execute(
                "SELECT AVG(press) FROM readings WHERE ts BETWEEN ? AND ?",
                (ts - window / 2, ts + window / 2),
            ).fetchone()
        return row[0] if row else None

    def prune(self, older_than: float) -> int:
        with self._lock:
            cur = self._db.execute("DELETE FROM readings WHERE ts < ?", (older_than,))
            self._db.commit()
            return cur.rowcount


def _r(v: Optional[float], nd: int = 2) -> Optional[float]:
    return None if v is None else round(v, nd)


# --------------------------------------------------------------------------- #
# Link statistics
# --------------------------------------------------------------------------- #
class LinkStats:
    def __init__(self):
        self._lock = threading.Lock()
        self.frames_ok = 0
        self.frames_rejected = 0
        self.frames_dropped = 0
        self.station_restarts = 0
        self.last_seq: Optional[int] = None
        self._rejected_since_ok = 0  # rejected lines also leave a seq gap; don't count them twice
        self.started_at = time.time()
        self.recent_rejects: collections.deque = collections.deque(maxlen=12)

    def accepted(self, seq: int) -> None:
        with self._lock:
            if self.last_seq is not None:
                if seq <= self.last_seq:
                    self.station_restarts += 1
                    log.info("sequence went %d -> %d: station restarted", self.last_seq, seq)
                else:
                    lost = seq - self.last_seq - 1 - self._rejected_since_ok
                    if lost > 0:
                        self.frames_dropped += lost
                        log.warning("lost %d frame(s) before seq %d", lost, seq)
            self.last_seq = seq
            self._rejected_since_ok = 0
            self.frames_ok += 1

    def rejected(self, line: str, reason: str) -> None:
        with self._lock:
            self.frames_rejected += 1
            self._rejected_since_ok += 1
            self.recent_rejects.appendleft(
                {"ts": time.time(), "line": line.rstrip("\r\n")[:80], "reason": reason}
            )

    def snapshot(self) -> dict:
        with self._lock:
            total = self.frames_ok + self.frames_rejected
            return {
                "frames_ok": self.frames_ok,
                "frames_rejected": self.frames_rejected,
                "frames_dropped": self.frames_dropped,
                "station_restarts": self.station_restarts,
                "link_quality_pct": round(100 * self.frames_ok / total, 1) if total else None,
                "uptime_s": round(time.time() - self.started_at),
                "recent_rejects": list(self.recent_rejects),
            }


# --------------------------------------------------------------------------- #
# Line sources
# --------------------------------------------------------------------------- #
def serial_lines(port: str, baud: int) -> Iterator[tuple[float, str]]:
    import serial  # pyserial; imported here so --simulate works without it

    backoff = 1.0
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                log.info("listening on %s @ %d baud", port, baud)
                backoff = 1.0
                while True:
                    raw = ser.readline()
                    if raw:
                        # 'replace' keeps line noise visible in the logs instead of crashing
                        yield time.time(), raw.decode("ascii", errors="replace")
        except serial.SerialException as exc:
            log.error("serial error on %s: %s (retrying in %.0fs)", port, exc, backoff)
            time.sleep(backoff)
            backoff = min(backoff * 2, 30.0)


def ingest(lines: Iterator[tuple[float, str]], store: Store, stats: LinkStats,
           retention_days: float) -> None:
    next_prune = 0.0
    for ts, line in lines:
        try:
            reading = parse_frame(line, received_at=ts)
        except FrameError as exc:
            stats.rejected(line, str(exc))
            log.warning("rejected %r: %s", line.rstrip("\r\n"), exc)
            continue
        if reading is None:
            if line.strip():
                log.info("station: %s", line.strip().lstrip("# "))
            continue
        stats.accepted(reading.seq)
        store.insert(reading)
        if reading.status:
            log.warning("seq %d: sensor fault %s", reading.seq, ", ".join(reading.faulted_sensors))

        if ts >= next_prune:
            removed = store.prune(ts - retention_days * 86400)
            if removed:
                log.info("pruned %d readings older than %g days", removed, retention_days)
            next_prune = ts + 3600


def backfill(store: Store, hours: float, period: float = 60.0) -> None:
    """Seed the database with simulated history so the charts aren't empty on first run."""
    start = time.time() - hours * 3600
    readings = []
    for ts, line in simulator.frames(period=period, seed=7, realtime=False, start=start):
        if ts >= time.time():
            break
        r = parse_frame(line, received_at=ts)
        if r is not None:
            readings.append(r)
    store.insert_many(readings)
    log.info("backfilled %d simulated readings over %g h", len(readings), hours)


# --------------------------------------------------------------------------- #
# Web API
# --------------------------------------------------------------------------- #
def create_app(store: Store, stats: LinkStats, source: str) -> Flask:
    app = Flask(__name__, static_folder=None)

    @app.get("/")
    def index():
        return send_from_directory(STATIC_DIR, "index.html")

    @app.get("/api/latest")
    def latest():
        now = time.time()
        reading = store.latest()
        trend = None
        if reading and reading["press"] is not None:
            past = store.pressure_near(reading["ts"] - 3 * 3600)
            if past is not None:
                trend = round(reading["press"] - past, 2)
        return jsonify(
            source=source,
            now=now,
            reading=reading,
            stale=reading is None or now - reading["ts"] > STALE_AFTER_S,
            pressure_trend_3h=trend,
            link=stats.snapshot(),
        )

    @app.get("/api/history")
    def history():
        hours = min(max(request.args.get("hours", default=6.0, type=float), 0.05), 24 * 30)
        now = time.time()
        return jsonify(hours=hours, points=store.history(now - hours * 3600, now))

    return app


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--serial", metavar="PORT", help="serial port, e.g. /dev/ttyACM0 or COM5")
    src.add_argument("--simulate", action="store_true", help="use the built-in station simulator")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--db", default="weather.db", help="SQLite file (default: %(default)s)")
    ap.add_argument("--host", default="0.0.0.0", help="dashboard bind address (default: %(default)s)")
    ap.add_argument("--port", type=int, default=8080, help="dashboard port (default: %(default)s)")
    ap.add_argument("--retention-days", type=float, default=30)
    ap.add_argument("--corrupt-rate", type=float, default=0.0,
                    help="simulator only: fraction of frames to garble (default: %(default)s)")
    ap.add_argument("--backfill-hours", type=float, default=0,
                    help="simulator only: seed this many hours of history on start")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s")
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    store = Store(args.db)
    stats = LinkStats()

    if args.simulate:
        if args.backfill_hours > 0 and store.latest() is None:
            backfill(store, args.backfill_hours)
        lines = simulator.frames(corrupt_rate=args.corrupt_rate)
        source = "simulator"
    else:
        lines = serial_lines(args.serial, args.baud)
        source = args.serial

    threading.Thread(
        target=ingest, args=(lines, store, stats, args.retention_days), daemon=True, name="ingest"
    ).start()

    log.info("dashboard on http://%s:%d/", args.host, args.port)
    create_app(store, stats, source).run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
