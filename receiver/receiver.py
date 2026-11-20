"""Weather station receiver: serial -> SQLite -> web dashboard.

Runs on the Raspberry Pi with the station's Nucleo board plugged in over USB
(the ST-LINK exposes the board's UART as /dev/ttyACM0):

    python receiver.py --serial /dev/ttyACM0 --names "1=Indoor,2=Outdoor"

Or with no hardware at all, using the built-in two-station simulator:

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
COLUMNS = ("station", "ts", "seq", "temp", "hum", "press", "temp2", "status")


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
                    station INTEGER NOT NULL,
                    ts      REAL NOT NULL,
                    seq     INTEGER NOT NULL,
                    temp    REAL,
                    hum     REAL,
                    press   REAL,
                    temp2   REAL,
                    status  INTEGER NOT NULL
                );
                CREATE INDEX IF NOT EXISTS readings_station_ts ON readings(station, ts);
                """
            )

    @staticmethod
    def _row(r: Reading) -> tuple:
        return (r.station, r.received_at, r.seq, r.temperature_c, r.humidity_pct,
                r.pressure_hpa, r.temperature2_c, r.status)

    def insert(self, r: Reading) -> None:
        self.insert_many([r])

    def insert_many(self, readings: list[Reading]) -> None:
        with self._lock:
            self._db.executemany(
                "INSERT INTO readings VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                [self._row(r) for r in readings],
            )
            self._db.commit()

    def latest(self) -> list[dict]:
        """Newest reading of every station that has ever reported."""
        with self._lock:
            rows = self._db.execute(
                f"""
                SELECT {", ".join("r." + c for c in COLUMNS)} FROM readings r
                JOIN (SELECT station, MAX(ts) AS ts FROM readings GROUP BY station) m
                  ON r.station = m.station AND r.ts = m.ts
                ORDER BY r.station
                """
            ).fetchall()
        return [dict(zip(COLUMNS, row)) for row in rows]

    def history(self, since: float, until: float) -> dict[int, list[dict]]:
        """Per-station series, averaged into at most MAX_HISTORY_POINTS buckets."""
        bucket = max((until - since) / MAX_HISTORY_POINTS, 1.0)
        with self._lock:
            rows = self._db.execute(
                """
                SELECT station, CAST(ts / :b AS INTEGER) * :b + :b / 2 AS t,
                       AVG(temp), AVG(hum), AVG(press), AVG(temp2)
                FROM readings WHERE ts >= :since AND ts <= :until
                GROUP BY station, CAST(ts / :b AS INTEGER) ORDER BY station, t
                """,
                {"b": bucket, "since": since, "until": until},
            ).fetchall()
        out: dict[int, list[dict]] = {}
        for station, t, a, b, c, d in rows:
            out.setdefault(station, []).append(
                {"ts": t, "temp": _r(a), "hum": _r(b), "press": _r(c), "temp2": _r(d)}
            )
        return out

    def pressure_near(self, station: int, ts: float, window: float = 600.0) -> Optional[float]:
        with self._lock:
            row = self._db.execute(
                "SELECT AVG(press) FROM readings WHERE station = ? AND ts BETWEEN ? AND ?",
                (station, ts - window / 2, ts + window / 2),
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
        self.per_station: dict[int, dict] = {}
        # A rejected line also leaves a seq gap at whichever station sent it.
        # We can't tell which station that was, so the next gap anywhere absorbs
        # it rather than being counted a second time as "dropped".
        self._unattributed_rejects = 0
        self.started_at = time.time()
        self.recent_rejects: collections.deque = collections.deque(maxlen=12)

    def accepted(self, station: int, seq: int) -> None:
        with self._lock:
            st = self.per_station.setdefault(station, {"frames_ok": 0, "last_seq": None})
            last = st["last_seq"]
            if last is not None:
                if seq <= last:
                    self.station_restarts += 1
                    log.info("station %d: sequence went %d -> %d, station restarted", station, last, seq)
                else:
                    gap = seq - last - 1
                    absorbed = min(gap, self._unattributed_rejects)
                    self._unattributed_rejects -= absorbed
                    if gap - absorbed > 0:
                        self.frames_dropped += gap - absorbed
                        log.warning("station %d: lost %d frame(s) before seq %d", station, gap - absorbed, seq)
            st["last_seq"] = seq
            st["frames_ok"] += 1
            self.frames_ok += 1

    def rejected(self, line: str, reason: str) -> None:
        with self._lock:
            self.frames_rejected += 1
            self._unattributed_rejects += 1
            self.recent_rejects.appendleft(
                {"ts": time.time(), "line": line.rstrip("\r\n")[:80], "reason": reason}
            )

    def station_frames(self, station: int) -> int:
        with self._lock:
            return self.per_station.get(station, {}).get("frames_ok", 0)

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
        stats.accepted(reading.station, reading.seq)
        store.insert(reading)
        if reading.status:
            log.warning("station %d seq %d: %s sensor fault", reading.station, reading.seq,
                        ", ".join(reading.faults))

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
def create_app(store: Store, stats: LinkStats, source: str, names: dict[int, str]) -> Flask:
    app = Flask(__name__, static_folder=None)

    @app.get("/")
    def index():
        return send_from_directory(STATIC_DIR, "index.html")

    @app.get("/api/latest")
    def latest():
        now = time.time()
        stations = []
        for reading in store.latest():
            sid = reading.pop("station")
            trend = None
            if reading["press"] is not None:
                past = store.pressure_near(sid, reading["ts"] - 3 * 3600)
                if past is not None:
                    trend = round(reading["press"] - past, 2)
            stations.append({
                "id": sid,
                "name": names.get(sid, f"Station {sid}"),
                "reading": reading,
                "stale": now - reading["ts"] > STALE_AFTER_S,
                "pressure_trend_3h": trend,
                "frames_ok": stats.station_frames(sid),
            })
        return jsonify(source=source, now=now, stations=stations, link=stats.snapshot())

    @app.get("/api/history")
    def history():
        hours = min(max(request.args.get("hours", default=6.0, type=float), 0.05), 24 * 30)
        now = time.time()
        series = store.history(now - hours * 3600, now)
        return jsonify(hours=hours, stations={str(k): v for k, v in series.items()})

    return app


def parse_names(text: str) -> dict[int, str]:
    """'1=Indoor,2=Outdoor' -> {1: 'Indoor', 2: 'Outdoor'}"""
    names = {}
    for part in filter(None, (p.strip() for p in text.split(","))):
        sid, sep, name = part.partition("=")
        if not sep or not sid.strip().isdigit() or not name.strip():
            raise argparse.ArgumentTypeError(f"bad station name {part!r}, expected ID=NAME")
        names[int(sid)] = name.strip()
    return names


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--serial", metavar="PORT", help="serial port, e.g. /dev/ttyACM0 or COM5")
    src.add_argument("--simulate", action="store_true", help="use the built-in two-station simulator")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--names", type=parse_names, default=None, metavar="ID=NAME,...",
                    help='station display names, e.g. "1=Indoor,2=Outdoor"')
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
        if args.backfill_hours > 0 and not store.latest():
            backfill(store, args.backfill_hours)
        lines = simulator.frames(corrupt_rate=args.corrupt_rate)
        source = "simulator"
        names = dict(simulator.DEFAULT_NAMES)
    else:
        lines = serial_lines(args.serial, args.baud)
        source = args.serial
        names = {}
    names.update(args.names or {})

    threading.Thread(
        target=ingest, args=(lines, store, stats, args.retention_days), daemon=True, name="ingest"
    ).start()

    log.info("dashboard on http://%s:%d/", args.host, args.port)
    create_app(store, stats, source, names).run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
