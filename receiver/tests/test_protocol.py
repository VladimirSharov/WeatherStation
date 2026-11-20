import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import simulator  # noqa: E402
from protocol import (  # noqa: E402
    FAULT_PRESSURE,
    FAULT_TEMP2,
    FAULT_TEMP_HUM,
    FrameError,
    checksum,
    encode_frame,
    parse_frame,
)


class ParseFrameTests(unittest.TestCase):
    def test_round_trip_full_station(self):
        line = encode_frame(1, 42, 23.51, 45.12, 1013.25, 23.87)
        r = parse_frame(line, received_at=1.0)
        self.assertEqual((r.station, r.seq), (1, 42))
        self.assertAlmostEqual(r.temperature_c, 23.51)
        self.assertAlmostEqual(r.humidity_pct, 45.12)
        self.assertAlmostEqual(r.pressure_hpa, 1013.25)
        self.assertAlmostEqual(r.temperature2_c, 23.87)
        self.assertEqual(r.status, 0)
        self.assertEqual(r.received_at, 1.0)

    def test_station_without_optional_sensors(self):
        line = encode_frame(2, 17, 12.34, 65.5)
        self.assertTrue(line.startswith("$WS,2,17,1234,6550,,,0*"))
        r = parse_frame(line)
        self.assertEqual(r.station, 2)
        self.assertAlmostEqual(r.temperature_c, 12.34)
        self.assertIsNone(r.pressure_hpa)
        self.assertIsNone(r.temperature2_c)
        self.assertEqual(r.status, 0, "a sensor that isn't fitted is not a fault")

    def test_matches_firmware_format(self):
        # Built by hand the way FormatFrame() in station/Src/main.c builds it
        body = "WS,1,7,-512,9000,98765,-480,0"
        line = f"${body}*{checksum(body):02X}\r\n"
        self.assertEqual(line, encode_frame(1, 7, -5.12, 90.0, 987.65, -4.80))
        self.assertAlmostEqual(parse_frame(line).temperature_c, -5.12)

    def test_comments_and_blanks_are_ignored(self):
        self.assertIsNone(parse_frame("# weather-station up\r\n"))
        self.assertIsNone(parse_frame("\r\n"))

    def test_bad_checksum_rejected(self):
        line = encode_frame(1, 1, 20.0, 50.0, 1000.0, 20.0)
        tampered = line.replace("2000", "2900", 1)
        with self.assertRaisesRegex(FrameError, "checksum"):
            parse_frame(tampered)

    def test_truncated_frame_rejected(self):
        line = encode_frame(1, 1, 20.0, 50.0, 1000.0, 20.0)
        with self.assertRaises(FrameError):
            parse_frame(line[:15])

    def test_line_noise_rejected(self):
        with self.assertRaises(FrameError):
            parse_frame("��$W�S,,1*ZZ\r\n")

    def test_original_firmware_output_rejected(self):
        """What the 2021 prototype firmware actually put on the wire.

        It printed the float's raw bits with %d and sent the whole 20-byte
        buffer, NULs and leftovers included.
        """
        bits = struct.unpack("<i", struct.pack("<f", 23.5))[0]
        self.assertEqual(bits, 1102839808)
        buf = f"\n{bits}\n\r".encode().ljust(20, b"\x00").decode()
        for line in buf.splitlines():
            if line.strip("\x00").strip():
                with self.assertRaises(FrameError):
                    parse_frame(line)

    def test_legacy_2021_pi_format_rejected(self):
        """The fixed-position line legacy/2021-pi/readdata.py sliced apart."""
        with self.assertRaises(FrameError):
            parse_frame("T1: 23 H1: 45% T2: 22 H2: 50%\r\n")

    def test_status_bits_null_out_values(self):
        r = parse_frame(encode_frame(1, 3, 0, 0, 1000.0, 21.0, status=FAULT_TEMP_HUM))
        self.assertIsNone(r.temperature_c)
        self.assertIsNone(r.humidity_pct)
        self.assertAlmostEqual(r.pressure_hpa, 1000.0)
        self.assertEqual(r.faults, ["temperature/humidity"])

    def test_out_of_range_values_flagged(self):
        r = parse_frame(encode_frame(1, 4, 400.0, 50.0, 5.0, 21.0))
        self.assertEqual(r.status, FAULT_TEMP_HUM | FAULT_PRESSURE)
        self.assertIsNone(r.temperature_c)
        self.assertIsNone(r.pressure_hpa)
        self.assertAlmostEqual(r.temperature2_c, 21.0)
        self.assertFalse(r.status & FAULT_TEMP2)

    def test_docstring_example_is_valid(self):
        import protocol
        example = "$WS,2,17,1234,6550,,,0*" + f"{checksum('WS,2,17,1234,6550,,,0'):02X}"
        self.assertIn(example, protocol.__doc__)


class SimulatorTests(unittest.TestCase):
    def test_both_stations_report_with_own_sequence(self):
        gen = simulator.frames(seed=1, realtime=False, start=1_700_000_000)
        self.assertIsNone(parse_frame(next(gen)[1]))  # boot banner
        for expected_seq in range(300):
            indoor = parse_frame(next(gen)[1])
            outdoor = parse_frame(next(gen)[1])
            self.assertEqual((indoor.station, indoor.seq), (1, expected_seq))
            self.assertEqual((outdoor.station, outdoor.seq), (2, expected_seq))
            self.assertIsNotNone(indoor.pressure_hpa)
            self.assertIsNone(outdoor.pressure_hpa)
            self.assertEqual(indoor.status | outdoor.status, 0)

    def test_corrupted_frames_are_rejected(self):
        gen = simulator.frames(seed=2, corrupt_rate=0.5, realtime=False, start=1_700_000_000)
        next(gen)
        rejected = 0
        for _ in range(1000):
            try:
                parse_frame(next(gen)[1])
            except FrameError:
                rejected += 1
        self.assertGreater(rejected, 350)


class ReceiverTests(unittest.TestCase):
    def setUp(self):
        import receiver
        self.receiver = receiver

    def test_rejected_line_is_not_also_counted_as_dropped(self):
        stats = self.receiver.LinkStats()
        stats.accepted(1, 0)
        stats.accepted(2, 0)
        stats.rejected("$WS,1,1,garbage", "missing checksum")  # station 1's seq 1
        stats.accepted(2, 1)
        stats.accepted(1, 2)
        self.assertEqual(stats.frames_dropped, 0)
        stats.accepted(2, 5)  # seqs 2..4 never arrived at all
        self.assertEqual(stats.frames_dropped, 3)

    def test_restart_detected_per_station(self):
        stats = self.receiver.LinkStats()
        stats.accepted(1, 10)
        stats.accepted(2, 3)
        stats.accepted(1, 0)
        self.assertEqual(stats.station_restarts, 1)

    def test_store_keeps_stations_apart(self):
        store = self.receiver.Store(":memory:")
        lines = simulator.frames(seed=3, realtime=False, start=1_700_000_000, period=60)
        next(lines)
        store.insert_many([parse_frame(next(lines)[1], received_at=1_700_000_000 + i * 30)
                           for i in range(40)])
        latest = store.latest()
        self.assertEqual([s["station"] for s in latest], [1, 2])
        hist = store.history(1_700_000_000, 1_700_000_000 + 1200)
        self.assertEqual(set(hist), {1, 2})
        self.assertTrue(all(p["press"] is None for p in hist[2]))
        self.assertTrue(all(p["press"] is not None for p in hist[1]))


if __name__ == "__main__":
    unittest.main()
