import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import simulator  # noqa: E402
from protocol import (  # noqa: E402
    FAULT_HTS221,
    FAULT_LPS22HH,
    FAULT_STTS751,
    FrameError,
    checksum,
    encode_frame,
    parse_frame,
)


class ParseFrameTests(unittest.TestCase):
    def test_round_trip(self):
        line = encode_frame(42, 23.51, 45.12, 1013.25, 23.87)
        r = parse_frame(line, received_at=1.0)
        self.assertEqual(r.seq, 42)
        self.assertAlmostEqual(r.temperature_c, 23.51)
        self.assertAlmostEqual(r.humidity_pct, 45.12)
        self.assertAlmostEqual(r.pressure_hpa, 1013.25)
        self.assertAlmostEqual(r.temperature2_c, 23.87)
        self.assertEqual(r.status, 0)
        self.assertEqual(r.received_at, 1.0)

    def test_matches_firmware_format(self):
        # Built by hand the way FormatFrame() in station/Src/main.c builds it
        body = "WS,7,-512,9000,98765,-480,0"
        line = f"${body}*{checksum(body):02X}\r\n"
        self.assertEqual(line, encode_frame(7, -5.12, 90.0, 987.65, -4.80))
        r = parse_frame(line)
        self.assertAlmostEqual(r.temperature_c, -5.12)

    def test_comments_and_blanks_are_ignored(self):
        self.assertIsNone(parse_frame("# weather-station up\r\n"))
        self.assertIsNone(parse_frame("\r\n"))

    def test_bad_checksum_rejected(self):
        line = encode_frame(1, 20.0, 50.0, 1000.0, 20.0)
        tampered = line.replace("2000", "2900", 1)
        with self.assertRaisesRegex(FrameError, "checksum"):
            parse_frame(tampered)

    def test_truncated_frame_rejected(self):
        line = encode_frame(1, 20.0, 50.0, 1000.0, 20.0)
        with self.assertRaises(FrameError):
            parse_frame(line[:15])

    def test_line_noise_rejected(self):
        with self.assertRaises(FrameError):
            parse_frame("��$W�S,,1*ZZ\r\n")

    def test_original_firmware_output_rejected(self):
        """What the 2021 firmware actually put on the wire.

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

    def test_status_bits_null_out_values(self):
        r = parse_frame(encode_frame(3, 0, 0, 1000.0, 21.0, status=FAULT_HTS221))
        self.assertIsNone(r.temperature_c)
        self.assertIsNone(r.humidity_pct)
        self.assertAlmostEqual(r.pressure_hpa, 1000.0)
        self.assertEqual(r.faulted_sensors, ["HTS221"])

    def test_out_of_range_values_flagged(self):
        r = parse_frame(encode_frame(4, 400.0, 50.0, 5.0, 21.0))
        self.assertEqual(r.status, FAULT_HTS221 | FAULT_LPS22HH)
        self.assertIsNone(r.temperature_c)
        self.assertIsNone(r.pressure_hpa)
        self.assertAlmostEqual(r.temperature2_c, 21.0)
        self.assertFalse(r.status & FAULT_STTS751)


class SimulatorTests(unittest.TestCase):
    def test_clean_frames_all_parse(self):
        gen = simulator.frames(seed=1, realtime=False, start=1_700_000_000)
        self.assertIsNone(parse_frame(next(gen)[1]))  # boot banner
        for expected_seq in range(500):
            r = parse_frame(next(gen)[1])
            self.assertEqual(r.seq, expected_seq)
            self.assertEqual(r.status, 0)

    def test_corrupted_frames_never_slip_through_as_wrong_data(self):
        gen = simulator.frames(seed=2, corrupt_rate=0.5, realtime=False, start=1_700_000_000)
        next(gen)
        rejected = 0
        for _ in range(1000):
            try:
                parse_frame(next(gen)[1])
            except FrameError:
                rejected += 1
        self.assertGreater(rejected, 350)


if __name__ == "__main__":
    unittest.main()
