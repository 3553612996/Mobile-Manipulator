#!/usr/bin/env python3
"""Self-tests for the host-side protocol codec.

The vectors come straight from docs/architecture/protocol.md section 10. Run with:

    python tools/host_gui/test_protocol.py

No pytest needed: it is a plain unittest module so it runs in the same bare
environment as the GUI.
"""

from __future__ import annotations

import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from host_gui import protocol as p  # noqa: E402


class CrcTests(unittest.TestCase):
    def test_check_value(self):
        data, expected = p.VECTOR_CRC
        self.assertEqual(p.crc16_ccitt(data), expected)

    def test_incremental_matches_one_shot(self):
        data = bytes(range(256)) * 3
        crc = 0xFFFF
        for byte in data:
            crc = p._crc_step(crc, byte)
        self.assertEqual(crc, p.crc16_ccitt(data))


class FrameVectorTests(unittest.TestCase):
    def test_set_velocity_vector(self):
        frame = p.build_frame(p.Cmd.SET_VELOCITY, 1,
                              p.set_velocity_payload(300, 500))
        self.assertEqual(frame, p.VECTOR_VELOCITY)

    def test_set_enable_vector(self):
        frame = p.build_frame(p.Cmd.SET_ENABLE, 2,
                              p.set_enable_payload(True))
        self.assertEqual(frame, p.VECTOR_ENABLE)

    def test_estop_vector(self):
        frame = p.build_frame(p.Cmd.ESTOP, 3)
        self.assertEqual(frame, p.VECTOR_ESTOP)

    def test_parse_velocity_fields(self):
        frames = p.FrameParser().feed(p.VECTOR_VELOCITY)
        self.assertEqual(len(frames), 1)
        frame = frames[0]
        self.assertEqual(frame.cmd, p.Cmd.SET_VELOCITY)
        self.assertEqual(frame.seq, 1)
        self.assertEqual(frame.payload, bytes.fromhex("2C 01 F4 01"))

    def test_parse_estop_shortest_frame(self):
        frame = p.FrameParser().feed(p.VECTOR_ESTOP)[0]
        self.assertEqual(frame.cmd, p.Cmd.ESTOP)
        self.assertEqual(frame.payload, b"")


class ParserRobustnessTests(unittest.TestCase):
    def test_byte_at_a_time_equals_bulk(self):
        stream = (p.VECTOR_VELOCITY + p.VECTOR_ENABLE + p.VECTOR_ESTOP)
        bulk = p.FrameParser().feed(stream)
        parser = p.FrameParser()
        drip = []
        for byte in stream:
            drip.extend(parser.feed(bytes((byte,))))
        self.assertEqual([f.raw for f in bulk], [f.raw for f in drip])

    def test_bad_crc_is_dropped(self):
        bad = bytearray(p.VECTOR_VELOCITY)
        bad[-1] ^= 0xFF
        parser = p.FrameParser()
        self.assertEqual(parser.feed(bytes(bad)), [])
        self.assertEqual(parser.crc_errors, 1)
        # The next good frame must still parse.
        self.assertEqual(len(parser.feed(p.VECTOR_ENABLE)), 1)

    def test_bad_len_is_dropped_without_corruption(self):
        for length in (0x00, 0x01, 0x43, 0xFF):
            parser = p.FrameParser()
            junk = bytes((0xAA, 0x55, length, 0x01, 0x01))
            self.assertEqual(parser.feed(junk), [])
            self.assertEqual(len(parser.feed(p.VECTOR_ESTOP)), 1,
                             f"LEN=0x{length:02X} broke resync")

    def test_bad_len_byte_may_start_next_frame(self):
        # 0xAA is an invalid LEN and also the next frame's first head byte.
        parser = p.FrameParser()
        frames = parser.feed(bytes((0xAA, 0x55, 0xAA, 0x55)) + p.VECTOR_ESTOP)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].cmd, p.Cmd.ESTOP)

    def test_random_stream_never_raises(self):
        rng = random.Random(1234)
        parser = p.FrameParser()
        for _ in range(2000):
            chunk = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 40)))
            parser.feed(chunk)
        # A valid frame still parses after all that noise.
        self.assertEqual(len(parser.feed(p.VECTOR_VELOCITY)), 1)

    def test_reset_discards_half_frame(self):
        parser = p.FrameParser()
        self.assertEqual(parser.feed(p.VECTOR_VELOCITY[:5]), [])
        parser.reset()  # the link calls this after a 50 ms gap
        self.assertEqual(len(parser.feed(p.VECTOR_VELOCITY)), 1)


class AckAndReportTests(unittest.TestCase):
    def test_ack_fields(self):
        raw = p.build_frame(p.Cmd.SET_ENABLE | p.ACK_FLAG, 7,
                            bytes((2, p.Result.OK)))
        frame = p.FrameParser().feed(raw)[0]
        self.assertTrue(frame.is_ack)
        self.assertEqual(frame.acked_cmd, p.Cmd.SET_ENABLE)
        self.assertEqual(frame.acked_seq, 2)
        self.assertEqual(frame.result, p.Result.OK)

    def test_status_round_trip(self):
        # state 2, fault 0x0501, 1252 mV, 32 C, 50 us
        payload = bytes.fromhex("02 01 05 E4 04 20 32 00")
        status = p.decode_status(payload)
        self.assertEqual(status, {"state": 2, "fault": 0x0501,
                                  "battery_mv": 0x04E4, "temp_c": 32,
                                  "jitter_us": 0x0032})

    def test_arm_round_trip(self):
        import struct
        angles = (0, -100, 200, -300, 400, -500)
        payload = struct.pack("<6hhB", *angles, 850, 1)
        self.assertEqual(len(payload), 15)
        arm = p.decode_arm(payload)
        self.assertEqual(tuple(arm["angles_cdeg"]), angles)
        self.assertEqual(arm["gripper_pos"], 850)
        self.assertEqual(arm["gripper_state"], 1)

    def test_wheel_speed_round_trip(self):
        import struct
        raw = p.build_frame(p.Report.WHEEL_SPEED, 4,
                            struct.pack("<4h", 100, -200, 300, -400))
        frame = p.FrameParser().feed(raw)[0]
        self.assertEqual(p.decode_wheel_speed(frame.payload),
                         {"fl_mm_s": 100, "fr_mm_s": -200,
                          "rl_mm_s": 300, "rr_mm_s": -400})

    def test_wheel_duty_round_trip(self):
        import struct
        raw = p.build_frame(p.Report.WHEEL_DUTY, 5,
                            struct.pack("<4h", 400, -400, 0, 25))
        frame = p.FrameParser().feed(raw)[0]
        self.assertEqual(p.decode_wheel_duty(frame.payload),
                         {"fl_permille": 400, "fr_permille": -400,
                          "rl_permille": 0, "rr_permille": 25})

    def test_test_duty_ids_are_named(self):
        # 0x30..0x33 must not collide with the gain ids 0x20..0x2B.
        self.assertEqual(p.TEST_DUTY_IDS, (0x30, 0x31, 0x32, 0x33))
        for param_id in p.TEST_DUTY_IDS:
            self.assertIn(param_id, p.PARAM_NAMES)
            self.assertFalse(any(param_id in ids for ids in p.PID_PARAM_IDS))

    def test_pid_param_ids_are_named(self):
        for wheel, ids in enumerate(p.PID_PARAM_IDS):
            for gain, param_id in enumerate(ids):
                self.assertEqual(param_id, 0x20 + wheel * 3 + gain)
                self.assertIn(param_id, p.PARAM_NAMES)

    def test_set_param_payload(self):
        payload = p.set_param_payload(0x2B, 1500)
        self.assertEqual(len(payload), 5)
        self.assertEqual(payload[0], 0x2B)
        import struct
        self.assertEqual(struct.unpack("<i", payload[1:])[0], 1500)

    def test_short_report_is_rejected(self):
        self.assertIsNone(p.decode_status(b"\x01\x02"))
        self.assertIsNone(p.decode_odom(b""))
        self.assertIsNone(p.decode_wheel_speed(b"\x00" * 7))
        self.assertIsNone(p.decode_wheel_duty(b"\x00" * 9))
        self.assertIsNone(p.decode_storage(b"\x00" * 23))
        self.assertIsNone(p.decode_log(b"\x00" * 14))
        self.assertIsNone(p.decode_crash(b"\x00" * 6))


class StorageTests(unittest.TestCase):
    def test_param_save_vector(self):
        frame = p.build_frame(p.Cmd.PARAM_SAVE, 4, p.param_save_payload(0))
        self.assertEqual(frame, p.VECTOR_PARAM_SAVE)

    def test_storage_query_vector(self):
        self.assertEqual(p.build_frame(p.Cmd.STORAGE_QUERY, 5),
                         p.VECTOR_STORAGE_QUERY)

    def test_log_export_vector(self):
        frame = p.build_frame(p.Cmd.LOG_EXPORT, 6,
                              p.log_export_payload(0, 1, 0))
        self.assertEqual(frame, p.VECTOR_LOG_EXPORT)

    def test_param_reset_requires_confirm(self):
        self.assertEqual(p.param_reset_payload(1), bytes((1, 0xA5)))

    def test_storage_round_trip(self):
        import struct
        raw = p.build_frame(p.Report.STORAGE, 7, struct.pack(
            "<BBHHHIIII", 0x03, 1, 7, 3, 0, 42, 1000, 5, 99))
        frame = p.FrameParser().feed(raw)[0]
        st = p.decode_storage(frame.payload)
        self.assertEqual(st["param_seq"], 7)
        self.assertEqual(st["calib_seq"], 3)
        self.assertEqual(st["boot_total"], 42)
        self.assertFalse(st["dirty"])
        self.assertTrue(st["param_valid"])
        self.assertFalse(st["save_failed"])

    def test_log_round_trip(self):
        import struct
        head = struct.pack("<BBBBIIHB", 0, 1, 6, 0x01, 123, 9, 0, 5)
        raw = p.build_frame(p.Report.LOG, 8, head + b"hello")
        frame = p.FrameParser().feed(raw)[0]
        rec = p.decode_log(frame.payload)
        self.assertEqual(rec["text"], "hello")
        self.assertEqual(rec["seq"], 9)
        self.assertTrue(rec["more"])
        self.assertFalse(rec["end"])

    def test_log_end_frame(self):
        import struct
        head = struct.pack("<BBBBIIHB", 0, 1, 6, 0x02, 124, 10, 0, 0)
        rec = p.decode_log(head)
        self.assertTrue(rec["end"])
        self.assertEqual(rec["text"], "")

    def test_crash_chunk_round_trip(self):
        import struct
        data = bytes(range(57))
        head = struct.pack("<BBHHB", 2, 0x01, 0, 1024, 57)
        rec = p.decode_crash(head + data)
        self.assertEqual(rec["slot"], 2)
        self.assertEqual(rec["offset"], 0)
        self.assertEqual(rec["total_len"], 1024)
        self.assertEqual(rec["data"], data)
        self.assertTrue(rec["more"])

    def test_calib_param_ids_are_named(self):
        # Deadzone, encoder scale and wheel direction belong to calibration.
        self.assertIn(0x01, p.CALIB_PARAM_IDS)
        self.assertIn(0x02, p.CALIB_PARAM_IDS)
        for param_id in range(0x50, 0x60):
            self.assertIn(param_id, p.CALIB_PARAM_IDS)
            self.assertIn(param_id, p.PARAM_NAMES)
        self.assertNotIn(0x30, p.CALIB_PARAM_IDS)
        self.assertIn(0x40, p.PARAM_NAMES)
        self.assertIn(0x41, p.PARAM_NAMES)


if __name__ == "__main__":
    unittest.main(verbosity=2)
