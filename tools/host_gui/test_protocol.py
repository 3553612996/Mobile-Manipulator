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

    def test_short_report_is_rejected(self):
        self.assertIsNone(p.decode_status(b"\x01\x02"))
        self.assertIsNone(p.decode_odom(b""))


if __name__ == "__main__":
    unittest.main(verbosity=2)
