#!/usr/bin/env python3
from __future__ import annotations

import math
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "third_party" / "rs06_mit_control"))

import read_all_motor_angles as monitor  # noqa: E402
import verify_all_can_joint_mapping as verify_mapping  # noqa: E402
from wd_mcu_protocol import (  # noqa: E402
    CONTROL_FLAG_CALIBRATION_READONLY,
    CONTROL_FLAG_DRY_RUN,
    CONTROL_FLAG_READONLY_POLL,
    ZERO_COMMANDS,
    build_setpoint_packet,
)


class FeedbackMappingTests(unittest.TestCase):
    @staticmethod
    def feedback(base: int, online_values: dict[int, float]) -> SimpleNamespace:
        joints = [SimpleNamespace(online=0, q=0.0) for _ in range(16)]
        for index, value in online_values.items():
            joints[index] = SimpleNamespace(online=1, q=value)
        return SimpleNamespace(bus_base=base, joints=joints)

    def test_mcu_a_maps_global_can1_and_can2(self) -> None:
        positions = monitor.positions_from_feedback(
            self.feedback(0, {0: 0.25, 7: -1.5, 8: 99.0})
        )
        self.assertEqual(positions, {(0, 1): 0.25, (1, 4): -1.5})

    def test_mcu_b_maps_global_can3_and_can4(self) -> None:
        positions = monitor.positions_from_feedback(
            self.feedback(2, {8: 1.25, 15: 2.5, 0: 99.0})
        )
        self.assertEqual(positions, {(2, 1): 1.25, (3, 4): 2.5})


class AngleStateTests(unittest.TestCase):
    def test_single_turn_wrap_is_continuous(self) -> None:
        state = monitor.AngleState()
        state.update(6.1, 1.0, 2.0 * math.pi)
        state.update(0.2, 2.0, 2.0 * math.pi)
        self.assertAlmostEqual(state.delta_rad, 2.0 * math.pi - 6.1 + 0.2)

    def test_continuous_mech_pos_is_not_forced_to_wrap(self) -> None:
        state = monitor.AngleState()
        state.update(6.1, 1.0, 2.0 * math.pi)
        state.update(6.4, 2.0, 2.0 * math.pi)
        self.assertAlmostEqual(state.delta_rad, 0.3)


class CalibrationPacketTests(unittest.TestCase):
    def test_sets_dryrun_readonly_and_calibration_flags(self) -> None:
        packet = build_setpoint_packet(
            1,
            ZERO_COMMANDS,
            live_control=False,
            enable_request=False,
            timeout_ms=500,
            calibration_readonly=True,
        )
        (flags,) = struct.unpack_from("<I", packet, 16)
        expected = (
            CONTROL_FLAG_DRY_RUN
            | CONTROL_FLAG_READONLY_POLL
            | CONTROL_FLAG_CALIBRATION_READONLY
        )
        self.assertEqual(flags, expected)

    def test_rejects_calibration_with_live_enable(self) -> None:
        with self.assertRaisesRegex(ValueError, "dry-run"):
            build_setpoint_packet(
                1,
                ZERO_COMMANDS,
                live_control=True,
                enable_request=True,
                timeout_ms=500,
                calibration_readonly=True,
            )


class MappingVerificationTests(unittest.TestCase):
    def test_passes_only_with_ids_1_through_4_on_every_can(self) -> None:
        positions = {
            (can_index, motor_id): 0.0
            for can_index in range(4)
            for motor_id in range(1, 5)
        }
        passed, ids_by_can = verify_mapping.evaluate_positions(positions)
        self.assertTrue(passed)
        self.assertEqual(ids_by_can[3], [1, 2, 3, 4])

        del positions[(3, 3)]
        passed, ids_by_can = verify_mapping.evaluate_positions(positions)
        self.assertFalse(passed)
        self.assertEqual(ids_by_can[3], [1, 2, 4])


if __name__ == "__main__":
    unittest.main()
