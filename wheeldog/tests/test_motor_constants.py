#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "third_party" / "rs06_mit_control"))

from motor_constants import (  # noqa: E402
    RS01_IQ_MAX,
    RS01_KT,
    RS06_IQ_MAX,
    RS06_KT,
    iq_to_torque,
    torque_to_iq,
)


class MotorCurrentUnitTests(unittest.TestCase):
    def test_official_constants_use_rms_kt_and_peak_current_limits(self) -> None:
        self.assertEqual(RS06_KT, 1.09)
        self.assertEqual(RS06_IQ_MAX, 57.0)
        self.assertEqual(RS01_KT, 1.22)
        self.assertEqual(RS01_IQ_MAX, 23.0)

    def test_rs06_rated_point_converts_peak_current_to_torque(self) -> None:
        # Official rated point: 14.3 Apk and 1.09 N.m/Arms produce about 11 N.m.
        rated_torque_nm = iq_to_torque(14.3, "rs06")
        self.assertAlmostEqual(
            rated_torque_nm,
            14.3 / math.sqrt(2.0) * 1.09,
            places=6,
        )
        self.assertAlmostEqual(rated_torque_nm, 11.0, places=1)
        self.assertAlmostEqual(
            torque_to_iq(11.0, "rs06"),
            11.0 * math.sqrt(2.0) / 1.09,
            places=6,
        )

    def test_rs01_conversion_uses_the_same_peak_to_rms_semantics(self) -> None:
        rated_torque_nm = iq_to_torque(7.0, "rs01")
        self.assertAlmostEqual(
            rated_torque_nm,
            7.0 / math.sqrt(2.0) * 1.22,
            places=6,
        )
        self.assertAlmostEqual(rated_torque_nm, 6.0, places=1)
        self.assertAlmostEqual(
            torque_to_iq(6.0, "rs01"),
            6.0 * math.sqrt(2.0) / 1.22,
            places=6,
        )

    def test_round_trip_and_torque_clamp(self) -> None:
        for model, torque_nm in (("rs06", 10.0), ("rs01", -6.0)):
            self.assertAlmostEqual(
                iq_to_torque(torque_to_iq(torque_nm, model), model),
                torque_nm,
                places=6,
            )

        self.assertAlmostEqual(
            torque_to_iq(100.0, "rs06"),
            36.0 * math.sqrt(2.0) / RS06_KT,
            places=6,
        )
        self.assertAlmostEqual(
            torque_to_iq(-100.0, "rs01"),
            -17.0 * math.sqrt(2.0) / RS01_KT,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
