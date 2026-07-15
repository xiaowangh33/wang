#!/usr/bin/env python3
from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class DeploymentTimingConfigTests(unittest.TestCase):
    def test_relaxed_transport_recovery_contract(self) -> None:
        run_script = (ROOT / "run.sh").read_text()
        self.assertRegex(
            run_script,
            re.compile(
                r'export WHEELDOG_MCU_COMMAND_TIMEOUT_MS="\$\{WHEELDOG_MCU_COMMAND_TIMEOUT_MS:-300\}"'
            ),
        )
        self.assertRegex(
            run_script,
            re.compile(
                r'export WHEELDOG_RUNTIME_FEEDBACK_TIMEOUT_MS="\$\{WHEELDOG_RUNTIME_FEEDBACK_TIMEOUT_MS:-500\}"'
            ),
        )

        bridge = (
            ROOT
            / "third_party"
            / "rs06_mit_control"
            / "wheeldog_mcu_bridge.py"
        ).read_text()
        self.assertRegex(
            bridge, r"SERIAL_SETPOINT_DELIVERY_GRACE_MS\s*=\s*250\s*"
        )
        self.assertRegex(
            bridge, r"COHERENT_OBSERVATION_LOSS_TIMEOUT_S\s*=\s*0\.500\s*"
        )
        self.assertRegex(bridge, r"OBSERVATION_HISTORY_DEPTH\s*=\s*64\s*")
        self.assertRegex(
            bridge, r"OBSERVATION_HISTORY_MAX_AGE_S\s*=\s*0\.100\s*"
        )
        self.assertRegex(
            bridge, r"DEFAULT_MAX_OBSERVATION_SKEW_MS\s*=\s*30\.0\s*"
        )
        self.assertRegex(
            bridge, r"DEFAULT_MIN_MOTOR_RATE_HZ\s*=\s*350\.0\s*"
        )
        self.assertRegex(
            bridge, r"COMMAND_SNAPSHOT_HOLD_TIMEOUT_S\s*=\s*0\.050\s*"
        )
        self.assertRegex(
            bridge,
            r"FIRMWARE_COMM_ERROR_CONSECUTIVE_FEEDBACKS\s*=\s*5\s*",
        )

        firmware_root = ROOT.parent / "robot-v4-firmware-sanpo_spine" / "firmware" / "sanpo_spine"
        fast_feedback = (firmware_root / "Core" / "Inc" / "wd_fast_feedback.h").read_text()
        self.assertRegex(
            fast_feedback, r"WD_FAST_FEEDBACK_STOP_MS\s*\(300u\)"
        )
        control = (firmware_root / "Core" / "Src" / "wd_control.c").read_text()
        self.assertRegex(control, r"WD_CAN_RX_QUEUE_SIZE\s*\(64u\)")
        can = (firmware_root / "Core" / "Src" / "can.c").read_text()
        self.assertEqual(can.count("Init.AutoBusOff = ENABLE;"), 2)

        hardware = (
            ROOT
            / "interface"
            / "robot"
            / "hardware"
            / "wheeled_dog_hardware_interface.cpp"
        ).read_text()
        self.assertRegex(hardware, r"kMaxObservationSkewUs\s*=\s*30000u\s*;")

    def test_policy_transport_and_torque_contract(self) -> None:
        config = (ROOT / "description" / "robot_model_config.hpp").read_text()
        self.assertRegex(config, r"kPolicyInferenceHz\s*=\s*50\s*;")
        self.assertRegex(config, r"kHardwareHighLevelTickHz\s*=\s*200\s*;")
        self.assertRegex(config, r"kHardwarePcToMcuSetpointHz\s*=\s*200\s*;")
        self.assertRegex(config, r"kHardwareBringupTorqueLimitNm\s*=\s*36\.0f\s*;")
        self.assertRegex(config, r"kTrainingLegKd\s*=\s*3\.0f\s*;")
        self.assertRegex(config, r"kStandLegKdScale\s*=\s*1\.5f\s*;")
        self.assertRegex(config, r"kHardwareRlHipKd\s*=\s*kTrainingLegKd\s*;")
        self.assertRegex(config, r"kHardwareRlKneeKd\s*=\s*kTrainingLegKd\s*;")
        joints_block = config.split("kJoints = {{", 1)[1].split("}};", 1)[0]
        self.assertEqual(joints_block.count("kTrainingLegKd"), 12)

        run_script = (ROOT / "run.sh").read_text()
        self.assertRegex(
            run_script,
            re.compile(
                r'export WHEELDOG_HW_TICK_HZ="\$\{WHEELDOG_HW_TICK_HZ:-200\}"'
            ),
        )
        self.assertRegex(
            run_script,
            re.compile(
                r'export WHEELDOG_MCU_SETPOINT_HZ="\$\{WHEELDOG_MCU_SETPOINT_HZ:-200\}"'
            ),
        )
        self.assertRegex(
            run_script,
            re.compile(r'export RL_HIP_KD="\$\{RL_HIP_KD:-2\.0\}"'),
        )
        self.assertRegex(
            run_script,
            re.compile(r'export RL_KNEE_KD="\$\{RL_KNEE_KD:-2\.0\}"'),
        )
        self.assertRegex(
            run_script,
            re.compile(
                r'export RL_STABILITY_MONITOR="\$\{RL_STABILITY_MONITOR:-1\}"'
            ),
        )

        hardware_cpp = (
            ROOT
            / "interface"
            / "robot"
            / "hardware"
            / "wheeled_dog_hardware_interface.cpp"
        ).read_text()
        self.assertIn("robot_model::kHardwarePcToMcuSetpointHz", hardware_cpp)

        dryrun = (ROOT / "tools" / "wd_mcu_dryrun.py").read_text()
        self.assertRegex(dryrun, r"BENCH_TORQUE_LIMIT_NM\s*=\s*36\.0")
        self.assertRegex(dryrun, r"DEFAULT_SETPOINT_HZ\s*=\s*200\.0")


if __name__ == "__main__":
    unittest.main()
