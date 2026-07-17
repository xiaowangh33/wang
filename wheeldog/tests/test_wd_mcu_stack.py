#!/usr/bin/env python3
from __future__ import annotations

import ctypes
import struct
import sys
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "third_party" / "rs06_mit_control"))

import wd_mcu_protocol as protocol  # noqa: E402
import wheeldog_mcu_bridge as bridge  # noqa: E402


def feedback_payload(
    base2: bool = False,
    *,
    last_setpoint_seq: int = 5,
    setpoint_age_ms: int = 1,
    command_timeout_ms: int = 100,
) -> bytes:
    status = protocol.STATUS_MCU_CAN_BASE_2 if base2 else 0
    local_mask = 0xFF00 if base2 else 0x00FF
    payload = bytearray(
        protocol.FEEDBACK_PREFIX.pack(
            status,
            0,
            123,
            4,
            last_setpoint_seq,
            6,
            0,
            0,
            1000,
            setpoint_age_ms,
            command_timeout_ms,
            local_mask,
            local_mask,
            0,
            1000.0,
            protocol.LIMIT_CALIBRATED_ABSOLUTE,
        )
    )
    for index in range(protocol.MOTOR_COUNT):
        payload += protocol.JOINT_FEEDBACK.pack(
            float(index),
            float(index) + 0.1,
            float(index) + 0.2,
            30.0,
            0,
            1 if local_mask & (1 << index) else 0,
            1 if local_mask & (1 << index) else 0,
            1 if index % 4 == 3 else 0,
            0,
        )
    payload += protocol.CAN_DIAGNOSTICS.pack(10, 0, 0, 0)
    payload += protocol.LAST_CAN.pack(1, 2, 3, 4)
    payload += protocol.MOTOR_COUNTERS.pack(*range(16))
    payload += protocol.MOTOR_COUNTERS.pack(*(500 + i for i in range(16)))
    payload += protocol.MOTOR_COUNTERS.pack(*(2 for _ in range(16)))
    payload += protocol.MOTOR_COUNTERS.pack(*(1000 + i for i in range(16)))
    payload += protocol.FAST_VALID_MASK.pack(local_mask)
    payload += protocol.FAST_VALID_MASK.pack(local_mask)
    payload += protocol.FAST_VALID_MASK.pack(local_mask)
    payload += protocol.MOTOR_FLOATS.pack(*(500.0 for _ in range(16)))
    payload += protocol.MOTOR_FLOATS.pack(*(0.01 for _ in range(16)))
    payload += protocol.MOTOR_FLOATS.pack(*(0.02 for _ in range(16)))
    payload += protocol.OBSERVATION_DIAGNOSTICS.pack(
        42, 123, 2, 3, *([2] * protocol.MOTOR_COUNT)
    )
    payload += protocol.LIVE_STOP_DIAGNOSTICS.pack(
        protocol.LIVE_STOP_REASON_FAST_FEEDBACK_LOST,
        1 << 5,
        1,
        456,
        local_mask & ~(1 << 5),
        0,
    )
    payload += protocol.MOTOR_BYTES.pack(*range(protocol.MOTOR_COUNT))
    payload += protocol.MOTOR_BYTES.pack(*(3 for _ in range(protocol.MOTOR_COUNT)))
    payload += protocol.MOTOR_FLOATS.pack(
        *(float(index) + 0.25 for index in range(protocol.MOTOR_COUNT))
    )
    payload += protocol.FAST_VALID_MASK.pack(local_mask)
    payload += protocol.MOTOR_FLOATS.pack(
        *(48.0 + index * 0.1 for index in range(protocol.MOTOR_COUNT))
    )
    self_contained = bytes(payload)
    assert len(self_contained) == protocol.MAX_PAYLOAD == 1172
    return self_contained


class ProtocolTests(unittest.TestCase):
    def test_setpoint_layout_and_modes(self) -> None:
        packet = protocol.build_setpoint_packet(
            7,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=100,
        )
        self.assertEqual(len(packet), 368)
        payload = packet[protocol.HEADER.size :]
        self.assertEqual(len(payload), 352)
        self.assertEqual(payload[12], protocol.LIMIT_CALIBRATED_ABSOLUTE)
        modes = payload[13:29]
        self.assertEqual([modes[i] for i in (3, 7, 11, 15)], [1, 1, 1, 1])

    def test_qualification_excitation_flag_is_explicit(self) -> None:
        commands = list(protocol.ZERO_COMMANDS)
        commands[0] = protocol.JointCommand(tau_ff=0.6)
        packet = protocol.build_setpoint_packet(
            8,
            commands,
            live_control=True,
            enable_request=True,
            timeout_ms=100,
            qualification_excitation=True,
        )
        (flags,) = struct.unpack_from("<I", packet, protocol.HEADER.size)
        self.assertTrue(flags & protocol.CONTROL_FLAG_QUALIFICATION_EXCITATION)

    def test_fragmented_packet_and_crc_rejection(self) -> None:
        payload = feedback_payload()
        packet = protocol.build_packet(protocol.PACKET_FEEDBACK, 9, payload)
        parser = protocol.PacketParser()
        result = []
        for byte in packet:
            result.extend(parser.feed(bytes([byte])))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][1], 9)

        corrupt = bytearray(packet)
        corrupt[-1] ^= 0x80
        self.assertEqual(protocol.PacketParser().feed(corrupt), [])

    def test_extended_feedback_diagnostics(self) -> None:
        parsed = protocol.parse_feedback(feedback_payload(base2=True))
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.bus_base, 2)
        self.assertEqual(parsed.local_mask, 0xFF00)
        self.assertEqual(parsed.live_command_tx_completed[0], 500)
        self.assertEqual(parsed.live_command_tx_deferred[15], 2)
        self.assertEqual(parsed.fast_feedback_valid_mask, 0xFF00)
        self.assertEqual(parsed.fast_position_excited_mask, 0xFF00)
        self.assertEqual(parsed.fast_velocity_excited_mask, 0xFF00)
        self.assertEqual(parsed.operation_status_rx_count[3], 1003)
        self.assertAlmostEqual(parsed.fast_feedback_rate_hz[7], 500.0)
        self.assertEqual(parsed.observation_seq, 42)
        self.assertEqual(parsed.observation_max_sample_age_ms, 2)
        self.assertEqual(
            parsed.live_stop_reason_flags,
            protocol.LIVE_STOP_REASON_FAST_FEEDBACK_LOST,
        )
        self.assertEqual(parsed.live_stop_motor_mask, 1 << 5)
        self.assertEqual(parsed.live_stop_trigger_time_ms, 456)
        self.assertEqual(parsed.operation_status_max_gap_ms[15], 15)
        self.assertEqual(parsed.live_stop_fast_age_ms[5], 3)
        self.assertTrue(parsed.final_torque_telemetry_present)
        self.assertAlmostEqual(parsed.final_joint_torque_cmd_nm[7], 7.25)
        self.assertTrue(parsed.supply_voltage_telemetry_present)
        self.assertEqual(parsed.supply_voltage_valid_mask, 0xFF00)
        self.assertAlmostEqual(parsed.supply_voltage_v[12], 49.2, places=4)

    def test_feedback_parser_accepts_previous_1040_byte_payload(self) -> None:
        parsed = protocol.parse_feedback(
            feedback_payload(base2=False)[
                : -(protocol.FAST_VALID_MASK.size + 2 * protocol.MOTOR_FLOATS.size)
            ]
        )
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertFalse(parsed.final_torque_telemetry_present)
        self.assertEqual(parsed.final_joint_torque_cmd_nm, (0.0,) * protocol.MOTOR_COUNT)
        self.assertFalse(parsed.supply_voltage_telemetry_present)

    def test_feedback_parser_accepts_previous_1104_byte_payload(self) -> None:
        parsed = protocol.parse_feedback(
            feedback_payload(base2=False)[
                : -(protocol.FAST_VALID_MASK.size + protocol.MOTOR_FLOATS.size)
            ]
        )
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertTrue(parsed.final_torque_telemetry_present)
        self.assertFalse(parsed.supply_voltage_telemetry_present)

    def test_feedback_parser_accepts_pre_extension_payload(self) -> None:
        parsed = protocol.parse_feedback(
            feedback_payload(base2=False)[
                : -(protocol.LIVE_STOP_DIAGNOSTICS.size + 2 * protocol.MOTOR_BYTES.size)
                - 2 * protocol.MOTOR_FLOATS.size
                - protocol.FAST_VALID_MASK.size
            ]
        )
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.live_stop_reason_flags, 0)
        self.assertEqual(parsed.live_stop_motor_mask, 0)


class BridgeTests(unittest.TestCase):
    def test_deployment_setpoint_default_is_200_hz(self) -> None:
        self.assertEqual(bridge.DEFAULT_SETPOINT_HZ, 200.0)
        self.assertEqual(bridge.MAX_SETPOINT_HZ, 200.0)

    def test_shared_layout(self) -> None:
        self.assertEqual(ctypes.sizeof(bridge.BridgeShm), 1200)

    def test_motor_telemetry_report_combines_both_mcus(self) -> None:
        first = protocol.parse_feedback(feedback_payload(base2=False))
        second = protocol.parse_feedback(feedback_payload(base2=True))
        assert first is not None and second is not None
        report = bridge.motor_telemetry_report({0: first, 2: second})
        self.assertIn("temp_C (HipX,HipY,Knee,Wheel) FL[30.0,30.0,30.0,30.0]", report)
        self.assertIn("bus_V  (HipX,HipY,Knee,Wheel) FL[48.0,48.1,48.2,48.3]", report)
        self.assertIn("HR[49.2,49.3,49.4,49.5]", report)

    def test_consistent_command_snapshot(self) -> None:
        shm = bridge.BridgeShm()
        shm.command_seq = 2
        for index in range(protocol.MOTOR_COUNT):
            shm.command_kp[index] = 65.0
            shm.command_q_des[index] = index * 0.1
            shm.command_kd[index] = 1.0
            shm.command_dq_des[index] = 0.2
            shm.command_tau_ff[index] = 0.3
        snapshot = bridge.read_command_snapshot(shm)
        commands = snapshot.commands
        self.assertEqual(len(commands), 16)
        self.assertEqual(snapshot.sequence, 2)
        self.assertFalse(snapshot.held)
        self.assertAlmostEqual(commands[10].q_des, 1.0)
        self.assertAlmostEqual(commands[10].kp, 65.0)
        self.assertAlmostEqual(commands[3].kd, 1.0)

    def test_command_snapshot_waits_for_active_cpp_writer(self) -> None:
        shm = bridge.BridgeShm()
        shm.command_seq = 3

        def finish_write() -> None:
            time.sleep(0.003)
            shm.command_kp[0] = 71.0
            shm.command_seq = 4

        writer = threading.Thread(target=finish_write)
        writer.start()
        snapshot = bridge.read_command_snapshot(shm, retry_timeout_s=0.050)
        writer.join(timeout=1.0)
        self.assertEqual(snapshot.sequence, 4)
        self.assertGreater(snapshot.attempts, 1)
        self.assertAlmostEqual(snapshot.commands[0].kp, 71.0)

    def test_command_snapshot_reader_holds_one_recent_frame(self) -> None:
        shm = bridge.BridgeShm()
        shm.command_seq = 2
        shm.command_kp[0] = 65.0
        reader = bridge.CommandSnapshotReader(
            retry_timeout_s=0.0002,
            hold_timeout_s=0.020,
        )
        first = reader.read(shm)
        shm.command_seq = 3
        held = reader.read(shm)
        self.assertFalse(first.held)
        self.assertTrue(held.held)
        self.assertEqual(held.sequence, first.sequence)
        self.assertAlmostEqual(held.commands[0].kp, 65.0)
        self.assertEqual(reader.hold_cycles, 1)
        report = reader.consume_timing_report()
        self.assertIn("holds=1", report)
        self.assertIn("max_consecutive_holds=1", report)
        self.assertEqual(reader.hold_cycles, 0)

    def test_command_snapshot_reader_rejects_persistently_stuck_writer(self) -> None:
        shm = bridge.BridgeShm()
        shm.command_seq = 2
        reader = bridge.CommandSnapshotReader(
            retry_timeout_s=0.0002,
            hold_timeout_s=0.002,
        )
        reader.read(shm)
        shm.command_seq = 3
        time.sleep(0.003)
        with self.assertRaisesRegex(RuntimeError, "exceeding the 2.0 ms safety limit"):
            reader.read(shm)

    def test_final_torque_diagnostic_uses_mcu_post_safety_command(self) -> None:
        mcu_a = protocol.parse_feedback(feedback_payload(base2=False))
        mcu_b = protocol.parse_feedback(feedback_payload(base2=True))
        assert mcu_a is not None and mcu_b is not None
        mcu_a.status_flags |= protocol.STATUS_TORQUE_SLEW_LIMITED
        diagnostic = bridge.final_torque_diagnostic({0: mcu_a, 2: mcu_b})
        self.assertIsNotNone(diagnostic)
        assert diagnostic is not None
        self.assertIn("wheel_final=(3.2 7.2 11.2 15.2)", diagnostic)
        self.assertIn("slew_this_feedback=A", diagnostic)

    def test_transient_serial_timeout_is_retried(self) -> None:
        class TimeoutOnceDevice:
            def __init__(self) -> None:
                self.calls = 0

            def write(self, packet: bytes) -> int:
                self.calls += 1
                if self.calls == 1:
                    raise bridge.SerialTimeoutException("Write timeout")
                return len(packet)

        device = TimeoutOnceDevice()
        state = bridge.PortState("mcu-a", device, protocol.PacketParser())
        bridge.send_packet([state], b"wdp4")
        self.assertEqual(device.calls, 2)
        self.assertEqual(state.write_timeout_events, 1)
        self.assertEqual(state.recovered_write_events, 1)

    def test_one_persistent_write_failure_does_not_starve_other_mcu(self) -> None:
        class AlwaysTimeoutDevice:
            def __init__(self) -> None:
                self.calls = 0

            def write(self, _packet: bytes) -> int:
                self.calls += 1
                raise bridge.SerialTimeoutException("Write timeout")

        class HealthyDevice:
            def __init__(self) -> None:
                self.calls = 0

            def write(self, packet: bytes) -> int:
                self.calls += 1
                return len(packet)

        failed = AlwaysTimeoutDevice()
        healthy = HealthyDevice()
        states = [
            bridge.PortState("mcu-a", failed, protocol.PacketParser()),
            bridge.PortState("mcu-b", healthy, protocol.PacketParser()),
        ]
        with self.assertRaisesRegex(
            RuntimeError, "persistent USB write failure; mcu-a: attempts=3"
        ):
            bridge.send_packet(states, b"wdp4")
        self.assertEqual(failed.calls, 3)
        self.assertEqual(healthy.calls, 1)

    def test_two_persistent_failures_are_retried_round_robin(self) -> None:
        write_order: list[str] = []

        class AlwaysTimeoutDevice:
            def __init__(self, name: str) -> None:
                self.name = name

            def write(self, _packet: bytes) -> int:
                write_order.append(self.name)
                raise bridge.SerialTimeoutException("Write timeout")

        states = [
            bridge.PortState(
                "mcu-a", AlwaysTimeoutDevice("a"), protocol.PacketParser()
            ),
            bridge.PortState(
                "mcu-b", AlwaysTimeoutDevice("b"), protocol.PacketParser()
            ),
        ]
        with self.assertRaisesRegex(RuntimeError, "persistent USB write failure"):
            bridge.send_packet(states, b"wdp4", max_attempts=3)
        self.assertEqual(write_order, ["a", "b", "a", "b", "a", "b"])

    def test_write_timeout_is_recovered_when_mcu_acknowledges_setpoint(self) -> None:
        class TimeoutWithAcknowledgementDevice:
            def __init__(self, feedback_packet: bytes) -> None:
                self.calls = 0
                self.pending = bytearray(feedback_packet)

            def write(self, _packet: bytes) -> int:
                self.calls += 1
                raise bridge.SerialTimeoutException("Write timeout")

            @property
            def in_waiting(self) -> int:
                if self.calls == 0:
                    return 0
                return len(self.pending)

            def read(self, size: int) -> bytes:
                data = bytes(self.pending[:size])
                del self.pending[:size]
                return data

        feedback_packet = protocol.build_packet(
            protocol.PACKET_FEEDBACK, 9, feedback_payload()
        )
        device = TimeoutWithAcknowledgementDevice(feedback_packet)
        state = bridge.PortState("mcu-a", device, protocol.PacketParser())
        packet = protocol.build_setpoint_packet(
            5,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=100,
        )

        bridge.send_packet([state], packet)

        self.assertEqual(device.calls, 1)
        self.assertEqual(state.write_timeout_events, 1)
        self.assertEqual(state.acknowledged_write_events, 1)
        self.assertIsNotNone(state.feedback)
        assert state.feedback is not None
        self.assertEqual(state.feedback.last_setpoint_seq, 5)

    def test_delayed_setpoint_ack_stops_duplicate_retries(self) -> None:
        class TimeoutWithDelayedAcknowledgementDevice:
            def __init__(self, feedback_packet: bytes) -> None:
                self.calls = 0
                self.pending = bytearray(feedback_packet)
                self.release_time = float("inf")

            def write(self, _packet: bytes) -> int:
                self.calls += 1
                if self.calls == 1:
                    self.release_time = time.monotonic() + 0.004
                raise bridge.SerialTimeoutException("Write timeout")

            @property
            def in_waiting(self) -> int:
                if time.monotonic() < self.release_time:
                    return 0
                return len(self.pending)

            def read(self, size: int) -> bytes:
                data = bytes(self.pending[:size])
                del self.pending[:size]
                return data

        feedback_packet = protocol.build_packet(
            protocol.PACKET_FEEDBACK,
            10,
            feedback_payload(last_setpoint_seq=77),
        )
        device = TimeoutWithDelayedAcknowledgementDevice(feedback_packet)
        state = bridge.PortState("mcu-a", device, protocol.PacketParser())
        packet = protocol.build_setpoint_packet(
            77,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=100,
        )

        bridge.send_packet([state], packet)

        self.assertEqual(device.calls, 1)
        self.assertEqual(state.write_timeout_events, 1)
        self.assertEqual(state.acknowledged_write_events, 1)

    def test_both_delayed_setpoint_acks_recover_in_same_round(self) -> None:
        class TimeoutWithDelayedAcknowledgementDevice:
            def __init__(self, feedback_packet: bytes, delay_s: float) -> None:
                self.calls = 0
                self.pending = bytearray(feedback_packet)
                self.delay_s = delay_s
                self.release_time = float("inf")

            def write(self, _packet: bytes) -> int:
                self.calls += 1
                if self.calls == 1:
                    self.release_time = time.monotonic() + self.delay_s
                raise bridge.SerialTimeoutException("Write timeout")

            @property
            def in_waiting(self) -> int:
                if time.monotonic() < self.release_time:
                    return 0
                return len(self.pending)

            def read(self, size: int) -> bytes:
                data = bytes(self.pending[:size])
                del self.pending[:size]
                return data

        packet = protocol.build_setpoint_packet(
            88,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=100,
        )
        states: list[bridge.PortState] = []
        devices: list[TimeoutWithDelayedAcknowledgementDevice] = []
        for name, base2, delay_s in (
            ("mcu-a", False, 0.003),
            ("mcu-b", True, 0.006),
        ):
            feedback_packet = protocol.build_packet(
                protocol.PACKET_FEEDBACK,
                11,
                feedback_payload(base2=base2, last_setpoint_seq=88),
            )
            device = TimeoutWithDelayedAcknowledgementDevice(
                feedback_packet, delay_s
            )
            devices.append(device)
            states.append(bridge.PortState(name, device, protocol.PacketParser()))

        bridge.send_packet(states, packet)

        self.assertEqual([device.calls for device in devices], [1, 1])
        self.assertEqual(
            [state.acknowledged_write_events for state in states], [1, 1]
        )

    def test_dual_blocking_timeouts_recover_before_mcu_watchdog(self) -> None:
        class BlockingTimeoutWithAcknowledgementDevice:
            def __init__(self, feedback_packet: bytes, ack_delay_s: float) -> None:
                self.calls = 0
                self.pending = bytearray(feedback_packet)
                self.ack_delay_s = ack_delay_s
                self.release_time = float("inf")

            def write(self, _packet: bytes) -> int:
                self.calls += 1
                if self.calls == 1:
                    self.release_time = time.monotonic() + self.ack_delay_s
                time.sleep(bridge.SERIAL_WRITE_TIMEOUT_S)
                raise bridge.SerialTimeoutException("Write timeout")

            @property
            def in_waiting(self) -> int:
                if time.monotonic() < self.release_time:
                    return 0
                return len(self.pending)

            def read(self, size: int) -> bytes:
                data = bytes(self.pending[:size])
                del self.pending[:size]
                return data

        packet = protocol.build_setpoint_packet(
            99,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=100,
        )
        devices: list[BlockingTimeoutWithAcknowledgementDevice] = []
        states: list[bridge.PortState] = []
        for name, base2, ack_delay_s in (
            ("mcu-a", False, 0.035),
            ("mcu-b", True, 0.035),
        ):
            feedback_packet = protocol.build_packet(
                protocol.PACKET_FEEDBACK,
                12,
                feedback_payload(base2=base2, last_setpoint_seq=99),
            )
            device = BlockingTimeoutWithAcknowledgementDevice(
                feedback_packet, ack_delay_s
            )
            devices.append(device)
            states.append(bridge.PortState(name, device, protocol.PacketParser()))

        started = time.monotonic()
        bridge.send_packet(states, packet)
        elapsed = time.monotonic() - started

        self.assertEqual([device.calls for device in devices], [2, 2])
        self.assertEqual(
            [state.acknowledged_write_events for state in states], [1, 1]
        )
        self.assertLess(elapsed, bridge.SERIAL_PACKET_RECOVERY_DEADLINE_S + 0.010)

    def test_dual_blocking_failures_stop_before_mcu_watchdog(self) -> None:
        class BlockingTimeoutDevice:
            def __init__(self) -> None:
                self.calls = 0

            def write(self, _packet: bytes) -> int:
                self.calls += 1
                time.sleep(bridge.SERIAL_WRITE_TIMEOUT_S)
                raise bridge.SerialTimeoutException("Write timeout")

        packet = protocol.build_setpoint_packet(
            100,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=100,
        )
        devices = [BlockingTimeoutDevice(), BlockingTimeoutDevice()]
        states = [
            bridge.PortState("mcu-a", devices[0], protocol.PacketParser()),
            bridge.PortState("mcu-b", devices[1], protocol.PacketParser()),
        ]

        started = time.monotonic()
        with self.assertRaisesRegex(
            RuntimeError, "last=recovery deadline exceeded"
        ):
            bridge.send_packet(states, packet)
        elapsed = time.monotonic() - started

        self.assertTrue(all(1 <= device.calls <= 3 for device in devices))
        self.assertLess(
            elapsed,
            bridge.SERIAL_PACKET_RECOVERY_DEADLINE_S + 0.010,
        )

    def test_unconfirmed_setpoint_is_tolerated_while_previous_is_fresh(self) -> None:
        class AlwaysTimeoutDevice:
            def write(self, _packet: bytes) -> int:
                raise bridge.SerialTimeoutException("Write timeout")

            @property
            def in_waiting(self) -> int:
                return 0

        parsed = protocol.parse_feedback(
            feedback_payload(
                last_setpoint_seq=98,
                setpoint_age_ms=20,
                command_timeout_ms=bridge.DEFAULT_COMMAND_TIMEOUT_MS,
            )
        )
        assert parsed is not None
        state = bridge.PortState(
            "mcu-a",
            AlwaysTimeoutDevice(),
            protocol.PacketParser(),
            parsed,
            time.monotonic(),
        )
        packet = protocol.build_setpoint_packet(
            99,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=bridge.DEFAULT_COMMAND_TIMEOUT_MS,
        )

        bridge.send_packet([state], packet)

        self.assertEqual(state.unconfirmed_setpoint_events, 1)
        self.assertEqual(state.consecutive_unconfirmed_setpoints, 1)

    def test_unconfirmed_setpoint_fails_before_mcu_watchdog(self) -> None:
        class AlwaysTimeoutDevice:
            def write(self, _packet: bytes) -> int:
                raise bridge.SerialTimeoutException("Write timeout")

            @property
            def in_waiting(self) -> int:
                return 0

        parsed = protocol.parse_feedback(
            feedback_payload(
                last_setpoint_seq=98,
                setpoint_age_ms=280,
                command_timeout_ms=bridge.DEFAULT_COMMAND_TIMEOUT_MS,
            )
        )
        assert parsed is not None
        state = bridge.PortState(
            "mcu-a",
            AlwaysTimeoutDevice(),
            protocol.PacketParser(),
            parsed,
            time.monotonic(),
        )
        packet = protocol.build_setpoint_packet(
            99,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=bridge.DEFAULT_COMMAND_TIMEOUT_MS,
        )

        with self.assertRaisesRegex(
            RuntimeError, "persistent USB write failure"
        ):
            bridge.send_packet([state], packet)

    def test_one_usb_device_error_is_tolerated_while_previous_command_is_fresh(self) -> None:
        class DisconnectedDevice:
            def write(self, _packet: bytes) -> int:
                raise OSError("device disconnected")

            @property
            def in_waiting(self) -> int:
                return 0

        parsed = protocol.parse_feedback(
            feedback_payload(
                last_setpoint_seq=98,
                setpoint_age_ms=1,
                command_timeout_ms=bridge.DEFAULT_COMMAND_TIMEOUT_MS,
            )
        )
        assert parsed is not None
        state = bridge.PortState(
            "mcu-a",
            DisconnectedDevice(),
            protocol.PacketParser(),
            parsed,
            time.monotonic(),
        )
        packet = protocol.build_setpoint_packet(
            99,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=bridge.DEFAULT_COMMAND_TIMEOUT_MS,
        )

        bridge.send_packet([state], packet)

        self.assertEqual(state.unconfirmed_setpoint_events, 1)
        self.assertEqual(state.consecutive_unconfirmed_setpoints, 1)

    def test_usb_device_error_fails_when_previous_command_nears_watchdog(self) -> None:
        class DisconnectedDevice:
            def write(self, _packet: bytes) -> int:
                raise OSError("device disconnected")

            @property
            def in_waiting(self) -> int:
                return 0

        parsed = protocol.parse_feedback(
            feedback_payload(
                last_setpoint_seq=98,
                setpoint_age_ms=280,
                command_timeout_ms=bridge.DEFAULT_COMMAND_TIMEOUT_MS,
            )
        )
        assert parsed is not None
        state = bridge.PortState(
            "mcu-a",
            DisconnectedDevice(),
            protocol.PacketParser(),
            parsed,
            time.monotonic(),
        )
        packet = protocol.build_setpoint_packet(
            99,
            protocol.ZERO_COMMANDS,
            live_control=True,
            enable_request=True,
            timeout_ms=bridge.DEFAULT_COMMAND_TIMEOUT_MS,
        )

        with self.assertRaisesRegex(RuntimeError, "OSError: device disconnected"):
            bridge.send_packet([state], packet)

    def test_one_usb_read_error_does_not_starve_other_mcu(self) -> None:
        class ReadErrorDevice:
            @property
            def in_waiting(self) -> int:
                raise OSError("transient read error")

        class FeedbackDevice:
            def __init__(self, packet: bytes) -> None:
                self.pending = bytearray(packet)

            @property
            def in_waiting(self) -> int:
                return len(self.pending)

            def read(self, size: int) -> bytes:
                data = bytes(self.pending[:size])
                del self.pending[:size]
                return data

        feedback_packet = protocol.build_packet(
            protocol.PACKET_FEEDBACK, 9, feedback_payload(base2=True)
        )
        failed = bridge.PortState(
            "mcu-a", ReadErrorDevice(), protocol.PacketParser()
        )
        healthy = bridge.PortState(
            "mcu-b", FeedbackDevice(feedback_packet), protocol.PacketParser()
        )

        self.assertEqual(bridge.read_feedback([failed, healthy]), 1)
        self.assertEqual(failed.read_error_events, 1)
        self.assertEqual(failed.consecutive_read_errors, 1)
        self.assertIsNotNone(healthy.feedback)

    def test_usb_timing_report_resets_window_only(self) -> None:
        state = bridge.PortState("mcu-a", object(), protocol.PacketParser())
        state.write_window_attempts = 2
        state.write_window_total_ms = 0.6
        state.write_window_max_ms = 0.4
        state.write_timeout_events = 3
        state.recovered_write_events = 1
        state.acknowledged_write_events = 2
        state.unconfirmed_setpoint_events = 4
        state.max_consecutive_unconfirmed_setpoints = 3
        state.suppressed_recovery_logs = 7
        report = bridge.consume_usb_timing_report([state])
        self.assertIn("avg=0.300ms", report)
        self.assertIn("max=0.400ms", report)
        self.assertIn("timeouts_total=3", report)
        self.assertIn("mcu_ack_recoveries_total=2", report)
        self.assertIn("unconfirmed_setpoints_total=4", report)
        self.assertIn("max_consecutive_unconfirmed=3", report)
        self.assertIn("suppressed_recovery_logs=7", report)
        self.assertEqual(state.write_window_attempts, 0)
        self.assertEqual(state.write_window_total_ms, 0.0)
        self.assertEqual(state.write_timeout_events, 3)

    def test_duplicate_mcu_base_is_rejected(self) -> None:
        parsed = protocol.parse_feedback(feedback_payload(base2=False))
        assert parsed is not None
        states = [
            bridge.PortState("a", object(), protocol.PacketParser(), parsed),
            bridge.PortState("b", object(), protocol.PacketParser(), parsed),
        ]
        with self.assertRaisesRegex(RuntimeError, "same WD_CAN_BUS_BASE"):
            bridge.feedback_by_base(states)

    def test_bridge_requires_fast_feedback_mask_and_status(self) -> None:
        parsed = protocol.parse_feedback(feedback_payload(base2=False))
        assert parsed is not None
        parsed.status_flags |= (
            protocol.STATUS_LIVE_CONTROL_ACTIVE
            | protocol.STATUS_LIVE_ENABLE_READY
            | protocol.STATUS_FAST_FEEDBACK_READY
        )
        parsed.live_stop_reason_flags = (
            protocol.LIVE_STOP_REASON_FAST_FEEDBACK_LOST
        )
        parsed.live_stop_motor_mask = 1 << 5
        self.assertTrue(bridge.local_feedback_ready(parsed))
        parsed.fast_feedback_valid_mask = 0
        self.assertFalse(bridge.local_feedback_ready(parsed))

    def test_firmware_fault_reports_all_bits_and_motor_detail(self) -> None:
        parsed = protocol.parse_feedback(feedback_payload(base2=False))
        assert parsed is not None
        parsed.status_flags |= (
            protocol.STATUS_LIVE_CONTROL_BLOCKED
            | protocol.STATUS_LIVE_SAFETY_STOP
        )
        parsed.fast_feedback_valid_mask &= ~(1 << 5)
        parsed.fast_velocity_error_radps = tuple(
            0.75 if i == 5 else value
            for i, value in enumerate(parsed.fast_velocity_error_radps)
        )
        error = bridge.critical_firmware_error(parsed)
        self.assertEqual(error, "live safety stop")
        diagnostic = bridge.firmware_fault_diagnostic(parsed)
        self.assertIn("flags=0x", diagnostic)
        self.assertIn("unqualified=[5]", diagnostic)
        self.assertIn("5:rx=1005", diagnostic)
        self.assertIn("ve=0.7500", diagnostic)
        self.assertIn("reason=fast-feedback-lost", diagnostic)
        self.assertIn("motors=0x0020", diagnostic)
        self.assertIn("velocity_snapshot_after_stop=5:5.100rad/s", diagnostic)
        self.assertIn("mcu_final_torque_at_stop=0:0.25Nm", diagnostic)
        self.assertIn("7:7.25Nm", diagnostic)

    def test_transient_firmware_communication_bits_are_not_immediate_fatal(self) -> None:
        parsed = protocol.parse_feedback(feedback_payload(base2=False))
        assert parsed is not None
        parsed.status_flags |= (
            protocol.STATUS_CAN_RX_OVERFLOW
            | protocol.STATUS_CAN_TX_ERROR
            | protocol.STATUS_CAN_TX_DEADLINE_MISS
            | protocol.STATUS_LIVE_CONTROL_BLOCKED
        )

        self.assertIsNone(bridge.critical_firmware_error(parsed))

        parsed.status_flags |= protocol.STATUS_LIVE_SAFETY_STOP
        self.assertEqual(
            bridge.critical_firmware_error(parsed), "live safety stop"
        )

    def test_firmware_can_errors_require_five_consecutive_feedback_packets(self) -> None:
        parsed = protocol.parse_feedback(feedback_payload(base2=False))
        assert parsed is not None
        state = bridge.PortState(
            "mcu-a", object(), protocol.PacketParser(), parsed
        )
        self.assertIsNone(
            bridge.persistent_firmware_communication_error(
                state, monitor_live_control=True
            )
        )

        for sample in range(1, bridge.FIRMWARE_COMM_ERROR_CONSECUTIVE_FEEDBACKS):
            parsed.feedback_time_ms += 5
            parsed.can_tx_errors += 1
            parsed.status_flags |= protocol.STATUS_CAN_TX_ERROR
            self.assertIsNone(
                bridge.persistent_firmware_communication_error(
                    state, monitor_live_control=True
                )
            )

        parsed.feedback_time_ms += 5
        parsed.can_tx_errors += 1
        error = bridge.persistent_firmware_communication_error(
            state, monitor_live_control=True
        )
        self.assertIsNotNone(error)
        assert error is not None
        self.assertIn("persistent CAN communication errors", error)

    def test_clean_feedback_resets_firmware_communication_debounce(self) -> None:
        parsed = protocol.parse_feedback(feedback_payload(base2=False))
        assert parsed is not None
        state = bridge.PortState(
            "mcu-a", object(), protocol.PacketParser(), parsed
        )
        bridge.persistent_firmware_communication_error(
            state, monitor_live_control=True
        )
        for _ in range(2):
            parsed.feedback_time_ms += 5
            parsed.can_rx_overflows += 1
            self.assertIsNone(
                bridge.persistent_firmware_communication_error(
                    state, monitor_live_control=True
                )
            )
        parsed.feedback_time_ms += 5
        self.assertIsNone(
            bridge.persistent_firmware_communication_error(
                state, monitor_live_control=True
            )
        )
        self.assertEqual(state.consecutive_can_error_feedbacks, 0)

    def test_overspeed_stop_reason_is_reported(self) -> None:
        parsed = protocol.parse_feedback(feedback_payload(base2=False))
        assert parsed is not None
        parsed.live_stop_reason_flags = protocol.LIVE_STOP_REASON_OVERSPEED
        parsed.live_stop_motor_mask = 1 << 6
        commands = list(protocol.ZERO_COMMANDS)
        commands[6] = protocol.JointCommand(
            kp=65.0, q_des=-1.0, kd=1.0, dq_des=0.0, tau_ff=0.0
        )
        diagnostic = bridge.firmware_fault_diagnostic(parsed, commands)
        self.assertIn("reason=measured-overspeed", diagnostic)
        self.assertIn("velocity_snapshot_after_stop=6:6.100rad/s", diagnostic)
        self.assertIn("joint_state_after_stop=6:q=6.000,dq=6.100", diagnostic)
        self.assertIn("pc_command_at_fault=6:kp=65.00,qdes=-1.000", diagnostic)

    def test_bridge_rejects_mixed_observation_epochs(self) -> None:
        a = protocol.parse_feedback(feedback_payload(base2=False))
        b = protocol.parse_feedback(feedback_payload(base2=True))
        assert a is not None and b is not None
        states = [
            bridge.PortState("a", object(), protocol.PacketParser(), a, 1.000),
            bridge.PortState("b", object(), protocol.PacketParser(), b, 1.002),
        ]
        coherent = bridge.coherent_observation_by_base(states)
        self.assertEqual(set(coherent), {0, 2})
        self.assertAlmostEqual(
            bridge.coherent_observation_skew_s(states, coherent), 0.002
        )
        # If only B advances, history intentionally keeps the last common
        # epoch available instead of permanently losing synchronization.
        b_next = protocol.parse_feedback(feedback_payload(base2=True))
        assert b_next is not None
        b_next.observation_seq = b.observation_seq + 1
        states[1].feedback = b_next
        states[1].last_feedback_time = 1.007
        bridge.remember_observation(states[1], b_next, 1.007)
        historical = bridge.coherent_observation_by_base(states, now=1.007)
        self.assertEqual(historical[0].observation_seq, a.observation_seq)
        self.assertEqual(historical[2].observation_seq, a.observation_seq)

    def test_bridge_recovers_when_lagging_mcu_catches_up(self) -> None:
        a = protocol.parse_feedback(feedback_payload(base2=False))
        b = protocol.parse_feedback(feedback_payload(base2=True))
        assert a is not None and b is not None
        a.observation_seq = 100
        b.observation_seq = 100
        states = [
            bridge.PortState("a", object(), protocol.PacketParser(), a, 1.000),
            bridge.PortState("b", object(), protocol.PacketParser(), b, 1.002),
        ]

        a_next = protocol.parse_feedback(feedback_payload(base2=False))
        assert a_next is not None
        a_next.observation_seq = 101
        states[0].feedback = a_next
        states[0].last_feedback_time = 1.005
        bridge.remember_observation(states[0], a_next, 1.005)
        self.assertEqual(
            bridge.coherent_observation_by_base(states, now=1.006)[0].observation_seq,
            100,
        )

        b_next = protocol.parse_feedback(feedback_payload(base2=True))
        assert b_next is not None
        b_next.observation_seq = 101
        states[1].feedback = b_next
        states[1].last_feedback_time = 1.014
        bridge.remember_observation(states[1], b_next, 1.014)
        recovered = bridge.coherent_observation_by_base(states, now=1.014)
        self.assertEqual(recovered[0].observation_seq, 101)
        self.assertEqual(recovered[2].observation_seq, 101)
        self.assertAlmostEqual(
            bridge.coherent_observation_skew_s(states, recovered), 0.009
        )

    def test_bridge_does_not_match_expired_observation_history(self) -> None:
        a = protocol.parse_feedback(feedback_payload(base2=False))
        b = protocol.parse_feedback(feedback_payload(base2=True))
        assert a is not None and b is not None
        states = [
            bridge.PortState("a", object(), protocol.PacketParser(), a, 1.000),
            bridge.PortState("b", object(), protocol.PacketParser(), b, 1.002),
        ]
        self.assertEqual(
            bridge.coherent_observation_by_base(
                states,
                now=1.002 + bridge.OBSERVATION_HISTORY_MAX_AGE_S + 0.001,
            ),
            {},
        )

    def test_bridge_does_not_publish_transiently_stale_observation(self) -> None:
        a = protocol.parse_feedback(feedback_payload(base2=False))
        b = protocol.parse_feedback(feedback_payload(base2=True))
        assert a is not None and b is not None
        by_base = {0: a, 2: b}
        self.assertTrue(bridge.coherent_observation_is_fresh(by_base, 5.0))
        b.fast_feedback_valid_mask &= ~(1 << 10)
        self.assertFalse(bridge.coherent_observation_is_fresh(by_base, 5.0))


if __name__ == "__main__":
    unittest.main()
