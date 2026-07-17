#!/usr/bin/env python3
from __future__ import annotations

import ctypes
import os
import pty
import select
import shutil
import struct
import subprocess
import sys
import threading
import time
import tty
import unittest
from multiprocessing import shared_memory
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = ROOT / "third_party" / "rs06_mit_control"
sys.path.insert(0, str(MODULE_ROOT))

import wd_mcu_protocol as protocol  # noqa: E402
import wheeldog_mcu_bridge as bridge  # noqa: E402


def python_with_pyserial() -> str | None:
    candidates = [
        "/home/gu/anaconda3/bin/python3",
        "/home/gu/miniconda3/bin/python3",
        shutil.which("python3"),
    ]
    for candidate in candidates:
        if not candidate or not os.path.isfile(candidate):
            continue
        result = subprocess.run(
            [candidate, "-c", "import serial"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode == 0:
            return candidate
    return None


class FakeMcu:
    def __init__(self, base2: bool):
        master, slave = pty.openpty()
        tty.setraw(slave)
        self.master = master
        self.port = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(master, False)
        self.base2 = base2
        self.parser = protocol.PacketParser()
        self.observation_seq = 0
        self.qualification_mask = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=2.0)
        os.close(self.master)

    def _payload(self, elapsed: float) -> bytes:
        status = protocol.STATUS_LIVE_CONTROL_ACTIVE | protocol.STATUS_LIVE_ENABLE_READY
        if self.base2:
            status |= protocol.STATUS_MCU_CAN_BASE_2
        local_mask = 0xFF00 if self.base2 else 0x00FF
        status |= protocol.STATUS_FAST_FEEDBACK_READY
        payload = bytearray(
            protocol.FEEDBACK_PREFIX.pack(
                status,
                0,
                int(elapsed * 1000),
                1,
                self.observation_seq,
                1,
                0,
                0,
                int(elapsed * 1000),
                0,
                100,
                local_mask,
                local_mask,
                0,
                1000.0,
                protocol.LIMIT_CALIBRATED_ABSOLUTE,
            )
        )
        for index in range(protocol.MOTOR_COUNT):
            local = bool(local_mask & (1 << index))
            payload += protocol.JOINT_FEEDBACK.pack(
                index * 0.01,
                0.0,
                0.0,
                30.0,
                0,
                int(local),
                int(local),
                int(index % 4 == 3),
                0,
            )
        payload += protocol.CAN_DIAGNOSTICS.pack(0, 0, 0, 0)
        payload += protocol.LAST_CAN.pack(0, 0, 0, 0)
        completed = [0] * protocol.MOTOR_COUNT
        first = 8 if self.base2 else 0
        count = int(elapsed * 500.0)
        for index in range(first, first + 8):
            completed[index] = count
        payload += protocol.MOTOR_COUNTERS.pack(*completed)
        payload += protocol.MOTOR_COUNTERS.pack(*completed)
        payload += protocol.MOTOR_COUNTERS.pack(*([0] * protocol.MOTOR_COUNT))
        payload += protocol.MOTOR_COUNTERS.pack(*completed)
        payload += protocol.FAST_VALID_MASK.pack(local_mask)
        payload += protocol.FAST_VALID_MASK.pack(self.qualification_mask & local_mask)
        payload += protocol.FAST_VALID_MASK.pack(self.qualification_mask & local_mask)
        payload += protocol.MOTOR_FLOATS.pack(*([500.0] * protocol.MOTOR_COUNT))
        payload += protocol.MOTOR_FLOATS.pack(*([0.01] * protocol.MOTOR_COUNT))
        payload += protocol.MOTOR_FLOATS.pack(*([0.01] * protocol.MOTOR_COUNT))
        payload += protocol.OBSERVATION_DIAGNOSTICS.pack(
            self.observation_seq,
            int(elapsed * 1000),
            2,
            3,
            *([2] * protocol.MOTOR_COUNT),
        )
        payload += protocol.LIVE_STOP_DIAGNOSTICS.pack(0, 0, 0, 0, 0, 0)
        payload += protocol.MOTOR_BYTES.pack(*([0] * protocol.MOTOR_COUNT))
        payload += protocol.MOTOR_BYTES.pack(*([0] * protocol.MOTOR_COUNT))
        payload += protocol.MOTOR_FLOATS.pack(*([0.0] * protocol.MOTOR_COUNT))
        payload += protocol.FAST_VALID_MASK.pack(local_mask)
        payload += protocol.MOTOR_FLOATS.pack(*([48.0] * protocol.MOTOR_COUNT))
        assert len(payload) == protocol.MAX_PAYLOAD
        return bytes(payload)

    def _run(self) -> None:
        started = time.monotonic()
        next_feedback = started
        seq = 1
        while not self.stop.is_set():
            now = time.monotonic()
            readable, _, _ = select.select([self.master], [], [], 0)
            if readable:
                try:
                    data = os.read(self.master, 8192)
                    for packet_type, packet_seq, payload in self.parser.feed(data):
                        if packet_type == protocol.PACKET_SETPOINT:
                            self.observation_seq = packet_seq
                            (flags,) = struct.unpack_from("<I", payload, 0)
                            if flags & protocol.CONTROL_FLAG_QUALIFICATION_EXCITATION:
                                first = 8 if self.base2 else 0
                                for index in range(first, first + 8):
                                    values = protocol.JOINT_COMMAND.unpack_from(
                                        payload,
                                        32 + index * protocol.JOINT_COMMAND.size,
                                    )
                                    if abs(values[3]) > 0.9 or abs(values[4]) > 0.5:
                                        self.qualification_mask |= 1 << index
                except BlockingIOError:
                    pass
                except OSError:
                    pass
            if now >= next_feedback:
                packet = protocol.build_packet(
                    protocol.PACKET_FEEDBACK,
                    seq,
                    self._payload(now - started),
                )
                try:
                    os.write(self.master, packet)
                    seq += 1
                except (BlockingIOError, OSError):
                    pass
                next_feedback += 0.02
            time.sleep(0.0005)


@unittest.skipUnless(python_with_pyserial(), "no Python interpreter with pyserial")
class BridgeIntegrationTests(unittest.TestCase):
    def test_dual_mcu_bridge_reports_500_hz(self) -> None:
        python = python_with_pyserial()
        assert python is not None
        mcu_a = FakeMcu(base2=False)
        mcu_b = FakeMcu(base2=True)
        mcu_a.start()
        mcu_b.start()

        shm_obj = shared_memory.SharedMemory(create=True, size=ctypes.sizeof(bridge.BridgeShm))
        shm = bridge.BridgeShm.from_buffer(shm_obj.buf)
        ctypes.memset(ctypes.addressof(shm), 0, ctypes.sizeof(shm))
        shm.magic = bridge.MAGIC
        shm.version = bridge.VERSION
        shm.motor_count = protocol.MOTOR_COUNT
        publisher_stop = threading.Event()

        def publish_commands() -> None:
            next_publish = time.monotonic()
            while not publisher_stop.is_set():
                seq = int(shm.command_seq)
                if seq & 1:
                    seq += 1
                shm.command_seq = seq + 1
                shm.command_q_des[0] += 0.0001
                shm.command_seq = seq + 2
                next_publish += 1.0 / bridge.DEFAULT_SETPOINT_HZ
                time.sleep(max(0.0, next_publish - time.monotonic()))

        publisher = threading.Thread(target=publish_commands, daemon=True)
        publisher.start()

        process = subprocess.Popen(
            [
                python,
                str(MODULE_ROOT / "wheeldog_mcu_bridge.py"),
                "--shm-name",
                shm_obj.name,
                "--ports",
                f"{mcu_a.port},{mcu_b.port}",
                "--verified-enable",
                "--rate-grace-s",
                "0.5",
                "--min-command-hz",
                "450",
                "--min-feedback-hz",
                "450",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                if shm.status_flags & bridge.BRIDGE_STATUS_READY:
                    break
                if process.poll() is not None:
                    break
                time.sleep(0.02)
            self.assertTrue(shm.status_flags & bridge.BRIDGE_STATUS_READY)
            self.assertTrue(shm.status_flags & bridge.BRIDGE_STATUS_ENABLED)

            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and shm.actual_hz < 170.0:
                time.sleep(0.02)
            self.assertAlmostEqual(
                shm.actual_hz,
                bridge.DEFAULT_SETPOINT_HZ,
                delta=30.0,
            )

            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and min(shm.motor_command_hz) < 450.0:
                time.sleep(0.05)
            self.assertGreaterEqual(min(shm.motor_command_hz), 450.0)
            self.assertLessEqual(max(shm.motor_command_hz), 550.0)
            self.assertGreaterEqual(min(shm.motor_feedback_hz), 450.0)
            self.assertLessEqual(max(shm.motor_feedback_hz), 550.0)
            self.assertEqual(shm.fast_feedback_valid_mask, 0xFFFF)
            self.assertEqual(mcu_a.qualification_mask, 0x0000)
            self.assertEqual(mcu_b.qualification_mask, 0x0000)
            self.assertGreater(shm.observation_seq, 0)
            self.assertLessEqual(shm.observation_max_sample_age_ms, 2)
            self.assertLessEqual(shm.observation_bridge_skew_us, 5000)
            self.assertAlmostEqual(shm.mcu_control_hz[0], 1000.0, delta=0.1)
            self.assertAlmostEqual(shm.mcu_control_hz[1], 1000.0, delta=0.1)
        finally:
            shm.control_flags |= bridge.CONTROL_EXIT
            try:
                stdout, stderr = process.communicate(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                stdout, stderr = process.communicate(timeout=3.0)
            if process.returncode != 0:
                status = bytes(shm.status_message).split(b"\0", 1)[0].decode(
                    errors="replace"
                )
                self.fail(
                    f"bridge exited {process.returncode}: {status}\n"
                    f"stdout={stdout}\nstderr={stderr}"
                )
            publisher_stop.set()
            publisher.join(timeout=1.0)
            del shm
            shm_obj.close()
            shm_obj.unlink()
            mcu_a.close()
            mcu_b.close()


if __name__ == "__main__":
    unittest.main()
