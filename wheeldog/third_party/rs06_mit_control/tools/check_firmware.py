#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""查询 SANPO 板固件版本（AT+VER，需 CRLF 结束符）。

手册: https://gitcode.com/sanpo/robot/blob/v4/docs/zh/usb_can.md
- V4.1+ : USB-CAN ET 协议可用
- V46   : 较新，支持 SocketCAN
- V33 或无响应: 建议刷最新 V4 原厂固件
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("请先安装: pip install pyserial")
    raise SystemExit(1)

from sanpo_bus_map import MCU1_COM, MCU2_COM

AT_VER = b"AT+VER\r\n"
AT_ET = b"AT+ET\r\n"  # 从小米模式切回正常模式（手册提及）


def interpret_version(text: str) -> str:
    t = text.strip().upper()
    if not t:
        return "无有效版本字符串"
    if "V33" in t:
        return "版本过旧 (V33)，建议刷最新 V4 原厂固件"
    if "V46" in t or "V4.6" in t:
        return "较新版本，支持 USB-CAN ET 协议及 SocketCAN"
    if any(x in t for x in ("V41", "V42", "V43", "V44", "V45", "V4.")):
        return "V4.1+，USB-CAN ET 协议应可用"
    if t.startswith("V") and any(c.isdigit() for c in t):
        return "收到版本号，请对照 SANPO 发布说明"
    return "响应内容异常，请人工核对"


def query_port(port: str, baud: int, timeout: float) -> dict:
    result = {
        "port": port,
        "opened": False,
        "version_raw": "",
        "interpret": "",
        "extra_rx": "",
        "error": "",
    }
    try:
        ser = serial.Serial(
            port,
            baud,
            timeout=timeout,
            dsrdtr=False,
            rtscts=False,
        )
        ser.dtr = True
        ser.rts = False
        time.sleep(0.08)
        ser.reset_input_buffer()
        result["opened"] = True

        ser.write(AT_VER)
        ser.flush()
        time.sleep(0.25)
        rx = b""
        deadline = time.time() + 0.8
        while time.time() < deadline:
            waiting = ser.in_waiting
            if waiting:
                rx += ser.read(waiting)
            else:
                time.sleep(0.02)
        if ser.in_waiting:
            rx += ser.read(ser.in_waiting)

        text = rx.decode("utf-8", errors="replace").strip()
        result["version_raw"] = text or "(无回传)"
        result["interpret"] = interpret_version(text)

        # 顺带发一帧 ET ping 看 CAN 网关是否活着
        ping = bytes([0x45, 0x54, 0x00, 0x00, 0x00, 0x01, 0xFE, 0x02, 0x01, 0x00, 0x0D, 0x0A])
        ser.reset_input_buffer()
        ser.write(ping)
        ser.flush()
        time.sleep(0.15)
        rx2 = b""
        if ser.in_waiting:
            rx2 = ser.read(ser.in_waiting)
        if rx2:
            result["extra_rx"] = rx2.hex()
        ser.close()
    except (serial.SerialException, OSError) as exc:
        result["error"] = str(exc)
    return result


def print_result(r: dict) -> None:
    mcu = "MCU1 (CAN1/CAN2)" if r["port"].upper() == MCU1_COM.upper() else "MCU2 (CAN3/CAN4)"
    if r["port"].upper() not in (MCU1_COM.upper(), MCU2_COM.upper()):
        mcu = "未知 MCU 映射"

    print(f"\n{'='*60}")
    print(f"串口: {r['port']}  [{mcu}]")
    if r["error"]:
        print(f"  状态: 无法打开 — {r['error']}")
        return
    print(f"  状态: 已连接")
    print(f"  AT+VER 回传: {r['version_raw']}")
    print(f"  判定: {r['interpret']}")
    if r["extra_rx"]:
        print(f"  ET 探测回传: {r['extra_rx'][:80]}{'...' if len(r['extra_rx'])>80 else ''}")
    else:
        print(f"  ET 探测: 无 CAN 回传（可能无电机或 Channel/COM 不对）")


def main() -> int:
    ap = argparse.ArgumentParser(description="SANPO 板固件版本查询 (AT+VER)")
    ap.add_argument("--ports", default=f"{MCU1_COM},{MCU2_COM}")
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--timeout", type=float, default=0.5)
    args = ap.parse_args()

    if args.list_ports:
        print("可用串口:")
        for p in list_ports.comports():
            mark = ""
            if "STMicroelectronics" in (p.description or ""):
                mark = "  <-- 可能是 SANPO STM32 VCP"
            print(f"  {p.device}\t{p.description}{mark}")
        return 0

    ports = [p.strip() for p in args.ports.split(",") if p.strip()]
    print("SANPO 固件检查")
    print(f"发送指令: AT+VER (CRLF) @ {args.baudrate}")
    print("参考: V4.1+ 正常 | V33 需刷固件 | 无响应检查 USB/驱动")

    results = [query_port(p, args.baudrate, args.timeout) for p in ports]
    for r in results:
        print_result(r)

    ok = sum(1 for r in results if r["opened"] and r["version_raw"] not in ("", "(无回传)"))
    print(f"\n{'='*60}")
    if ok == len(ports):
        print("结论: 两个 MCU 串口均有版本回传，固件通信正常。")
        return 0
    if ok > 0:
        print(f"结论: {ok}/{len(ports)} 个串口有版本回传，请检查未响应的 MCU 或固件是否已刷。")
        return 1
    print("结论: 无有效版本回传。请检查 USB 连接、COM 口、驱动，或刷 SANPO V4 原厂固件。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
