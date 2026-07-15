#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RS06 电机 MIT 运控模式控制程序。

支持：
- 命令行参数快速下发
- 交互式命令修改 setpoint 并实时控制
- 串口收发打印，便于调试 USB 转 CAN 通信
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None  # type: ignore

from config import AppConfig, MitSetpoint
from motor import RS06Motor
from multi_motor import RS06Bus, parse_ids


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="灵足时代 RS06 MIT 运控控制程序")
    parser.add_argument("--port", default="COM4", help="USB转CAN串口号，默认 COM4")
    parser.add_argument("--baudrate", type=int, default=921600, help="串口波特率，默认 921600")
    parser.add_argument("--motor-id", type=int, default=1, help="单电机 CAN ID，默认 1")
    parser.add_argument("--motor-ids", default="", help="多电机 ID，逗号分隔，如 1,127")
    parser.add_argument("--host-id", type=lambda x: int(x, 0), default=0xFD, help="主机 ID，默认 0xFD")

    parser.add_argument("--pos", type=float, default=0.0, help="目标位置 (rad)")
    parser.add_argument("--vel", type=float, default=0.0, help="目标速度 (rad/s)")
    parser.add_argument("--kp", type=float, default=20.0, help="位置刚度 Kp")
    parser.add_argument("--kd", type=float, default=0.5, help="阻尼 Kd")
    parser.add_argument("--tau", type=float, default=0.0, help="前馈力矩 (N·m)")
    parser.add_argument(
        "--iq",
        type=float,
        default=0.5,
        help="力控模式 iq_ref 峰值电流 (Apeak)，默认 0.5",
    )

    parser.add_argument("--hz", type=float, default=50.0, help="控制循环频率 (Hz)")
    parser.add_argument("--duration", type=float, default=None, help="循环运行时长 (秒)")
    parser.add_argument("--loops", type=int, default=None, help="循环发送次数")

    parser.add_argument(
        "--mode",
        choices=["interactive", "once", "loop", "force", "dual-force", "enable", "disable", "ping", "read"],
        default="interactive",
        help="运行模式；force=力控(电流)模式循环，默认 interactive",
    )
    parser.add_argument("--quiet", action="store_true", help="减少打印输出")
    parser.add_argument("--no-feedback", action="store_true", help="不打印反馈帧")
    parser.add_argument("--skip-ensure-mit", action="store_true", help="使能时不写 run_mode=0")
    return parser


def config_from_args(args: argparse.Namespace) -> AppConfig:
    cfg = AppConfig()
    cfg.serial.port = args.port
    cfg.serial.baudrate = args.baudrate
    cfg.can.motor_id = args.motor_id
    cfg.can.host_id = args.host_id
    cfg.setpoint = MitSetpoint(
        position=args.pos,
        velocity=args.vel,
        kp=args.kp,
        kd=args.kd,
        torque=args.tau,
    )
    cfg.current.iq = args.iq
    if args.mode == "force":
        cfg.control.control_mode = "current"
    cfg.control.loop_hz = args.hz
    cfg.control.verbose = not args.quiet
    cfg.control.print_feedback = not args.no_feedback
    cfg.control.ensure_mit_mode = not args.skip_ensure_mit
    return cfg


def run_interactive(motor: RS06Motor) -> None:
    print("进入交互模式。输入 help 查看命令。")
    while True:
        try:
            line = input("rs06> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出交互模式")
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        args = parts[1:]

        try:
            if cmd in ("help", "?"):
                print_help()
            elif cmd in ("quit", "exit", "q"):
                break
            elif cmd == "show":
                motor.print_config()
            elif cmd == "ping":
                motor.ping()
            elif cmd == "enable":
                if motor.config.control.control_mode == "current":
                    motor.enable_current()
                else:
                    motor.enable()
            elif cmd == "disable":
                clear = bool(args and args[0] in ("1", "true", "clear"))
                motor.disable(clear_error=clear)
            elif cmd == "zero":
                motor.set_zero()
            elif cmd == "mit":
                motor.send_mit()
            elif cmd in ("force", "current", "iq"):
                motor.send_current()
            elif cmd == "read":
                motor.read_state()
            elif cmd == "loop":
                duration = float(args[0]) if len(args) >= 1 else 5.0
                motor.run_loop(duration_s=duration)
            elif cmd == "set":
                _handle_set_command(motor, args)
            elif cmd == "port":
                if not args:
                    print(f"当前串口: {motor.config.serial.port}")
                else:
                    motor.config.serial.port = args[0]
                    print(f"串口已设置为: {args[0]}")
            elif cmd == "motor_id":
                if not args:
                    print(f"当前电机ID: {motor.motor_id}")
                else:
                    motor.config.can.motor_id = int(args[0])
                    print(f"电机ID已设置为: {motor.config.can.motor_id}")
            else:
                print(f"未知命令: {cmd}，输入 help 查看帮助")
        except Exception as exc:
            print(f"命令执行失败: {exc}")


def _handle_set_command(motor: RS06Motor, args: list[str]) -> None:
    if len(args) < 2:
        print("用法: set <pos|vel|kp|kd|tau|iq|hz|mode> <value>")
        return
    key = args[0].lower()
    value = args[1]
    if key == "hz":
        motor.config.control.loop_hz = float(value)
        print(f"控制频率已设置为 {value} Hz")
        return
    if key == "mode":
        mode = value.lower()
        if mode not in ("mit", "current"):
            print("mode 仅支持 mit 或 current")
            return
        motor.config.control.control_mode = mode
        print(f"控制模式已设置为 {mode}")
        return
    if key == "iq":
        motor.current_setpoint.iq = float(value)
        print(f"iq_ref 已设置为 {motor.current_setpoint.iq:.3f} Apeak")
        return
    motor.update_setpoint(**{key: float(value)})


def print_help() -> None:
    print(
        """
可用命令:
  help                 显示帮助
  show                 打印当前配置
  ping                 获取设备 ID
  enable               使能（按当前控制模式）
  disable [clear]      失能；加 clear 可同时清错
  zero                 设置当前机械零位
  mit                  发送一帧 MIT 运控指令
  force / current / iq 发送一帧力控 iq_ref
  read                 读取 mechPos / mechVel 参数
  loop <seconds>       按当前模式循环控制
  set pos <rad>        设置目标位置
  set vel <rad/s>      设置目标速度
  set kp <value>       设置 Kp
  set kd <value>       设置 Kd
  set tau <Nm>         设置前馈力矩
  set iq <A>           设置力控电流给定
  set mode <mit|current> 切换控制模式
  set hz <Hz>          设置循环频率
  port [COMx]          查看/设置串口
  motor_id [id]        查看/设置电机 ID
  quit                 退出
"""
    )


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    config = config_from_args(args)

    motor_ids = parse_ids(args.motor_ids) if args.motor_ids else []
    use_bus = len(motor_ids) > 1 or args.mode == "dual-force"
    if use_bus and not motor_ids:
        motor_ids = [1, 127]

    motor = None
    bus = None
    try:
        if use_bus:
            bus = RS06Bus(
                config.serial.port,
                motor_ids,
                config.serial.baudrate,
                config.limits,
                config.control,
            )
            if len(motor_ids) >= 1:
                bus.channels[motor_ids[0]].current.iq = config.current.iq
            if len(motor_ids) >= 2:
                bus.channels[motor_ids[1]].current.iq = -config.current.iq
            bus.connect()
            print(f"多电机模式: IDs={motor_ids}")

            if args.mode in ("ping", "dual-force", "force"):
                hits = bus.ping_all()
                for mid, ok in hits.items():
                    print(f"  ID={mid}: {'在线' if ok else '无响应'}")
            if args.mode == "ping":
                return 0
            if args.mode in ("dual-force", "force"):
                bus.run_current_loop(duration_s=args.duration or 3.0, loop_count=args.loops)
                return 0
            parser.error(f"多电机模式暂不支持 --mode {args.mode}")
            return 1

        motor = RS06Motor(config)
        motor.connect()
        motor.print_config()

        if args.mode == "interactive":
            run_interactive(motor)
        elif args.mode == "ping":
            motor.ping()
        elif args.mode == "enable":
            motor.enable()
        elif args.mode == "disable":
            motor.disable()
        elif args.mode == "read":
            motor.read_state()
        elif args.mode == "once":
            motor.enable()
            time.sleep(0.05)
            motor.send_mit()
            time.sleep(0.1)
            motor.disable()
        elif args.mode == "loop":
            motor.run_loop(duration_s=args.duration, loop_count=args.loops)
        elif args.mode == "force":
            motor.run_current_loop(duration_s=args.duration or 3.0, loop_count=args.loops)
        else:
            parser.error(f"未知模式: {args.mode}")
    except KeyboardInterrupt:
        print("\n用户中断")
        try:
            if bus:
                bus.disable_all()
            elif motor:
                motor.disable()
        except Exception:
            pass
        return 130
    except Exception as exc:
        if serial and isinstance(exc, serial.SerialException):
            print(f"串口错误: {exc}")
            print("提示: 请先关闭上位机再运行")
            return 1
        raise
    finally:
        if bus:
            bus.disconnect()
        elif motor:
            motor.disconnect()

    return 0


if __name__ == "__main__":
    if serial is None:
        print("缺少依赖 pyserial，请执行: pip install -r requirements.txt")
        sys.exit(1)
    raise SystemExit(main())
