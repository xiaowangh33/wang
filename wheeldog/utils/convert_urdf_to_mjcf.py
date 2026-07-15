#!/usr/bin/env python3
"""
批量将 URDF 转换为 MJCF(XML)。

特点:
- 不依赖 urdfpy
- 直接调用仓库内 MuJoCo 的 `compile` 可执行文件
- 默认扫描: wheeldog_description/mywheeldog/urdf
- 默认输出: description
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def strip_contact_tags(src_urdf: Path) -> Path:
    """
    删除 URDF 中 link 下的 <contact> 标签，避免 MuJoCo URDF 解析报错。
    返回临时文件路径。
    """
    tree = ET.parse(src_urdf)
    root = tree.getroot()
    removed = 0
    for link in root.findall(".//link"):
        for node in list(link.findall("contact")):
            link.remove(node)
            removed += 1

    tmp = src_urdf.with_suffix(".tmp_no_contact.urdf")
    tree.write(tmp, encoding="utf-8", xml_declaration=True)
    if removed:
        print(f"[INFO] {src_urdf.name}: removed {removed} <contact> tag(s)")
    return tmp


def convert_one(compile_bin: Path, urdf_path: Path, out_xml: Path, strip_contact: bool) -> bool:
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    source = urdf_path
    tmp_file: Path | None = None
    try:
        if strip_contact:
            tmp_file = strip_contact_tags(urdf_path)
            source = tmp_file

        proc = subprocess.run(
            [str(compile_bin), str(source), str(out_xml)],
            capture_output=True,
            text=True,
            check=False,
        )
        msg = "\n".join(x for x in [proc.stdout, proc.stderr] if x).strip()

        # MuJoCo compile 在部分错误场景下可能返回 0，但日志里有 "Error:"
        if proc.returncode != 0 or "Error:" in msg or (not out_xml.exists()):
            err = msg if msg else f"compile failed with return code {proc.returncode}"
            print(f"[FAIL] {urdf_path}: {err}")
            return False

        print(f"[OK] {urdf_path} -> {out_xml}")
        return True
    finally:
        if tmp_file is not None and tmp_file.exists():
            tmp_file.unlink(missing_ok=True)


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser(description="Batch convert URDF to MJCF using MuJoCo compile.")
    parser.add_argument(
        "--urdf-root",
        type=Path,
        default=project_root / "wheeldog_description" / "mywheeldog" / "urdf",
        help="URDF 根目录",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=project_root / "description",
        help="输出 MJCF 根目录",
    )
    parser.add_argument(
        "--compile-bin",
        type=Path,
        default=project_root / "third_party" / "mujoco" / "x86" / "bin" / "compile",
        help="MuJoCo compile 可执行文件路径",
    )
    parser.add_argument(
        "--pattern",
        default="*.urdf",
        help="匹配 URDF 的 glob，默认 *.urdf",
    )
    parser.add_argument(
        "--keep-contact-tags",
        action="store_true",
        help="保留 <contact> 标签（默认会剥离）",
    )
    parser.add_argument(
        "--input-file",
        type=Path,
        default=None,
        help="单文件模式：指定输入 URDF 文件路径",
    )
    parser.add_argument(
        "--output-file",
        type=Path,
        default=None,
        help="单文件模式：指定输出 MJCF(XML) 文件路径",
    )
    args = parser.parse_args()

    compile_bin = args.compile_bin.resolve()
    strip_contact = not args.keep_contact_tags

    if not compile_bin.exists():
        print(f"[ERROR] compile 不存在: {compile_bin}")
        return 1

    # 单文件模式：要求 input/output 同时提供，避免误操作
    if (args.input_file is None) ^ (args.output_file is None):
        print("[ERROR] 单文件模式需要同时提供 --input-file 和 --output-file")
        return 1

    if args.input_file is not None and args.output_file is not None:
        input_file = args.input_file.resolve()
        output_file = args.output_file.resolve()
        if not input_file.exists():
            print(f"[ERROR] 输入文件不存在: {input_file}")
            return 1
        if input_file.suffix.lower() != ".urdf":
            print(f"[ERROR] 输入文件不是 .urdf: {input_file}")
            return 1

        print(f"[INFO] Single file mode")
        print(f"[INFO] Input      : {input_file}")
        print(f"[INFO] Output     : {output_file}")
        print(f"[INFO] Compile bin: {compile_bin}")

        ok = convert_one(compile_bin, input_file, output_file, strip_contact)
        print(f"[DONE] success={1 if ok else 0}, failed={0 if ok else 1}")
        return 0 if ok else 2

    urdf_root = args.urdf_root.resolve()
    out_root = args.output_root.resolve()
    if not urdf_root.exists():
        print(f"[ERROR] URDF 根目录不存在: {urdf_root}")
        return 1

    urdf_files = sorted(urdf_root.rglob(args.pattern))
    if not urdf_files:
        print(f"[WARN] 在 {urdf_root} 未找到 URDF (pattern={args.pattern})")
        return 0

    print(f"[INFO] URDF root   : {urdf_root}")
    print(f"[INFO] Output root : {out_root}")
    print(f"[INFO] Compile bin : {compile_bin}")
    print(f"[INFO] File count  : {len(urdf_files)}")

    ok = 0
    for urdf in urdf_files:
        rel = urdf.relative_to(urdf_root)
        out_xml = (out_root / rel).with_suffix(".xml")
        if convert_one(compile_bin, urdf, out_xml, strip_contact):
            ok += 1

    fail = len(urdf_files) - ok
    print(f"[DONE] success={ok}, failed={fail}")
    return 0 if fail == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
# python3 utils/convert_urdf_to_mjcf.py \
#   --input-file /home/gu/wheeldog/wheeldog_description/mywheeldog/urdf/wheel_legged_dog.urdf \
#   --output-file /home/gu/wheeldog/description/wheel_legged_dog_mujoco.xml
