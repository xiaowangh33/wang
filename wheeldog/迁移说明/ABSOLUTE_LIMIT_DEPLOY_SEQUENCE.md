# 机械绝对限位固件：烧录与首次方向复核顺序

以下命令以 `/home/gu/wheel_dog_pc_mcu` 为工作目录。整机必须可靠吊装或支撑，轮子和
所有连杆周围无人；急停可立即触达。第一次方向检查只用资格激励的 `0.6 Nm/1 rad/s`，
不要在未吊装、未确认 raw 边界时进入已开放到 36 Nm 的 StandUp。

## 1. 构建

```bash
cd /home/gu/wheel_dog_pc_mcu/robot-v4-firmware-sanpo_spine/firmware/sanpo_spine
make host-test
make release-mcu-a
make release-mcu-b
```

正式固件同时保留只读 raw 角监视能力：

```text
build/Release/bus0_bench0_live1_rawread1/sanpo_spine.bin  # MCU A, CAN1/CAN2
build/Release/bus2_bench0_live1_rawread1/sanpo_spine.bin  # MCU B, CAN3/CAN4
```

## 2. 依次烧录两块 MCU

ST-LINK 一次只连接目标板。先接 MCU A：

```bash
cd /home/gu/wheel_dog_pc_mcu/robot-v4-firmware-sanpo_spine/firmware/sanpo_spine
LD_LIBRARY_PATH=/home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/lib/x86_64-linux-gnu \
/home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/bin/st-flash \
  --connect-under-reset --reset write \
  build/Release/bus0_bench0_live1_rawread1/sanpo_spine.bin 0x08000000
```

看到 `Flash written and verified` 后手动 reset MCU A。再断开 ST-LINK、接 MCU B：

```bash
LD_LIBRARY_PATH=/home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/lib/x86_64-linux-gnu \
/home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/bin/st-flash \
  --connect-under-reset --reset write \
  build/Release/bus2_bench0_live1_rawread1/sanpo_spine.bin 0x08000000
```

看到验证成功后手动 reset MCU B。两块板最终都接回 USB 和动力电。

## 3. 识别端口并做 16 路只读检查

```bash
cd /home/gu/wheel_dog_pc_mcu
PY=/home/gu/anaconda3/bin/python3

$PY wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM0 --duration 2
$PY wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM1 --duration 2
```

输出带 `mcu-base2` 的端口是 MCU B，另一个是 MCU A。按实际结果设置，例如：

```bash
PORT_A=/dev/ttyACM1
PORT_B=/dev/ttyACM0
```

然后验证映射并读 raw：

```bash
$PY wheeldog/tools/verify_all_can_joint_mapping.py \
  --ports "$PORT_A,$PORT_B" --confirm-supported

$PY wheeldog/tools/read_all_motor_angles.py \
  --ports "$PORT_A,$PORT_B" --confirm-supported --once --no-clear
```

必须满足 16/16 在线、CAN1~CAN4 均为唯一 ID1~4、没有 enabled/fault，并检查 12 个
腿关节 raw 都位于映射表边界内部。膝关节当前外推边界尤其要留意。

## 4. 一次一个电机确认正方向

通用腿关节命令（`INDEX` 依次替换）：

```bash
$PY wheeldog/tools/wd_mcu_dryrun.py --port "$PORT" --duration 2 \
  --live-control --enable-request --i-understand-live-can \
  --qualification-excitation --joint-torque INDEX:0.6 --print-joints
```

轮子命令：

```bash
$PY wheeldog/tools/wd_mcu_dryrun.py --port "$PORT" --duration 2 \
  --live-control --enable-request --i-understand-live-can \
  --qualification-excitation --joint-velocity INDEX:1.0 --print-joints
```

执行顺序：

```text
MCU A / PORT_A: 0 FL_HipX, 1 FL_HipY, 2 FL_Knee, 3 FL_Ankle,
                4 FR_HipX, 5 FR_HipY, 6 FR_Knee, 7 FR_Ankle
MCU B / PORT_B: 8 HL_HipX, 9 HL_HipY, 10 HL_Knee, 11 HL_Ankle,
                12 HR_HipX, 13 HR_HipY, 14 HR_Knee, 15 HR_Ankle
```

对腿关节，`+0.6 Nm` 应使反馈 URDF `q` 朝正方向变化；对轮子，`+1 rad/s` 应符合期望
前进符号。若任一项相反，立即 Ctrl+C，不要反复测试，也不要进入整机部署；修改唯一
映射表中的该电机 `direction` 后，重新构建并重刷两块对应固件。

每个电机退出后再发一次普通 dry-run 清理：

```bash
$PY wheeldog/tools/wd_mcu_dryrun.py --port "$PORT" --duration 1 --print-joints
```

## 5. 构建并启动整体部署

只有 16 个方向全部确认后执行：

```bash
cd /home/gu/wheel_dog_pc_mcu/wheeldog
./build.sh hw
sudo -E env \
  WHEELDOG_MOTOR_PORTS="$PORT_A,$PORT_B" \
  WHEELDOG_PYTHON="$PY" \
  ./run.sh
```

第一次 StandUp 保持吊装，持续观察 `raw-position-limit`、`measured-overspeed`、
`fault`、16 路 500 Hz 和 URDF q 符号。任何 raw 墙持续介入或姿态朝错误方向发展都应
立即急停；不要用提高力矩掩盖标定或方向错误。
