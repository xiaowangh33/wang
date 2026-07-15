# IMU调试工具使用说明

## ✅ 测试结果

使用原始hihost工具测试成功：
```bash
cd /home/gu/r_m_k_i_v1.2/imu/products-master/examples/C
sudo ./build/hihost -p /dev/ttyUSB0 -b 115200 read
```

## 重要发现

1. **波特率**: 当前 HI91 在 2026-07-10 断电重连后实测为 **115200**；
   以前日志中的 921600 不适用于当前上电配置。
2. **权限**: 可能需要sudo权限
3. **串口**: `/dev/ttyUSB0`

## 使用编译好的调试工具

```bash
cd /home/gu/r_m_k_i_v1.2/rl/utils
sudo ./imu_debug_tool /dev/ttyUSB0 115200
```

## 部署程序中的 IMU

`./build.sh hw` 后，`sudo -E ./run.sh` 会在硬件接口中启动 HiPNUC IMU 线程。常用变量：

```bash
export WHEELDOG_IMU_PORT=/dev/ttyUSB0
export WHEELDOG_IMU_BAUD=115200
export WHEELDOG_IMU_AXIS_MAP=x,y,z
export WHEELDOG_HW_TICK_HZ=200
sudo -E ./run.sh
```

坐标约定：驱动输出到 URDF body frame，`+X` 为机器狗头部，`+Y` 为左侧，`+Z` 为上侧。若现场快测发现 IMU 模组轴向不同，可用 `WHEELDOG_IMU_AXIS_MAP` 调整轴向，例如 `x,-y,-z`；姿态默认会按这个轴映射做旋转矩阵外参变换。`WHEELDOG_IMU_RPY_MAP` 是调试覆盖项，只有明确需要手动重排 RPY 分量时再设置。

## 部署 map 检验程序

编译：

```bash
utils/build_imu_mapping_check.sh
```

运行：

```bash
sudo ./utils/imu_mapping_check /dev/ttyUSB0 115200 x,y,z
```

第三个参数就是待验证的 `WHEELDOG_IMU_AXIS_MAP`，也可以不传，程序会读取环境变量。若要验证调试覆盖项 `WHEELDOG_IMU_RPY_MAP`，可加第四个参数：

```bash
sudo ./utils/imu_mapping_check /dev/ttyUSB0 115200 x,y,z x,y,z
```

水平静止时重点看：

- `mapped acc unit` 应接近 `[0, 0, +1]`
- `projected_gravity` 应接近 `[0, 0, -1]`
- `dot(acc_unit, -projected_gravity)` 应接近 `+1`
- `det` 应为 `+1`；若是 `-1`，说明 map 是左手系，不适合作为刚体安装外参

动作符号检查：

- 左侧抬高，roll 应变正
- 低头，pitch 应变正；抬头，pitch 应变负
- 原地左转，`gyro_z` / yaw 应变正

## 工具功能

- 实时显示IMU数据：
  - 姿态角（Roll, Pitch, Yaw）
  - 加速度（X, Y, Z）
  - 角速度（X, Y, Z）
  - 磁力计数据
  - 四元数
  - 温度、气压等

- 显示更新频率：每50ms更新一次
- 帧率统计：每秒显示一次数据接收帧率

## 退出

按 `Ctrl+C` 退出程序

## 故障排查

如果无法读取数据：
1. 检查串口权限：`ls -l /dev/ttyUSB0`
2. 确认当前硬件波特率是115200
3. 尝试使用sudo运行
4. 检查IMU是否已正确连接
