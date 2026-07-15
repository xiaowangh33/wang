# SANPO兴普智能 - 机器人集成开发板固件更新指南

## 准备工作
1. 准备STLINK(SWD)下载器，连接开发板STLINK接口与电脑
2. 开发板有两个STLINK接口，分别对应两颗STM32F407芯片，需分别更新固件
3. 更新时开发板需要使用USB或者其他5V口供电

<img src="../images/stlink_wire.png">

## 更新步骤
1. **安装工具**  
   下载并安装ST官方烧录软件：[STM32CubeProgrammer](https://gitcode.com/sanpo/robot/tree/v4/firmware/tools/STM32CubeProgrammer_win64.zip)

2. **获取固件**  
   下载最新原厂固件：[sanpo_robot_spine_board_firmware-v4-latest.bin](https://gitcode.com/sanpo/robot/tree/v4/firmware/sanpo_spine/Release/sanpo_robot_spine_board_firmware-v4-latest.bin)

3. **烧录固件**  
   断开开发板外部电源，使用STM32CubeProgrammer将固件刷入开发板（两颗芯片需分别操作）
   <img src="../images/update_firmware_stm32.png">