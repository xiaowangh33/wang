/**
 * @file imu_debug_tool.cpp
 * @brief IMU独立调试工具，用于测试IMU通信是否正常
 * @details 基于products-master/examples/C中的示例代码
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

// 包含IMU驱动头文件
#include "../../imu/products-master/drivers/hipnuc_dec.h"
#include "../../imu/products-master/examples/C/serial_port.h"

#define DISPLAY_UPDATE_INTERVAL 0.05  // 50ms更新一次显示
#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD 115200  // 2026-07-10 当前 HI91 上电配置实测
#define STANDARD_GRAVITY_MPS2 (9.81f)
#define DEG_TO_RAD_F (0.01745329251994329577f)

static bool g_running = true;

void signal_handler(int sig) {
    (void)sig;
    g_running = false;
    printf("\n[IMU调试工具] 收到退出信号，正在关闭...\n");
}

void print_imu_data(hipnuc_raw_t *raw) {
    printf("\033[H\033[J");  // 清屏
    printf("========== IMU 调试工具 ==========\n");
    printf("时间戳: %.3f 秒\n", (double)raw->hi91.system_time / 1000.0);
    
    if (raw->hi91.tag == 0x91) {
        printf("\n[HI91 数据包]\n");
        printf("温度: %d°C\n", raw->hi91.temp);
        printf("气压: %.2f Pa\n", raw->hi91.air_pressure);
        printf("\n加速度 (m/s²):\n");
        printf("  X: %8.3f  Y: %8.3f  Z: %8.3f\n", 
               raw->hi91.acc[0] * STANDARD_GRAVITY_MPS2,
               raw->hi91.acc[1] * STANDARD_GRAVITY_MPS2,
               raw->hi91.acc[2] * STANDARD_GRAVITY_MPS2);
        printf("\n角速度 (rad/s):\n");
        printf("  X: %8.3f  Y: %8.3f  Z: %8.3f\n", 
               raw->hi91.gyr[0] * DEG_TO_RAD_F,
               raw->hi91.gyr[1] * DEG_TO_RAD_F,
               raw->hi91.gyr[2] * DEG_TO_RAD_F);
        printf("\n磁力计 (uT):\n");
        printf("  X: %8.3f  Y: %8.3f  Z: %8.3f\n", 
               raw->hi91.mag[0], raw->hi91.mag[1], raw->hi91.mag[2]);
        printf("\n姿态角 (度):\n");
        printf("  Roll:  %8.3f°\n", raw->hi91.roll);
        printf("  Pitch: %8.3f°\n", raw->hi91.pitch);
        printf("  Yaw:   %8.3f°\n", raw->hi91.yaw);
        printf("\n四元数:\n");
        printf("  W: %8.3f  X: %8.3f  Y: %8.3f  Z: %8.3f\n",
               raw->hi91.quat[0], raw->hi91.quat[1], 
               raw->hi91.quat[2], raw->hi91.quat[3]);
    }
    
    if (raw->hi83.tag == 0x83) {
        printf("\n[HI83 数据包]\n");
        printf("系统时间: %llu us\n", (unsigned long long)raw->hi83.system_time_us);
        printf("\n加速度 (m/s²):\n");
        printf("  X: %8.3f  Y: %8.3f  Z: %8.3f\n", 
               raw->hi83.acc_b[0], raw->hi83.acc_b[1], raw->hi83.acc_b[2]);
        printf("\n角速度 (rad/s):\n");
        printf("  X: %8.3f  Y: %8.3f  Z: %8.3f\n", 
               raw->hi83.gyr_b[0] * DEG_TO_RAD_F,
               raw->hi83.gyr_b[1] * DEG_TO_RAD_F,
               raw->hi83.gyr_b[2] * DEG_TO_RAD_F);
        printf("\n姿态角 (度):\n");
        printf("  Roll:  %8.3f°\n", raw->hi83.rpy[0]);
        printf("  Pitch: %8.3f°\n", raw->hi83.rpy[1]);
        printf("  Yaw:   %8.3f°\n", raw->hi83.rpy[2]);
    }
    
    printf("\n==================================\n");
    printf("按 Ctrl+C 退出\n");
}

int main(int argc, char *argv[]) {
    const char *port_name = DEFAULT_PORT;
    int baud_rate = DEFAULT_BAUD;
    
    // 解析命令行参数
    if (argc >= 2) {
        port_name = argv[1];
    }
    if (argc >= 3) {
        baud_rate = atoi(argv[2]);
    }
    
    printf("[IMU调试工具] 初始化...\n");
    printf("串口: %s\n", port_name);
    printf("波特率: %d\n", baud_rate);
    
    // 注册信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 打开串口
    int fd = serial_port_open(port_name);
    if (fd < 0) {
        fprintf(stderr, "[错误] 无法打开串口 %s\n", port_name);
        return -1;
    }
    
    if (serial_port_configure(fd, baud_rate) < 0) {
        fprintf(stderr, "[错误] 无法配置串口波特率 %d\n", baud_rate);
        serial_port_close(fd);
        return -1;
    }
    
    printf("[IMU调试工具] 串口打开成功\n");
    printf("[IMU调试工具] 启动数据读取...\n");
    
    // 发送启动命令
    char recv_buf[1024];
    serial_send_then_recv_str(fd, "LOG ENABLE\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);
    
    // 初始化数据结构
    hipnuc_raw_t hipnuc_raw = {0};
    uint8_t read_buf[1024];
    struct timespec last_display_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &last_display_time);
    
    long long frame_count = 0;
    int frame_rate = 0;
    struct timespec last_rate_time = last_display_time;
    
    printf("\n[IMU调试工具] 开始读取数据...\n");
    printf("等待3秒后开始显示...\n");
    sleep(3);
    
    // 主循环
    while (g_running) {
        // 读取串口数据
        int len = serial_port_read(fd, (char *)read_buf, sizeof(read_buf));
        if (len > 0) {
            // 处理每个字节
            for (int i = 0; i < len; i++) {
                if (hipnuc_input(&hipnuc_raw, read_buf[i]) > 0) {
                    frame_count++;
                    
                    // 检查是否需要更新显示
                    clock_gettime(CLOCK_MONOTONIC, &current_time);
                    double display_elapsed = (current_time.tv_sec - last_display_time.tv_sec) +
                                           (current_time.tv_nsec - last_display_time.tv_nsec) / 1e9;
                    
                    if (display_elapsed >= DISPLAY_UPDATE_INTERVAL) {
                        print_imu_data(&hipnuc_raw);
                        last_display_time = current_time;
                    }
                }
            }
        }
        
        // 计算帧率
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double rate_elapsed = (current_time.tv_sec - last_rate_time.tv_sec) +
                             (current_time.tv_nsec - last_rate_time.tv_nsec) / 1e9;
        if (rate_elapsed >= 1.0) {
            frame_rate = (int)(frame_count / rate_elapsed);
            frame_count = 0;
            last_rate_time = current_time;
            printf("\n[统计] 帧率: %d Hz\n", frame_rate);
        }
        
        usleep(1000);  // 1ms延迟
    }
    
    printf("\n[IMU调试工具] 关闭串口...\n");
    serial_port_close(fd);
    printf("[IMU调试工具] 退出\n");
    
    return 0;
}
