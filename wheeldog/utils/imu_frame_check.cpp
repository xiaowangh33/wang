/**
 * @file imu_frame_check.cpp
 * @brief 快速对照 URDF 基座 TORSO（+X前 +Y左 +Z上）与 HiPNUC 机体系是否一致
 *
 * 编译（在 rl/utils 目录下）:
 *   g++ -O2 -std=c++17 -o imu_frame_check imu_frame_check.cpp \
 *     ../../imu/products-master/drivers/hipnuc_dec.c \
 *     ../../imu/products-master/examples/C/serial_port.c \
 *     -I../../imu/products-master/drivers -I../../imu/products-master/examples/C \
 *     -lm
 *
 * 运行: sudo ./imu_frame_check /dev/ttyUSB0 115200
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "../../imu/products-master/drivers/hipnuc_dec.h"
#include "../../imu/products-master/examples/C/serial_port.h"

#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD 115200
#define DISPLAY_HZ 20.0
#define GYRO_STATIC_RADS (0.15f) /* 认为接近静止 */
#define STANDARD_GRAVITY_MPS2 (9.81f)
#define DEG_TO_RAD_F (0.01745329251994329577f)

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig) {
    (void)sig;
    g_running = 0;
}

static float v3_norm(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

static int dominant_axis(float x, float y, float z) {
    float ax = fabsf(x), ay = fabsf(y), az = fabsf(z);
    if (ax >= ay && ax >= az) return 0;
    if (ay >= ax && ay >= az) return 1;
    return 2;
}

static const char *axis_name(int i) {
    static const char *n[] = {"X", "Y", "Z"};
    return n[i & 3];
}

static void extract_imu(const hipnuc_raw_t *raw, float rpy_deg[3], float acc[3], float gyr[3],
                        int *ok) {
    if (raw->hi91.tag == 0x91) {
        rpy_deg[0] = raw->hi91.roll;
        rpy_deg[1] = raw->hi91.pitch;
        rpy_deg[2] = raw->hi91.yaw;
        acc[0] = raw->hi91.acc[0] * STANDARD_GRAVITY_MPS2;
        acc[1] = raw->hi91.acc[1] * STANDARD_GRAVITY_MPS2;
        acc[2] = raw->hi91.acc[2] * STANDARD_GRAVITY_MPS2;
        gyr[0] = raw->hi91.gyr[0] * DEG_TO_RAD_F;
        gyr[1] = raw->hi91.gyr[1] * DEG_TO_RAD_F;
        gyr[2] = raw->hi91.gyr[2] * DEG_TO_RAD_F;
        *ok = 1;
        return;
    }
    if (raw->hi83.tag == 0x83) {
        rpy_deg[0] = raw->hi83.rpy[0];
        rpy_deg[1] = raw->hi83.rpy[1];
        rpy_deg[2] = raw->hi83.rpy[2];
        acc[0] = raw->hi83.acc_b[0];
        acc[1] = raw->hi83.acc_b[1];
        acc[2] = raw->hi83.acc_b[2];
        gyr[0] = raw->hi83.gyr_b[0] * DEG_TO_RAD_F;
        gyr[1] = raw->hi83.gyr_b[1] * DEG_TO_RAD_F;
        gyr[2] = raw->hi83.gyr_b[2] * DEG_TO_RAD_F;
        *ok = 1;
        return;
    }
    *ok = 0;
}

static void print_live(const float rpy_deg[3], const float acc[3], const float gyr[3], int have_data) {
    printf("\033[2J\033[H");
    printf(" IMU 轴向快测 | URDF TORSO: +X前 +Y左 +Z上 | Ctrl+C 退出\n");
    printf("-----------------------------------------------------------------\n");
    printf(" 快测步骤：①水平静置→|acc|≈9.8，看重力落在哪根轴 ②抬头低头→pitch\n");
    printf(" ③左右侧倾→roll ④水平转弯→yaw/绕Z角速度\n");
    printf("-----------------------------------------------------------------\n");
    if (!have_data) {
        printf("等待 HiPNUC 数据帧 (0x91 或 0x83)...\n");
        fflush(stdout);
        return;
    }

    float an = v3_norm(acc[0], acc[1], acc[2]);
    float gn = v3_norm(gyr[0], gyr[1], gyr[2]);
    int dom = dominant_axis(acc[0], acc[1], acc[2]);
    int static_pose = (gn < GYRO_STATIC_RADS);

    printf("--- 实时（度 / m/s² / rad/s）---\n");
    printf(" RPY:  %8.2f %8.2f %8.2f   (roll pitch yaw)\n", rpy_deg[0], rpy_deg[1], rpy_deg[2]);
    printf(" Acc:  %8.3f %8.3f %8.3f   |a|=%.3f\n", acc[0], acc[1], acc[2], an);
    printf(" Gyr:  %8.3f %8.3f %8.3f   |ω|=%.3f\n", gyr[0], gyr[1], gyr[2], gn);

    printf("\n--- 快判 ---\n");
    if (an > 5.0f && an < 14.0f && static_pose) {
        printf(" 静止-ish：加速度模长合理；|acc| 最大分量轴: %s (索引 %d)\n",
               axis_name(dom), dom);
        float inv = 1.0f / an;
        printf(" 归一化 acc 方向: [%.2f, %.2f, %.2f]（静止时近似「上」在机体系投影）\n",
               acc[0] * inv, acc[1] * inv, acc[2] * inv);
        printf(" 若 URDF +Z 朝天且安装一致，通常 |Z| 最大且符号与模组定义相符。\n");
    } else if (!static_pose) {
        printf(" 正在运动：用角速度看绕哪根轴转最明显；与陀螺仪轴名对照 URDF。\n");
    } else {
        printf(" |acc|=%.2f 异常（应接近 9.8 附近）或数据未稳，先水平静置再读。\n", an);
    }

    printf("\n 提示：水平静止看 |acc| 与主轴；俯仰/横滚/转弯看 RPY 与陀螺。\n");
    fflush(stdout);
}

int main(int argc, char **argv) {
    const char *port = DEFAULT_PORT;
    int baud = DEFAULT_BAUD;
    if (argc >= 2) port = argv[1];
    if (argc >= 3) baud = atoi(argv[2]);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int fd = serial_port_open(port);
    if (fd < 0) {
        fprintf(stderr, "无法打开串口 %s: %s\n", port, strerror(errno));
        return 1;
    }
    if (serial_port_configure(fd, baud) < 0) {
        fprintf(stderr, "无法配置波特率 %d\n", baud);
        serial_port_close(fd);
        return 1;
    }

    char recv_buf[1024];
    serial_send_then_recv_str(fd, "LOG ENABLE\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);

    hipnuc_raw_t raw = {0};
    uint8_t read_buf[1024];

    struct timespec last_show, now;
    clock_gettime(CLOCK_MONOTONIC, &last_show);

    while (g_running) {
        int n = serial_port_read(fd, (char *)read_buf, sizeof(read_buf));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                hipnuc_input(&raw, read_buf[i]);
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        double dt = (now.tv_sec - last_show.tv_sec) + (now.tv_nsec - last_show.tv_nsec) * 1e-9;
        if (dt < 1.0 / DISPLAY_HZ) {
            usleep(500);
            continue;
        }
        last_show = now;

        float rpy_deg[3], acc[3], gyr[3];
        int ok = 0;
        extract_imu(&raw, rpy_deg, acc, gyr, &ok);
        print_live(rpy_deg, acc, gyr, ok);

        usleep(500);
    }

    printf("\033[0m\n");
    serial_port_close(fd);
    return 0;
}
