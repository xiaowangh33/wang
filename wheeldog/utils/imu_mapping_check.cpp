/**
 * @file imu_mapping_check.cpp
 * @brief Standalone HiPNUC IMU frame/map checker for the wheeldog URDF body frame.
 *
 * URDF body frame:
 *   +X forward/head, +Y left, +Z up.
 *
 * Static level expectations:
 *   mapped accelerometer specific force ~= [0, 0, +9.81] m/s^2
 *   RPY-derived projected gravity        ~= [0, 0, -1]
 *   dot(normalized_acc, -projected_gravity) ~= +1
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>

extern "C" {
#include "hipnuc_dec.h"
#include "serial_port.h"
}

namespace {

constexpr float kDegToRad = 0.01745329251994329577f;
constexpr float kRadToDeg = 57.295779513082320876f;
constexpr float kGravity = 9.81f;
constexpr float kGyroStaticRadps = 0.15f;

std::atomic<bool> g_running{true};

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Mat3 {
    float v[3][3] = {};
};

struct AxisMap {
    std::array<int, 3> index{{0, 1, 2}};
    std::array<float, 3> sign{{1.0f, 1.0f, 1.0f}};
};

struct Sample {
    bool ok = false;
    const char* packet = "none";
    Vec3 raw_rpy_rad{};
    Vec3 raw_acc{};
    Vec3 raw_gyro{};
};

void OnSignal(int) {
    g_running.store(false);
}

float Norm(Vec3 a) {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

float Dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 Normalize(Vec3 a) {
    const float n = Norm(a);
    if (n < 1e-6f) {
        return {};
    }
    return {a.x / n, a.y / n, a.z / n};
}

Vec3 Neg(Vec3 a) {
    return {-a.x, -a.y, -a.z};
}

std::string Trim(std::string s) {
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    while (!s.empty() && !not_space(static_cast<unsigned char>(s.front()))) {
        s.erase(s.begin());
    }
    while (!s.empty() && !not_space(static_cast<unsigned char>(s.back()))) {
        s.pop_back();
    }
    return s;
}

bool ParseAxisMap(const std::string& text, AxisMap* out) {
    AxisMap map;
    std::array<bool, 3> used{{false, false, false}};
    std::stringstream ss(text);
    std::string token;

    for (int axis = 0; axis < 3; ++axis) {
        if (!std::getline(ss, token, ',')) {
            return false;
        }
        token = Trim(token);
        float sign = 1.0f;
        if (!token.empty() && token.front() == '+') {
            token.erase(token.begin());
        } else if (!token.empty() && token.front() == '-') {
            sign = -1.0f;
            token.erase(token.begin());
        }
        token = Trim(token);
        if (token.size() != 1) {
            return false;
        }
        const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(token[0])));
        int idx = -1;
        if (c == 'x') idx = 0;
        if (c == 'y') idx = 1;
        if (c == 'z') idx = 2;
        if (idx < 0 || used[idx]) {
            return false;
        }
        used[idx] = true;
        map.index[axis] = idx;
        map.sign[axis] = sign;
    }

    if (std::getline(ss, token, ',')) {
        return false;
    }
    *out = map;
    return true;
}

std::string FormatAxisMap(const AxisMap& map) {
    static constexpr char names[] = {'x', 'y', 'z'};
    std::ostringstream oss;
    for (int i = 0; i < 3; ++i) {
        if (i > 0) oss << ",";
        oss << (map.sign[i] < 0.0f ? "-" : "") << names[map.index[i]];
    }
    return oss.str();
}

float Component(Vec3 a, int idx) {
    if (idx == 0) return a.x;
    if (idx == 1) return a.y;
    return a.z;
}

Vec3 ApplyMap(Vec3 in, const AxisMap& map) {
    return {
        map.sign[0] * Component(in, map.index[0]),
        map.sign[1] * Component(in, map.index[1]),
        map.sign[2] * Component(in, map.index[2]),
    };
}

Mat3 AxisMapMatrix(const AxisMap& map) {
    Mat3 m{};
    for (int row = 0; row < 3; ++row) {
        m.v[row][map.index[row]] = map.sign[row];
    }
    return m;
}

float Det(Mat3 m) {
    return m.v[0][0] * (m.v[1][1] * m.v[2][2] - m.v[1][2] * m.v[2][1]) -
           m.v[0][1] * (m.v[1][0] * m.v[2][2] - m.v[1][2] * m.v[2][0]) +
           m.v[0][2] * (m.v[1][0] * m.v[2][1] - m.v[1][1] * m.v[2][0]);
}

Mat3 Mul(Mat3 a, Mat3 b) {
    Mat3 out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            for (int k = 0; k < 3; ++k) {
                out.v[r][c] += a.v[r][k] * b.v[k][c];
            }
        }
    }
    return out;
}

Mat3 Transpose(Mat3 m) {
    Mat3 out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.v[r][c] = m.v[c][r];
        }
    }
    return out;
}

Vec3 Mul(Mat3 m, Vec3 a) {
    return {
        m.v[0][0] * a.x + m.v[0][1] * a.y + m.v[0][2] * a.z,
        m.v[1][0] * a.x + m.v[1][1] * a.y + m.v[1][2] * a.z,
        m.v[2][0] * a.x + m.v[2][1] * a.y + m.v[2][2] * a.z,
    };
}

Mat3 RpyToMatrix(Vec3 rpy) {
    const float cr = std::cos(rpy.x), sr = std::sin(rpy.x);
    const float cp = std::cos(rpy.y), sp = std::sin(rpy.y);
    const float cy = std::cos(rpy.z), sy = std::sin(rpy.z);

    Mat3 r{};
    r.v[0][0] = cy * cp;
    r.v[0][1] = cy * sp * sr - sy * cr;
    r.v[0][2] = cy * sp * cr + sy * sr;
    r.v[1][0] = sy * cp;
    r.v[1][1] = sy * sp * sr + cy * cr;
    r.v[1][2] = sy * sp * cr - cy * sr;
    r.v[2][0] = -sp;
    r.v[2][1] = cp * sr;
    r.v[2][2] = cp * cr;
    return r;
}

Vec3 MatrixToRpy(Mat3 rm) {
    const float pitch_arg = std::fmax(-1.0f, std::fmin(1.0f, -rm.v[2][0]));
    return {
        std::atan2(rm.v[2][1], rm.v[2][2]),
        std::asin(pitch_arg),
        std::atan2(rm.v[1][0], rm.v[0][0]),
    };
}

Vec3 MapRpyByMatrix(Vec3 raw_rpy, const AxisMap& map, bool* right_handed) {
    const Mat3 sensor_to_body = AxisMapMatrix(map);
    const float det = Det(sensor_to_body);
    *right_handed = det > 0.5f;
    if (!*right_handed) {
        return ApplyMap(raw_rpy, map);
    }
    return MatrixToRpy(Mul(RpyToMatrix(raw_rpy), Transpose(sensor_to_body)));
}

Sample Extract(const hipnuc_raw_t& raw) {
    Sample s;
    if (raw.hi91.tag == 0x91) {
        s.ok = true;
        s.packet = "HI91";
        s.raw_rpy_rad = {
            raw.hi91.roll * kDegToRad,
            raw.hi91.pitch * kDegToRad,
            raw.hi91.yaw * kDegToRad,
        };
        s.raw_acc = {
            raw.hi91.acc[0] * kGravity,
            raw.hi91.acc[1] * kGravity,
            raw.hi91.acc[2] * kGravity,
        };
        s.raw_gyro = {
            raw.hi91.gyr[0] * kDegToRad,
            raw.hi91.gyr[1] * kDegToRad,
            raw.hi91.gyr[2] * kDegToRad,
        };
    } else if (raw.hi83.tag == 0x83) {
        s.ok = true;
        s.packet = "HI83";
        s.raw_rpy_rad = {
            raw.hi83.rpy[0] * kDegToRad,
            raw.hi83.rpy[1] * kDegToRad,
            raw.hi83.rpy[2] * kDegToRad,
        };
        s.raw_acc = {raw.hi83.acc_b[0], raw.hi83.acc_b[1], raw.hi83.acc_b[2]};
        s.raw_gyro = {
            raw.hi83.gyr_b[0] * kDegToRad,
            raw.hi83.gyr_b[1] * kDegToRad,
            raw.hi83.gyr_b[2] * kDegToRad,
        };
    }
    return s;
}

int DominantAxis(Vec3 a) {
    const float ax = std::fabs(a.x), ay = std::fabs(a.y), az = std::fabs(a.z);
    if (ax >= ay && ax >= az) return 0;
    if (ay >= ax && ay >= az) return 1;
    return 2;
}

std::string TokenForRawAxis(int axis, float value) {
    static constexpr char names[] = {'x', 'y', 'z'};
    std::string token;
    if (value < 0.0f) token += "-";
    token += names[axis];
    return token;
}

void PrintVec(const char* label, Vec3 a, const char* unit = "") {
    std::printf(" %-20s [%8.3f %8.3f %8.3f] %s\n", label, a.x, a.y, a.z, unit);
}

void PrintLive(const Sample& s,
               const AxisMap& map,
               const AxisMap& rpy_map,
               bool use_rpy_component_map,
               uint64_t frames,
               float hz) {
    bool right_handed = false;
    const Vec3 raw_acc_unit = Normalize(s.raw_acc);
    const Vec3 mapped_rpy = use_rpy_component_map
        ? ApplyMap(s.raw_rpy_rad, rpy_map)
        : MapRpyByMatrix(s.raw_rpy_rad, map, &right_handed);
    if (use_rpy_component_map) {
        right_handed = Det(AxisMapMatrix(map)) > 0.5f;
    }
    const Vec3 mapped_acc = ApplyMap(s.raw_acc, map);
    const Vec3 mapped_gyro = ApplyMap(s.raw_gyro, map);
    const Vec3 mapped_acc_unit = Normalize(mapped_acc);
    const Mat3 body_rm = RpyToMatrix(mapped_rpy);
    const Vec3 projected_gravity = Mul(Transpose(body_rm), Vec3{0.0f, 0.0f, -1.0f});
    const float acc_gravity_dot = Dot(mapped_acc_unit, Neg(projected_gravity));
    const bool static_pose = Norm(mapped_gyro) < kGyroStaticRadps;
    const int raw_dom = DominantAxis(s.raw_acc);
    const float raw_dom_value = Component(s.raw_acc, raw_dom);

    std::printf("\033[2J\033[H");
    std::printf("IMU map check | URDF body: +X head, +Y left, +Z up | Ctrl+C exit\n");
    std::printf("map=%s det=%.0f (%s) rpy=%s packet=%s frames=%llu rate=%.1f Hz\n",
                FormatAxisMap(map).c_str(), Det(AxisMapMatrix(map)),
                right_handed ? "right-handed" : "LEFT-HANDED/RPY fallback",
                use_rpy_component_map ? FormatAxisMap(rpy_map).c_str() : "matrix-from-map",
                s.packet, static_cast<unsigned long long>(frames), hz);
    std::printf("--------------------------------------------------------------------------\n");
    std::printf("Static level expectation:\n");
    std::printf("  accelerometer specific force ~= [0,0,+9.81] m/s^2 (up)\n");
    std::printf("  policy projected_gravity     ~= [0,0,-1]       (down)\n");
    std::printf("  dot(acc_unit, -projected_gravity) should be close to +1\n");
    std::printf("--------------------------------------------------------------------------\n");

    Vec3 raw_rpy_deg{s.raw_rpy_rad.x * kRadToDeg, s.raw_rpy_rad.y * kRadToDeg,
                     s.raw_rpy_rad.z * kRadToDeg};
    Vec3 mapped_rpy_deg{mapped_rpy.x * kRadToDeg, mapped_rpy.y * kRadToDeg,
                        mapped_rpy.z * kRadToDeg};

    PrintVec("raw rpy", raw_rpy_deg, "deg");
    PrintVec("raw acc", s.raw_acc, "m/s^2");
    PrintVec("raw acc unit", raw_acc_unit);
    PrintVec("raw gyro", s.raw_gyro, "rad/s");
    std::printf("\n");
    PrintVec("mapped rpy", mapped_rpy_deg, "deg");
    PrintVec("mapped acc", mapped_acc, "m/s^2");
    PrintVec("mapped acc unit", mapped_acc_unit);
    PrintVec("mapped gyro", mapped_gyro, "rad/s");
    PrintVec("projected_gravity", projected_gravity);
    std::printf(" gravity consistency dot(acc_unit, -projected_gravity): %.3f\n",
                acc_gravity_dot);

    std::printf("\nQuick verdict:\n");
    if (!right_handed) {
        std::printf("  ! map is left-handed. For a rigid IMU mount, prefer a det=+1 map; flip two axes, not one.\n");
    }
    if (static_pose) {
        if (Norm(mapped_acc) < 7.0f || Norm(mapped_acc) > 12.5f) {
            std::printf("  ! |acc| is not near 9.81. Keep the robot still and check IMU mode/units.\n");
        }
        if (mapped_acc_unit.z > 0.80f && std::fabs(mapped_acc_unit.x) < 0.45f &&
            std::fabs(mapped_acc_unit.y) < 0.45f) {
            std::printf("  OK level acc points mostly body +Z.\n");
        } else if (mapped_acc_unit.z < -0.80f) {
            std::printf("  ! mapped acc points body -Z. Flip the sign of the Z source in WHEELDOG_IMU_AXIS_MAP.\n");
        } else {
            std::printf("  ! mapped acc is not mostly +Z. Level-only hint: third map token likely '%s'.\n",
                        TokenForRawAxis(raw_dom, raw_dom_value).c_str());
        }
        if (acc_gravity_dot > 0.85f) {
            std::printf("  OK acc(up) and RPY-derived gravity(down) are consistent.\n");
        } else if (acc_gravity_dot < -0.85f) {
            std::printf("  ! acc and projected_gravity have the same direction; gravity sign/RPY mapping is inverted.\n");
        } else {
            std::printf("  ! acc and RPY gravity disagree. Suspect RPY convention or axis map, not just acc units.\n");
        }
    } else {
        std::printf("  Robot/IMU is moving. Hold still for gravity checks; use mapped gyro for turn-axis checks.\n");
    }

    std::printf("\nMotion sign checks after level check:\n");
    std::printf("  left side up  => roll should become positive\n");
    std::printf("  nose down     => pitch should become positive (nose up => negative)\n");
    std::printf("  turn left     => gyro_z/yaw should become positive\n");
    std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
    const char* port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    const int baud = argc > 2 ? std::atoi(argv[2]) : 921600;
    // Defaults for the installed HI91, measured against the URDF body frame.
    std::string map_text = "y,-x,z";
    if (const char* env_map = std::getenv("WHEELDOG_IMU_AXIS_MAP")) {
        if (env_map[0] != '\0') map_text = env_map;
    }
    if (argc > 3) {
        map_text = argv[3];
    }

    AxisMap map;
    if (!ParseAxisMap(map_text, &map)) {
        std::fprintf(stderr, "Invalid map '%s'. Expected e.g. x,y,z or x,-y,-z\n",
                     map_text.c_str());
        return 2;
    }

    bool use_rpy_component_map = true;
    std::string rpy_map_text = "x,-y,z";
    if (const char* env_rpy_map = std::getenv("WHEELDOG_IMU_RPY_MAP")) {
        if (env_rpy_map[0] != '\0') {
            rpy_map_text = env_rpy_map;
            use_rpy_component_map = true;
        }
    }
    if (argc > 4) {
        rpy_map_text = argv[4];
        use_rpy_component_map = true;
    }
    AxisMap rpy_map;
    if (!ParseAxisMap(rpy_map_text, &rpy_map)) {
        std::fprintf(stderr, "Invalid RPY map '%s'. Expected e.g. x,y,z or x,-y,-z\n",
                     rpy_map_text.c_str());
        return 2;
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    const int fd = serial_port_open(port);
    if (fd < 0) {
        std::fprintf(stderr, "Failed to open %s\n", port);
        return 1;
    }
    if (serial_port_configure(fd, baud) < 0) {
        std::fprintf(stderr, "Failed to configure baud %d\n", baud);
        serial_port_close(fd);
        return 1;
    }

    char recv_buf[1024];
    serial_send_then_recv_str(fd, "LOG ENABLE\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);

    hipnuc_raw_t raw{};
    uint8_t read_buf[1024];
    Sample last_sample;
    uint64_t frames = 0;
    uint64_t frames_at_rate_mark = 0;
    float hz = 0.0f;
    auto last_print = std::chrono::steady_clock::now();
    auto last_rate = last_print;

    while (g_running.load()) {
        const int n = serial_port_read(fd, reinterpret_cast<char*>(read_buf), sizeof(read_buf));
        if (n > 0) {
            for (int i = 0; i < n; ++i) {
                if (hipnuc_input(&raw, read_buf[i]) > 0) {
                    Sample s = Extract(raw);
                    if (s.ok) {
                        last_sample = s;
                        ++frames;
                    }
                }
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const float rate_dt = std::chrono::duration<float>(now - last_rate).count();
        if (rate_dt >= 1.0f) {
            hz = static_cast<float>(frames - frames_at_rate_mark) / rate_dt;
            frames_at_rate_mark = frames;
            last_rate = now;
        }

        const float print_dt = std::chrono::duration<float>(now - last_print).count();
        if (print_dt >= 0.10f) {
            if (last_sample.ok) {
                PrintLive(last_sample, map, rpy_map, use_rpy_component_map, frames, hz);
            } else {
                std::printf("\033[2J\033[HWaiting for HiPNUC 0x91/0x83 frames on %s @ %d...\n",
                            port, baud);
                std::fflush(stdout);
            }
            last_print = now;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::printf("\033[0m\n");
    serial_port_close(fd);
    return 0;
}
