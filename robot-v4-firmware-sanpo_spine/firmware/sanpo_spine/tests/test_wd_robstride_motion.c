#include "wd_robstride_motion.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static uint16_t read_be_u16(const uint8_t *input) {
  return (uint16_t)(((uint16_t)input[0] << 8) | (uint16_t)input[1]);
}

static float u16_to_float(uint16_t raw, float minimum, float maximum) {
  return minimum + ((float)raw * (maximum - minimum) / 65535.0f);
}

static void test_training_aligned_wheel_command(void) {
  uint32_t can_id = 0u;
  uint8_t payload[8] = {0};
  static const uint8_t expected_payload[8] = {
      0x7Fu, 0xFFu, 0x68u, 0xB9u, 0x00u, 0x00u, 0x14u, 0x7Au};
  const uint16_t torque_raw_expected = (uint16_t)(0.5f * 65535.0f);
  float decoded_velocity;
  float decoded_kd;
  uint32_t byte_index;

  assert(wd_robstride_build_motion_command(
             4u, 0.0f, 0.0f, -8.0f, 0.0f, 0.4f,
             &can_id, payload) != 0);
  assert((can_id >> 24) == WD_ROBSTRIDE_MOTION_COMM_TYPE);
  assert((can_id & 0xFFu) == 4u);
  assert(((can_id >> 8) & 0xFFFFu) == torque_raw_expected);
  assert(can_id == 0x017FFF04u);
  for (byte_index = 0u; byte_index < 8u; ++byte_index) {
    assert(payload[byte_index] == expected_payload[byte_index]);
  }
  assert(read_be_u16(&payload[4]) == 0u);

  decoded_velocity = u16_to_float(
      read_be_u16(&payload[2]),
      -WD_RS01_MOTION_VELOCITY_MAX_RADPS,
      WD_RS01_MOTION_VELOCITY_MAX_RADPS);
  decoded_kd = u16_to_float(
      read_be_u16(&payload[6]), 0.0f, WD_RS01_MOTION_KD_MAX);
  assert(fabsf(decoded_velocity - (-8.0f)) < 0.002f);
  assert(fabsf(decoded_kd - 0.4f) < 0.0001f);
}

static void test_zero_torque_stop_command(void) {
  uint32_t can_id = 0u;
  uint8_t payload[8] = {0};

  assert(wd_robstride_build_motion_command(
             1u, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
             &can_id, payload) != 0);
  assert(read_be_u16(&payload[4]) == 0u);
  assert(read_be_u16(&payload[6]) == 0u);
}

static void test_documented_range_clamps_and_rejections(void) {
  uint32_t can_id = 0u;
  uint8_t payload[8] = {0};

  assert(wd_robstride_build_motion_command(
             2u, 100.0f, 100.0f, 100.0f, 1000.0f, 100.0f,
             &can_id, payload) != 0);
  assert(((can_id >> 8) & 0xFFFFu) == 65535u);
  assert(read_be_u16(&payload[0]) == 65535u);
  assert(read_be_u16(&payload[2]) == 65535u);
  assert(read_be_u16(&payload[4]) == 65535u);
  assert(read_be_u16(&payload[6]) == 65535u);

  assert(wd_robstride_build_motion_command(
             0u, 0.0f, 0.0f, 0.0f, 0.0f, 0.4f,
             &can_id, payload) == 0);
  assert(wd_robstride_build_motion_command(
             1u, NAN, 0.0f, 0.0f, 0.0f, 0.4f,
             &can_id, payload) == 0);
  assert(wd_robstride_build_motion_command(
             1u, 0.0f, 0.0f, 0.0f, 0.0f, 0.4f,
             NULL, payload) == 0);
}

int main(void) {
  test_training_aligned_wheel_command();
  test_zero_torque_stop_command();
  test_documented_range_clamps_and_rejections();
  puts("wd_robstride_motion: all tests passed");
  return 0;
}
