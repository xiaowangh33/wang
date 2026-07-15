#include "wd_robstride_motion.h"

#include <math.h>
#include <stddef.h>

static float wd_robstride_clamp(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

static uint16_t wd_robstride_float_to_u16(float value,
                                          float minimum,
                                          float maximum) {
  const float clamped = wd_robstride_clamp(value, minimum, maximum);
  const float scaled =
      (clamped - minimum) * 65535.0f / (maximum - minimum);
  return (uint16_t)scaled;
}

static void wd_robstride_write_be_u16(uint8_t *output, uint16_t value) {
  output[0] = (uint8_t)((value >> 8) & 0xFFu);
  output[1] = (uint8_t)(value & 0xFFu);
}

int wd_robstride_build_motion_command(uint8_t motor_id,
                                      float torque_feedforward_nm,
                                      float position_desired_rad,
                                      float velocity_desired_radps,
                                      float kp,
                                      float kd,
                                      uint32_t *extended_can_id,
                                      uint8_t payload[8]) {
  uint16_t torque_raw;

  if (motor_id == 0u || motor_id > 0x7Fu || extended_can_id == NULL ||
      payload == NULL || !isfinite(torque_feedforward_nm) ||
      !isfinite(position_desired_rad) ||
      !isfinite(velocity_desired_radps) || !isfinite(kp) || !isfinite(kd)) {
    return 0;
  }

  torque_raw = wd_robstride_float_to_u16(
      torque_feedforward_nm,
      -WD_RS01_MOTION_TORQUE_MAX_NM,
      WD_RS01_MOTION_TORQUE_MAX_NM);
  *extended_can_id =
      ((uint32_t)WD_ROBSTRIDE_MOTION_COMM_TYPE << 24) |
      ((uint32_t)torque_raw << 8) |
      (uint32_t)motor_id;

  wd_robstride_write_be_u16(
      &payload[0],
      wd_robstride_float_to_u16(position_desired_rad,
                                -WD_RS01_MOTION_POSITION_MAX_RAD,
                                WD_RS01_MOTION_POSITION_MAX_RAD));
  wd_robstride_write_be_u16(
      &payload[2],
      wd_robstride_float_to_u16(velocity_desired_radps,
                                -WD_RS01_MOTION_VELOCITY_MAX_RADPS,
                                WD_RS01_MOTION_VELOCITY_MAX_RADPS));
  wd_robstride_write_be_u16(
      &payload[4],
      wd_robstride_float_to_u16(kp, 0.0f, WD_RS01_MOTION_KP_MAX));
  wd_robstride_write_be_u16(
      &payload[6],
      wd_robstride_float_to_u16(kd, 0.0f, WD_RS01_MOTION_KD_MAX));
  return 1;
}
