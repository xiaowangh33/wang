#include "wd_safety.h"
#include "wd_motor_calibration.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int nearly_equal(float lhs, float rhs) {
  return fabsf(lhs - rhs) < 1.0e-6f;
}

static void test_deployment_limits(void) {
  WdSafetyRuntimeConfig runtime;
  uint32_t index;

  wd_safety_default_runtime_config(&runtime);
  assert(nearly_equal(runtime.torque_limit_nm, 36.0f));
  assert(nearly_equal(runtime.velocity_limit_radps, 44.0f));
  assert(nearly_equal(runtime.torque_slew_rate_nm_per_s,
                      WD_ACTIVE_TORQUE_SLEW_RATE_NM_PER_S));

  for (index = 0u; index < WD_MOTOR_COUNT; ++index) {
    const float expected_torque_limit = ((index % 4u) == 3u) ? 17.0f : 36.0f;
    const float expected_velocity_limit =
        ((index % 4u) == 3u) ? 44.0f : 8.0f;
    assert(nearly_equal(
        wd_safety_effective_torque_limit(index, &runtime),
        expected_torque_limit));
    assert(nearly_equal(
        wd_safety_effective_velocity_limit(index, &runtime),
        expected_velocity_limit));
  }

  assert(wd_safety_velocity_is_overspeed(0u, 8.0f, &runtime) == 0);
  assert(wd_safety_velocity_is_overspeed(0u, 9.99f, &runtime) == 0);
  assert(wd_safety_velocity_is_overspeed(0u, 10.0f, &runtime) != 0);
  assert(wd_safety_velocity_is_overspeed(3u, 8.0f, &runtime) == 0);
  assert(wd_safety_velocity_is_overspeed(3u, -9.99f, &runtime) == 0);
  assert(wd_safety_velocity_is_overspeed(3u, -100.0f, &runtime) == 0);

  {
    uint8_t wheel_consecutive = 2u;
    assert(wd_safety_update_overspeed_trip(
               3u, 100.0f, &runtime, &wheel_consecutive) == 0);
    assert(wheel_consecutive == 0u);
  }

  {
    uint8_t consecutive = 0u;
    assert(wd_safety_update_overspeed_trip(
               0u, 10.05f, &runtime, &consecutive) == 0);
    assert(consecutive == 1u);
    assert(wd_safety_update_overspeed_trip(
               0u, 10.10f, &runtime, &consecutive) == 0);
    assert(consecutive == 2u);
    assert(wd_safety_update_overspeed_trip(
               0u, 10.20f, &runtime, &consecutive) != 0);
    assert(consecutive == WD_OVERSPEED_CONSECUTIVE_SAMPLES);
    assert(wd_safety_update_overspeed_trip(
               0u, 9.99f, &runtime, &consecutive) == 0);
    assert(consecutive == 0u);
    assert(wd_safety_update_overspeed_trip(
               0u, -16.0f, &runtime, &consecutive) != 0);
  }

  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          0u, 0.9f, 2.0f, &runtime),
                      2.0f));
  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          0u, 1.5f, 2.0f, &runtime),
                      2.0f));
  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          0u, 7.99f, 2.0f, &runtime),
                      2.0f));
  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          0u, 8.0f, 2.0f, &runtime),
                      2.0f));
  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          0u, 9.0f, 2.0f, &runtime),
                      1.0f));
  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          0u, 10.0f, 2.0f, &runtime),
                      0.0f));
  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          0u, -9.0f, -2.0f, &runtime),
                      -1.0f));
  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          0u, -9.8f, 2.0f, &runtime),
                      2.0f));
  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          3u, 100.0f, 2.0f, &runtime),
                      2.0f));
  assert(nearly_equal(wd_safety_derate_torque_for_velocity(
                          3u, -100.0f, -2.0f, &runtime),
                      -2.0f));
}

static void test_final_command_clamps(void) {
  WdSafetyRuntimeConfig runtime;
  WdJointCommand command = {0};
  uint32_t flags = 0u;
  float output;

  wd_safety_default_runtime_config(&runtime);

  output = wd_safety_limit_torque_command(
      0u, 40.0f, 0.0f, 0.0f, &runtime, &flags);
  assert(nearly_equal(output, 36.0f));
  assert((flags & WD_STATUS_COMMAND_CLIPPED) != 0u);

  flags = 0u;
  output = wd_safety_limit_velocity_command(3u, 8.0f, &runtime, &flags);
  assert(nearly_equal(output, 8.0f));
  assert((flags & WD_STATUS_COMMAND_CLIPPED) == 0u);

  flags = 0u;
  /* A wheel dq_des is the virtual target in Kd*(dq_des-dq), so it uses the
   * full RS01 +/-44 rad/s motion-protocol range without a separate measured
   * wheel-speed guard. */
  output = wd_safety_limit_velocity_command(3u, -20.0f, &runtime, &flags);
  assert(nearly_equal(output, -20.0f));
  assert((flags & WD_STATUS_COMMAND_CLIPPED) == 0u);

  flags = 0u;
  output = wd_safety_limit_velocity_command(3u, 50.0f, &runtime, &flags);
  assert(nearly_equal(output, 44.0f));
  assert((flags & WD_STATUS_COMMAND_CLIPPED) != 0u);

  /* At zero measured speed Kd=0.4 and a full virtual target request expose
   * the complete 17 Nm peak envelope: 0 + 17/0.4 = 42.5 rad/s. */
  {
    float torque = 0.0f;
    flags = 0u;
    output = wd_safety_compute_velocity_motion_target(
        3u, 44.0f, 0.0f, 0.4f, 0.0f, 0.0f,
        &runtime, &flags, &torque);
    assert(nearly_equal(output, 42.5f));
    assert(nearly_equal(torque, 17.0f));

    /* Wheel measured-speed derating is disabled. At 9 rad/s the full virtual
     * target remains available and produces 14 Nm. */
    flags = 0u;
    output = wd_safety_compute_velocity_motion_target(
        3u, 44.0f, 9.0f, 0.4f, 17.0f, 0.002f,
        &runtime, &flags, &torque);
    assert(nearly_equal(output, 44.0f));
    assert(nearly_equal(torque, 14.0f));
    assert((flags & WD_STATUS_VELOCITY_GUARD) == 0u);

    flags = 0u;
    output = wd_safety_compute_velocity_motion_target(
        3u, -44.0f, 9.0f, 0.4f, 7.0f, 0.002f,
        &runtime, &flags, &torque);
    assert(nearly_equal(output, -33.5f));
    assert(nearly_equal(torque, -17.0f));
    assert((flags & WD_STATUS_VELOCITY_GUARD) == 0u);

    /* Even if external motion drives a wheel beyond the protocol target
     * range, the reported final command remains bounded by the native 17 Nm
     * drive limit while no measured-speed trip is generated. */
    flags = 0u;
    output = wd_safety_compute_velocity_motion_target(
        3u, 44.0f, 100.0f, 0.4f, 0.0f, 0.002f,
        &runtime, &flags, &torque);
    assert(nearly_equal(output, 44.0f));
    assert(nearly_equal(torque, -17.0f));
    assert((flags & WD_STATUS_VELOCITY_GUARD) == 0u);
  }

  command.kp = 65.0f;
  command.q_des = 1.0f;
  command.kd = 1.0f;
  output = wd_safety_compute_pd_torque(
      0u, 0.0f, 0.0f, &command, WD_LIMIT_CALIBRATED_ABSOLUTE, &runtime);
  assert(nearly_equal(output, 36.0f));

  memset(&command, 0, sizeof(command));
  command.q_des = 1.0f;
  command.dq_des = 2.0f;
  assert(wd_safety_position_command_is_passive(&command) != 0);
  command.kp = 1.0f;
  assert(wd_safety_position_command_is_passive(&command) == 0);
  command.kp = 0.0f;
  command.kd = 1.0f;
  assert(wd_safety_position_command_is_passive(&command) == 0);
  command.kd = 0.0f;
  command.tau_ff = 0.1f;
  assert(wd_safety_position_command_is_passive(&command) == 0);
  assert(wd_safety_position_command_is_passive(NULL) == 0);

  /* A folded front knee starts inside the 5-degree soft zone but within the
   * corrected +/-2.65 rad hard range. Matching/inward StandUp commands must
   * not receive an artificial wall impulse; outward torque is blocked. */
  memset(&command, 0, sizeof(command));
  command.kp = 65.0f;
  command.kd = 2.0f;
  command.q_des = -2.596f;
  command.dq_des = 0.04f;
  output = wd_safety_compute_pd_torque(
      6u, -2.596f, 0.04f, &command,
      WD_LIMIT_CALIBRATED_ABSOLUTE, &runtime);
  assert(nearly_equal(output, 0.0f));

  command.q_des = -2.65f;
  command.dq_des = 0.0f;
  output = wd_safety_compute_pd_torque(
      6u, -2.596f, 0.0f, &command,
      WD_LIMIT_CALIBRATED_ABSOLUTE, &runtime);
  assert(nearly_equal(output, 0.0f));

  command.q_des = -1.0f;
  output = wd_safety_compute_pd_torque(
      6u, -2.596f, 0.0f, &command,
      WD_LIMIT_CALIBRATED_ABSOLUTE, &runtime);
  assert(nearly_equal(output, 36.0f));
}

static void test_leg_has_no_active_torque_slew(void) {
  WdSafetyRuntimeConfig runtime;
  uint32_t flags = 0u;
  float output;

  wd_safety_default_runtime_config(&runtime);

  /* Normal position-PD torque growth and reversal are immediate up to the
   * unchanged hard actuator ceiling. */
  output = wd_safety_limit_torque_command_motion_aware(
      0u, 36.0f, 0.0f, 1.0f, 0.002f, &runtime, &flags);
  assert(nearly_equal(output, 36.0f));
  assert((flags & WD_STATUS_TORQUE_SLEW_LIMITED) == 0u);

  flags = 0u;
  output = wd_safety_limit_torque_command_motion_aware(
      0u, -36.0f, 36.0f, 0.0f, 0.002f, &runtime, &flags);
  assert(nearly_equal(output, -36.0f));
  assert((flags & WD_STATUS_TORQUE_SLEW_LIMITED) == 0u);

  /* The dedicated qualification path deliberately calls the basic limiter,
   * so its 50 Nm/s excitation ramp remains available. */
  runtime.torque_slew_rate_nm_per_s = WD_ENABLE_TORQUE_SLEW_RATE_NM_PER_S;
  flags = 0u;
  output = wd_safety_limit_torque_command(
      0u, 0.6f, 0.0f, 0.001f, &runtime, &flags);
  assert(nearly_equal(output, 0.05f));
  assert((flags & WD_STATUS_TORQUE_SLEW_LIMITED) != 0u);
}

static void test_wheel_has_no_active_torque_slew(void) {
  WdSafetyRuntimeConfig runtime;
  uint32_t flags = 0u;
  float output;

  wd_safety_default_runtime_config(&runtime);

  /* RS01 same-direction torque growth is immediate up to the 17 Nm ceiling. */
  output = wd_safety_limit_torque_command_motion_aware(
      3u, 17.0f, 0.0f, 1.0f, 0.002f, &runtime, &flags);
  assert(nearly_equal(output, 17.0f));
  assert((flags & WD_STATUS_TORQUE_SLEW_LIMITED) == 0u);

  /* At meaningful positive speed, negative wheel torque is braking and may
   * reverse immediately up to the 17 Nm RS01 ceiling. */
  flags = 0u;
  output = wd_safety_limit_torque_command_motion_aware(
      3u, -17.0f, 17.0f, 11.5f, 0.002f, &runtime, &flags);
  assert(nearly_equal(output, -17.0f));
  assert((flags & WD_STATUS_TORQUE_SLEW_LIMITED) == 0u);

  /* Near zero speed a wheel sign reversal is also immediate. */
  flags = 0u;
  output = wd_safety_limit_torque_command_motion_aware(
      3u, -17.0f, 17.0f, 0.0f, 0.002f, &runtime, &flags);
  assert(nearly_equal(output, -17.0f));
  assert((flags & WD_STATUS_TORQUE_SLEW_LIMITED) == 0u);
}

static void test_absolute_calibration_table(void) {
  static const char *const expected_symbols[WD_MOTOR_COUNT] = {
      "FL_HipX", "FL_HipY", "FL_Knee", "FL_Ankle",
      "FR_HipX", "FR_HipY", "FR_Knee", "FR_Ankle",
      "HL_HipX", "HL_HipY", "HL_Knee", "HL_Ankle",
      "HR_HipX", "HR_HipY", "HR_Knee", "HR_Ankle"};
  static const float expected_sign[WD_MOTOR_COUNT] = {
      -1.0f, -1.0f, 1.0f, -1.0f,
      -1.0f, 1.0f, -1.0f, 1.0f,
      1.0f, -1.0f, 1.0f, -1.0f,
      1.0f, 1.0f, -1.0f, 1.0f};
  static const float expected_zero[WD_MOTOR_COUNT] = {
      2.199250f, 5.717500f, -0.123200f, 0.0f,
      2.698700f, 1.736150f, 2.875100f, 0.0f,
      2.380600f, 2.246850f, 5.830000f, 0.0f,
      5.340450f, 2.294450f, -2.326500f, 0.0f};
  static const float expected_raw_lower[WD_MOTOR_COUNT] = {
      1.407500f, 3.102000f, -2.773200f, 0.0f,
      1.910000f, -0.881200f, 0.225100f, 0.0f,
      1.588700f, -0.367500f, 3.180000f, 0.0f,
      4.555400f, -0.323500f, -4.976500f, 0.0f};
  static const float expected_raw_upper[WD_MOTOR_COUNT] = {
      2.991000f, 8.333000f, 2.526800f, 0.0f,
      3.487400f, 4.353500f, 5.525100f, 0.0f,
      3.172500f, 4.861200f, 8.480000f, 0.0f,
      6.125500f, 4.912400f, 0.323500f, 0.0f};
  const WdMotorCalibration *fl_hipx;
  const WdMotorCalibration *fl_hipy;
  const WdMotorCalibration *hr_knee;
  const WdMotorCalibration *wheel;
  const float two_pi = WD_MOTOR_RAW_WRAP_PERIOD_RAD;
  uint32_t index;

  assert(wd_motor_calibration_validate_all() != 0);
  for (index = 0u; index < WD_MOTOR_COUNT; ++index) {
    const WdMotorCalibration *entry = wd_motor_calibration(index);
    assert(entry != NULL);
    assert(strcmp(entry->symbol, expected_symbols[index]) == 0);
    assert(entry->can_number == (index / 4u) + 1u);
    assert(entry->motor_id == (index % 4u) + 1u);
    assert(nearly_equal(entry->direction_sign, expected_sign[index]));
    assert(nearly_equal(entry->raw_zero_rad, expected_zero[index]));
    assert(nearly_equal(entry->raw_hard_lower_rad,
                        expected_raw_lower[index]));
    assert(nearly_equal(entry->raw_hard_upper_rad,
                        expected_raw_upper[index]));
    if ((index % 4u) == 3u) {
      assert(entry->mode == WD_ACTUATOR_VELOCITY);
      assert(entry->raw_position_limit_enabled == 0u);
    } else {
      const float q_at_raw_lower = wd_motor_raw_to_joint(
          index, entry->raw_hard_lower_rad);
      const float q_at_raw_upper = wd_motor_raw_to_joint(
          index, entry->raw_hard_upper_rad);
      assert(entry->mode == WD_ACTUATOR_POSITION_PD);
      assert(entry->raw_position_limit_enabled != 0u);
      assert(nearly_equal(wd_motor_raw_to_joint(index, entry->raw_zero_rad),
                          0.0f));
      if (entry->direction_sign > 0.0f) {
        assert(fabsf(q_at_raw_lower - entry->joint_lower_rad) < 0.01f);
        assert(fabsf(q_at_raw_upper - entry->joint_upper_rad) < 0.01f);
      } else {
        assert(fabsf(q_at_raw_lower - entry->joint_upper_rad) < 0.01f);
        assert(fabsf(q_at_raw_upper - entry->joint_lower_rad) < 0.01f);
      }
    }
  }
  fl_hipx = wd_motor_calibration(0u);
  fl_hipy = wd_motor_calibration(1u);
  hr_knee = wd_motor_calibration(14u);
  wheel = wd_motor_calibration(15u);
  assert(fl_hipx != NULL && fl_hipy != NULL && hr_knee != NULL && wheel != NULL);
  assert(fl_hipx->can_number == 1u && fl_hipx->motor_id == 1u);
  assert(nearly_equal(fl_hipx->direction_sign, -1.0f));
  assert(nearly_equal(fl_hipx->raw_zero_rad, 2.199250f));
  assert(nearly_equal(fl_hipy->raw_hard_upper_rad, 8.333000f));
  assert(nearly_equal(hr_knee->raw_zero_rad, -2.326500f));
  assert(nearly_equal(wd_motor_calibration(6u)->direction_sign, -1.0f));
  assert(nearly_equal(hr_knee->direction_sign, -1.0f));
  assert(nearly_equal(wd_motor_calibration(3u)->direction_sign, -1.0f));
  assert(nearly_equal(wd_motor_calibration(7u)->direction_sign, 1.0f));
  assert(nearly_equal(wd_motor_calibration(11u)->direction_sign, -1.0f));
  assert(nearly_equal(wd_motor_calibration(15u)->direction_sign, 1.0f));
  assert(wheel->mode == WD_ACTUATOR_VELOCITY);
  assert(wheel->raw_position_limit_enabled == 0u);

  /* The same FL_HipY mechanical stop may be reported one 2*pi turn lower. */
  assert(fabsf(wd_motor_align_raw(1u, 8.333000f - two_pi) - 8.333000f) <
         1.0e-5f);
  assert(fabsf(wd_motor_raw_to_joint(1u, 8.333000f) + 2.615500f) <
         1.0e-5f);
  assert(fabsf(wd_motor_raw_to_joint(1u, 3.102000f) - 2.615500f) <
         1.0e-5f);
  assert(nearly_equal(wd_motor_raw_to_joint(6u, 2.875100f - 0.25f),
                      0.25f));
  assert(nearly_equal(wd_motor_raw_to_joint(14u, -2.326500f + 0.25f),
                      -0.25f));
  assert(nearly_equal(wd_motor_raw_to_joint(14u, -2.326500f), 0.0f));
}

static void test_setpoint_is_forced_to_absolute_limits(void) {
  WdSafetyRuntimeConfig runtime;
  WdSetpointPayload setpoint = {0};
  uint32_t flags;
  uint32_t index;

  wd_safety_default_runtime_config(&runtime);
  setpoint.limit_mode = (uint8_t)WD_LIMIT_STARTUP_RELATIVE;
  for (index = 0u; index < WD_MOTOR_COUNT; ++index) {
    const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
    setpoint.actuator_mode[index] = (uint8_t)joint->mode;
  }
  setpoint.joint[0].q_des = 10.0f;
  flags = wd_safety_sanitize_setpoint(&setpoint, &runtime);
  assert(setpoint.limit_mode == (uint8_t)WD_LIMIT_CALIBRATED_ABSOLUTE);
  assert(nearly_equal(setpoint.joint[0].q_des, 0.7854f));
  assert((flags & WD_STATUS_BAD_PACKET) != 0u);
  assert((flags & WD_STATUS_SETPOINT_CLIPPED) != 0u);
  assert((flags & WD_STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED) == 0u);

  memset(&setpoint, 0, sizeof(setpoint));
  setpoint.limit_mode = (uint8_t)WD_LIMIT_CALIBRATED_ABSOLUTE;
  for (index = 0u; index < WD_MOTOR_COUNT; ++index) {
    const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
    setpoint.actuator_mode[index] = (uint8_t)joint->mode;
  }
  setpoint.joint[2].q_des = -2.60f;
  flags = wd_safety_sanitize_setpoint(&setpoint, &runtime);
  assert(nearly_equal(setpoint.joint[2].q_des, -2.60f));
  assert((flags & WD_STATUS_SETPOINT_CLIPPED) == 0u);

  setpoint.joint[2].q_des = -2.70f;
  flags = wd_safety_sanitize_setpoint(&setpoint, &runtime);
  assert(nearly_equal(setpoint.joint[2].q_des, -2.65f));
  assert((flags & WD_STATUS_SETPOINT_CLIPPED) != 0u);

  /* Wheel velocity actions retain the training-aligned motion-mode damping.
   * Position stiffness/feedforward remain forbidden, and the motor protocol's
   * documented Kd ceiling is enforced by the MCU. */
  memset(&setpoint, 0, sizeof(setpoint));
  setpoint.limit_mode = (uint8_t)WD_LIMIT_CALIBRATED_ABSOLUTE;
  for (index = 0u; index < WD_MOTOR_COUNT; ++index) {
    const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
    setpoint.actuator_mode[index] = (uint8_t)joint->mode;
  }
  setpoint.joint[3].kd = 0.4f;
  setpoint.joint[3].dq_des = 2.0f;
  flags = wd_safety_sanitize_setpoint(&setpoint, &runtime);
  assert(nearly_equal(setpoint.joint[3].kd, 0.4f));
  assert(nearly_equal(setpoint.joint[3].dq_des, 2.0f));
  assert((flags & WD_STATUS_SETPOINT_CLIPPED) == 0u);

  setpoint.joint[3].dq_des = 50.0f;
  flags = wd_safety_sanitize_setpoint(&setpoint, &runtime);
  assert(nearly_equal(setpoint.joint[3].dq_des, 44.0f));
  assert((flags & WD_STATUS_SETPOINT_CLIPPED) != 0u);

  setpoint.joint[3].kp = 1.0f;
  setpoint.joint[3].q_des = 2.0f;
  setpoint.joint[3].kd = 6.0f;
  setpoint.joint[3].tau_ff = 1.0f;
  flags = wd_safety_sanitize_setpoint(&setpoint, &runtime);
  assert(nearly_equal(setpoint.joint[3].kp, 0.0f));
  assert(nearly_equal(setpoint.joint[3].q_des, 0.0f));
  assert(nearly_equal(setpoint.joint[3].kd, 5.0f));
  assert(nearly_equal(setpoint.joint[3].tau_ff, 0.0f));
  assert((flags & WD_STATUS_SETPOINT_CLIPPED) != 0u);
}

static void test_raw_boundary_blocks_only_outward_torque(void) {
  WdSafetyRuntimeConfig runtime;
  uint32_t flags = 0u;
  float output;

  wd_safety_default_runtime_config(&runtime);

  /* FL_HipX sign is negative. Numeric raw lower is joint upper: positive
   * joint torque is outward and must be canceled. */
  output = wd_safety_enforce_raw_position_limit(
      0u, 1.407500f, 0.0f, 2.0f, &runtime, &flags);
  assert(nearly_equal(output, 0.0f));
  assert((flags & WD_STATUS_RAW_POSITION_LIMIT) != 0u);
  assert((flags & WD_STATUS_COMMAND_CLIPPED) != 0u);

  /* Planned inward torque passes through unchanged; the raw wall does not
   * synthesize a full-limit restoring impulse. */
  flags = 0u;
  output = wd_safety_enforce_raw_position_limit(
      0u, 1.407500f, 0.0f, -2.0f, &runtime, &flags);
  assert(nearly_equal(output, -2.0f));
  assert((flags & WD_STATUS_RAW_POSITION_LIMIT) != 0u);
  assert((flags & WD_STATUS_COMMAND_CLIPPED) == 0u);

  flags = 0u;
  output = wd_safety_enforce_raw_position_limit(
      0u, 2.991000f, 0.0f, -2.0f, &runtime, &flags);
  assert(nearly_equal(output, 0.0f));
  assert((flags & WD_STATUS_RAW_POSITION_LIMIT) != 0u);

  flags = 0u;
  output = wd_safety_enforce_raw_position_limit(
      0u, 2.991000f, 0.0f, 2.0f, &runtime, &flags);
  assert(nearly_equal(output, 2.0f));
  assert((flags & WD_STATUS_RAW_POSITION_LIMIT) != 0u);

  flags = 0u;
  output = wd_safety_enforce_raw_position_limit(
      0u, 2.199250f, 0.0f, 0.75f, &runtime, &flags);
  assert(nearly_equal(output, 0.75f));
  assert((flags & WD_STATUS_RAW_POSITION_LIMIT) == 0u);

  /* A raw sample on an adjacent single-turn branch maps to the same wall. */
  flags = 0u;
  output = wd_safety_enforce_raw_position_limit(
      0u,
      wd_motor_align_raw(0u,
                         1.407500f - WD_MOTOR_RAW_WRAP_PERIOD_RAD),
      0.0f,
      2.0f,
      &runtime,
      &flags);
  assert(nearly_equal(output, 0.0f));
  assert((flags & WD_STATUS_RAW_POSITION_LIMIT) != 0u);

  /* Continuous wheel phase is never subject to a position wall. */
  output = wd_safety_enforce_raw_position_limit(
      3u, 1000.0f, 20.0f, 0.5f, &runtime, &flags);
  assert(nearly_equal(output, 0.5f));
}

int main(void) {
  test_deployment_limits();
  test_final_command_clamps();
  test_leg_has_no_active_torque_slew();
  test_wheel_has_no_active_torque_slew();
  test_absolute_calibration_table();
  test_setpoint_is_forced_to_absolute_limits();
  test_raw_boundary_blocks_only_outward_torque();
  puts("wd_safety: all tests passed");
  return 0;
}
