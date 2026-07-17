#include "wd_safety.h"

#include "wd_motor_calibration.h"
#include "wd_robstride_motion.h"

#include <math.h>
#include <string.h>

#define WD_PI (3.14159265358979323846f)
#define WD_DEFAULT_SOFT_ZONE_RAD (5.0f * WD_PI / 180.0f)
#define WD_DEPLOYMENT_LEG_TORQUE_LIMIT_NM (36.0f)
/* Literal leg velocity commands retain their 8 rad/s safety ceiling. Wheel
 * dq_des is the virtual target in tau=Kd*(dq_des-dq); deployment caps that
 * target at +/-20 rad/s while the native RS01 CAN codec remains +/-44 rad/s. */
#define WD_BRINGUP_WHEEL_VELOCITY_LIMIT_RADPS (20.0f)
#define WD_RUNTIME_VELOCITY_LIMIT_RADPS WD_BRINGUP_WHEEL_VELOCITY_LIMIT_RADPS
#define WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS (8.0f)
#define WD_WHEEL_MOTION_VIRTUAL_VELOCITY_LIMIT_RADPS (20.0f)
/* The measured-speed derate/trip remains a leg-joint protection only. Wheel
 * motion no longer enters this 8..10 rad/s guard or its 16 rad/s immediate
 * trip. The requested target is capped at +/-20 rad/s and the drive's own
 * hardware protections remain active. */
#define WD_LEG_DERATE_START_RADPS (8.0f)
#define WD_LEG_OVERSPEED_TRIP_RADPS (10.0f)
#define WD_LEG_OVERSPEED_IMMEDIATE_TRIP_RADPS (16.0f)
/* RS01 motion-control mode uses this as its motor-side limit_torque value.
 * The user authorized the actuator's complete rated 17 Nm peak envelope;
 * continuous thermal loading is still the drive's own responsibility. */
#define WD_WHEEL_MOTION_TORQUE_LIMIT_NM (17.0f)

static const WdJointSafetyConfig kJointSafety[WD_MOTOR_COUNT] = {
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_VELOCITY, 17.0f, WD_BRINGUP_WHEEL_VELOCITY_LIMIT_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_VELOCITY, 17.0f, WD_BRINGUP_WHEEL_VELOCITY_LIMIT_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_VELOCITY, 17.0f, WD_BRINGUP_WHEEL_VELOCITY_LIMIT_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_POSITION_PD, 36.0f, WD_BRINGUP_LEG_VELOCITY_GUARD_RADPS},
    {WD_ACTUATOR_VELOCITY, 17.0f, WD_BRINGUP_WHEEL_VELOCITY_LIMIT_RADPS},
};

void wd_safety_default_runtime_config(WdSafetyRuntimeConfig *config) {
  if (config == NULL) {
    return;
  }
  config->torque_limit_nm = WD_DEPLOYMENT_LEG_TORQUE_LIMIT_NM;
  config->velocity_limit_radps = WD_RUNTIME_VELOCITY_LIMIT_RADPS;
  config->torque_slew_rate_nm_per_s =
      WD_ACTIVE_TORQUE_SLEW_RATE_NM_PER_S;
  config->limit_soft_zone_rad = WD_DEFAULT_SOFT_ZONE_RAD;
}

const WdJointSafetyConfig *wd_safety_joint_config(uint32_t index) {
  if (index >= WD_MOTOR_COUNT) {
    return NULL;
  }
  return &kJointSafety[index];
}

void wd_safety_default_modes(uint8_t modes[WD_MOTOR_COUNT]) {
  uint32_t i;
  if (modes == NULL) {
    return;
  }
  for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
    modes[i] = (uint8_t)kJointSafety[i].mode;
  }
}

float wd_safety_clamp(float value, float low, float high) {
  if (low > high) {
    return value;
  }
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

int wd_safety_is_finite(float value) {
  return isfinite(value) ? 1 : 0;
}

static float finite_or_zero(float value) {
  return wd_safety_is_finite(value) ? value : 0.0f;
}

static void add_status(uint32_t *flags, uint32_t status) {
  if (flags != NULL) {
    *flags |= status;
  }
}

static float finite_or_zero_with_status(float value, uint32_t *flags) {
  if (wd_safety_is_finite(value) != 0) {
    return value;
  }
  add_status(flags, WD_STATUS_BAD_PACKET | WD_STATUS_SETPOINT_CLIPPED);
  return 0.0f;
}

static float clamp_with_status(float value,
                               float low,
                               float high,
                               uint32_t *flags) {
  float clamped = wd_safety_clamp(value, low, high);
  if (clamped != value) {
    add_status(flags, WD_STATUS_SETPOINT_CLIPPED);
  }
  return clamped;
}

static float finite_command_or_zero(float value, uint32_t *flags) {
  if (wd_safety_is_finite(value) != 0) {
    return value;
  }
  add_status(flags, WD_STATUS_BAD_PACKET | WD_STATUS_COMMAND_CLIPPED);
  return 0.0f;
}

static float clamp_command_with_status(float value,
                                       float low,
                                       float high,
                                       uint32_t *flags) {
  float clamped = wd_safety_clamp(value, low, high);
  if (clamped != value) {
    add_status(flags, WD_STATUS_COMMAND_CLIPPED);
  }
  return clamped;
}

static float joint_velocity_limit(const WdJointSafetyConfig *joint,
                                  const WdSafetyRuntimeConfig *runtime) {
  float limit = joint->velocity_limit_radps;
  if (runtime != NULL && runtime->velocity_limit_radps > 0.0f) {
    limit = fminf(limit, runtime->velocity_limit_radps);
  }
  if (limit < 0.0f) {
    limit = 0.0f;
  }
  return limit;
}

/* Wheel dq_des is a virtual motion-control target capped at the deployment
 * limit; continuous wheels are excluded from the leg-only measured guard. */
static float joint_velocity_command_limit(
    const WdJointSafetyConfig *joint,
    const WdSafetyRuntimeConfig *runtime) {
  if (joint != NULL && joint->mode == WD_ACTUATOR_VELOCITY) {
    return WD_WHEEL_MOTION_VIRTUAL_VELOCITY_LIMIT_RADPS;
  }
  return joint_velocity_limit(joint, runtime);
}

static float joint_torque_limit(const WdJointSafetyConfig *joint,
                                const WdSafetyRuntimeConfig *runtime) {
  float limit = joint->effort_limit_nm;
  if (joint->mode == WD_ACTUATOR_VELOCITY) {
    limit = fminf(limit, WD_WHEEL_MOTION_TORQUE_LIMIT_NM);
  }
  if (runtime != NULL && runtime->torque_limit_nm > 0.0f) {
    limit = fminf(limit, runtime->torque_limit_nm);
  }
  if (limit < 0.0f) {
    limit = 0.0f;
  }
  return limit;
}

float wd_safety_effective_torque_limit(uint32_t index,
                                       const WdSafetyRuntimeConfig *runtime) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  if (joint == NULL) {
    return 0.0f;
  }
  return joint_torque_limit(joint, runtime);
}

float wd_safety_effective_velocity_limit(uint32_t index,
                                         const WdSafetyRuntimeConfig *runtime) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  if (joint == NULL) {
    return 0.0f;
  }
  return joint_velocity_limit(joint, runtime);
}

int wd_safety_velocity_is_overspeed(uint32_t index,
                                    float measured_velocity_radps,
                                    const WdSafetyRuntimeConfig *runtime) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  float limit;
  if (joint == NULL || joint->mode == WD_ACTUATOR_VELOCITY) {
    return 0;
  }
  (void)runtime;
  limit = WD_LEG_OVERSPEED_TRIP_RADPS;
  if (limit <= 0.0f || wd_safety_is_finite(measured_velocity_radps) == 0) {
    return 0;
  }
  return fabsf(measured_velocity_radps) >= limit ? 1 : 0;
}

int wd_safety_update_overspeed_trip(uint32_t index,
                                    float measured_velocity_radps,
                                    const WdSafetyRuntimeConfig *runtime,
                                    uint8_t *consecutive_count) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  uint8_t count;

  if (consecutive_count == NULL) {
    return 0;
  }
  if (joint == NULL || joint->mode == WD_ACTUATOR_VELOCITY) {
    *consecutive_count = 0u;
    return 0;
  }
  count = *consecutive_count;
  if (wd_safety_is_finite(measured_velocity_radps) != 0 &&
      fabsf(measured_velocity_radps) >=
          WD_LEG_OVERSPEED_IMMEDIATE_TRIP_RADPS) {
    *consecutive_count = WD_OVERSPEED_CONSECUTIVE_SAMPLES;
    return 1;
  }
  if (wd_safety_velocity_is_overspeed(index,
                                      measured_velocity_radps,
                                      runtime) == 0) {
    *consecutive_count = 0u;
    return 0;
  }
  if (count < WD_OVERSPEED_CONSECUTIVE_SAMPLES) {
    ++count;
  }
  *consecutive_count = count;
  return (count >= WD_OVERSPEED_CONSECUTIVE_SAMPLES) ? 1 : 0;
}

float wd_safety_derate_torque_for_velocity(
    uint32_t index,
    float measured_velocity_radps,
    float desired_torque_nm,
    const WdSafetyRuntimeConfig *runtime) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  float speed;
  float scale;

  (void)runtime;
  if (joint == NULL || wd_safety_is_finite(measured_velocity_radps) == 0 ||
      wd_safety_is_finite(desired_torque_nm) == 0) {
    return finite_or_zero(desired_torque_nm);
  }
  if (joint->mode == WD_ACTUATOR_VELOCITY) {
    return desired_torque_nm;
  }
  /* Never weaken torque that opposes the measured motion: it is braking.
   * Only derate torque that would accelerate farther in the same direction. */
  if (measured_velocity_radps * desired_torque_nm <= 0.0f) {
    return desired_torque_nm;
  }

  speed = fabsf(measured_velocity_radps);
  if (speed <= WD_LEG_DERATE_START_RADPS) {
    return desired_torque_nm;
  }
  if (speed >= WD_LEG_OVERSPEED_TRIP_RADPS) {
    return 0.0f;
  }
  scale = (WD_LEG_OVERSPEED_TRIP_RADPS - speed) /
          (WD_LEG_OVERSPEED_TRIP_RADPS - WD_LEG_DERATE_START_RADPS);
  return desired_torque_nm * scale;
}

static uint32_t sanitize_joint_command(WdJointCommand *cmd,
                                       const WdJointSafetyConfig *joint,
                                       const WdMotorCalibration *calibration,
                                       const WdSafetyRuntimeConfig *runtime) {
  uint32_t flags = 0u;
  float q_low;
  float q_high;
  float vel_limit;
  float tau_limit;

  cmd->kp = finite_or_zero_with_status(cmd->kp, &flags);
  cmd->q_des = finite_or_zero_with_status(cmd->q_des, &flags);
  cmd->kd = finite_or_zero_with_status(cmd->kd, &flags);
  cmd->dq_des = finite_or_zero_with_status(cmd->dq_des, &flags);
  cmd->tau_ff = finite_or_zero_with_status(cmd->tau_ff, &flags);

  if (cmd->kp < 0.0f) {
    cmd->kp = 0.0f;
    flags |= WD_STATUS_SETPOINT_CLIPPED;
  }
  if (cmd->kd < 0.0f) {
    cmd->kd = 0.0f;
    flags |= WD_STATUS_SETPOINT_CLIPPED;
  }

  vel_limit = joint_velocity_command_limit(joint, runtime);
  cmd->dq_des = clamp_with_status(cmd->dq_des, -vel_limit, vel_limit, &flags);

  if (joint->mode == WD_ACTUATOR_VELOCITY) {
    if (cmd->kp != 0.0f || cmd->q_des != 0.0f || cmd->tau_ff != 0.0f) {
      flags |= WD_STATUS_SETPOINT_CLIPPED;
    }
    cmd->kp = 0.0f;
    cmd->q_des = 0.0f;
    cmd->kd = clamp_with_status(cmd->kd,
                                0.0f,
                                WD_RS01_MOTION_KD_MAX,
                                &flags);
    cmd->tau_ff = 0.0f;
    return flags;
  }

  if (joint->mode == WD_ACTUATOR_POSITION_PD) {
    if (calibration != NULL &&
        calibration->raw_position_limit_enabled != 0u) {
      q_low = calibration->joint_lower_rad;
      q_high = calibration->joint_upper_rad;
      cmd->q_des = clamp_with_status(cmd->q_des, q_low, q_high, &flags);
    }
    tau_limit = joint_torque_limit(joint, runtime);
    cmd->tau_ff = clamp_with_status(cmd->tau_ff, -tau_limit, tau_limit, &flags);
  }

  return flags;
}

uint32_t wd_safety_sanitize_setpoint(WdSetpointPayload *setpoint,
                                     const WdSafetyRuntimeConfig *runtime) {
  uint32_t flags = 0u;
  uint32_t i;

  if (setpoint == NULL) {
    return WD_STATUS_BAD_PACKET;
  }

  if (wd_motor_calibration_validate_all() == 0) {
    return WD_STATUS_BAD_PACKET | WD_STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED;
  }
  if (setpoint->limit_mode != (uint8_t)WD_LIMIT_CALIBRATED_ABSOLUTE) {
    flags |= WD_STATUS_BAD_PACKET | WD_STATUS_SETPOINT_CLIPPED;
    setpoint->limit_mode = (uint8_t)WD_LIMIT_CALIBRATED_ABSOLUTE;
  }

  for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
    const WdJointSafetyConfig *joint = wd_safety_joint_config(i);
    const WdMotorCalibration *calibration = wd_motor_calibration(i);
    if (joint == NULL || calibration == NULL ||
        joint->mode != calibration->mode) {
      flags |= WD_STATUS_BAD_PACKET;
      continue;
    }
    if (setpoint->actuator_mode[i] != (uint8_t)joint->mode) {
      setpoint->actuator_mode[i] = (uint8_t)joint->mode;
      flags |= WD_STATUS_BAD_PACKET;
    }
    flags |= sanitize_joint_command(
        &setpoint->joint[i], joint, calibration, runtime);
  }

  return flags;
}

float wd_safety_limit_velocity_command(uint32_t index,
                                       float desired_dq,
                                       const WdSafetyRuntimeConfig *runtime,
                                       uint32_t *status_flags) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  float velocity_limit;

  if (joint == NULL) {
    add_status(status_flags, WD_STATUS_BAD_PACKET);
    return 0.0f;
  }

  desired_dq = finite_command_or_zero(desired_dq, status_flags);
  velocity_limit = joint_velocity_command_limit(joint, runtime);
  return clamp_command_with_status(desired_dq,
                                   -velocity_limit,
                                   velocity_limit,
                                   status_flags);
}

float wd_safety_limit_torque_command(uint32_t index,
                                     float desired_tau,
                                     float previous_tau,
                                     float dt_s,
                                     const WdSafetyRuntimeConfig *runtime,
                                     uint32_t *status_flags) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  float torque_limit;
  float slew_rate;
  float max_delta;
  float delta;

  if (joint == NULL) {
    add_status(status_flags, WD_STATUS_BAD_PACKET);
    return 0.0f;
  }

  desired_tau = finite_command_or_zero(desired_tau, status_flags);
  previous_tau = finite_or_zero(previous_tau);
  torque_limit = joint_torque_limit(joint, runtime);
  desired_tau = clamp_command_with_status(desired_tau,
                                          -torque_limit,
                                          torque_limit,
                                          status_flags);

  slew_rate = (runtime != NULL) ? runtime->torque_slew_rate_nm_per_s : 0.0f;
  if (wd_safety_is_finite(dt_s) != 0 && dt_s > 0.0f &&
      wd_safety_is_finite(slew_rate) != 0 && slew_rate > 0.0f) {
    max_delta = slew_rate * dt_s;
    delta = desired_tau - previous_tau;
    if (delta > max_delta) {
      desired_tau = previous_tau + max_delta;
      add_status(status_flags,
                 WD_STATUS_COMMAND_CLIPPED | WD_STATUS_TORQUE_SLEW_LIMITED);
    } else if (delta < -max_delta) {
      desired_tau = previous_tau - max_delta;
      add_status(status_flags,
                 WD_STATUS_COMMAND_CLIPPED | WD_STATUS_TORQUE_SLEW_LIMITED);
    }
  }

  return desired_tau;
}

float wd_safety_limit_torque_command_motion_aware(
    uint32_t index,
    float desired_tau,
    float previous_tau,
    float measured_velocity_radps,
    float dt_s,
    const WdSafetyRuntimeConfig *runtime,
    uint32_t *status_flags) {
  /* Live RS06 and RS01 control has no torque slew limiter. Retain only the
   * finite-value check and per-actuator hard torque ceiling. The dedicated
   * qualification path still calls wd_safety_limit_torque_command() directly
   * with its conservative 50 Nm/s rate. */
  (void)measured_velocity_radps;
  (void)dt_s;
  return wd_safety_limit_torque_command(index,
                                         desired_tau,
                                         previous_tau,
                                         0.0f,
                                         runtime,
                                         status_flags);
}

float wd_safety_compute_velocity_motion_target(
    uint32_t index,
    float virtual_velocity_desired_radps,
    float measured_velocity_radps,
    float kd,
    float previous_torque_nm,
    float dt_s,
    const WdSafetyRuntimeConfig *runtime,
    uint32_t *status_flags,
    float *limited_torque_nm) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  float command_limit;
  float desired_torque;
  float torque;
  float motor_target;

  if (limited_torque_nm != NULL) {
    *limited_torque_nm = 0.0f;
  }
  if (joint == NULL || joint->mode != WD_ACTUATOR_VELOCITY) {
    add_status(status_flags, WD_STATUS_BAD_PACKET);
    return 0.0f;
  }

  virtual_velocity_desired_radps =
      finite_command_or_zero(virtual_velocity_desired_radps, status_flags);
  measured_velocity_radps =
      finite_command_or_zero(measured_velocity_radps, status_flags);
  kd = finite_command_or_zero(kd, status_flags);
  kd = clamp_command_with_status(
      kd, 0.0f, WD_RS01_MOTION_KD_MAX, status_flags);
  command_limit = joint_velocity_command_limit(joint, runtime);
  virtual_velocity_desired_radps = clamp_command_with_status(
      virtual_velocity_desired_radps,
      -command_limit,
      command_limit,
      status_flags);

  if (kd <= 0.0f) {
    return 0.0f;
  }

  desired_torque =
      kd * (virtual_velocity_desired_radps - measured_velocity_radps);
  torque = wd_safety_limit_torque_command_motion_aware(
      index,
      desired_torque,
      previous_torque_nm,
      measured_velocity_radps,
      dt_s,
      runtime,
      status_flags);

  /* Encode the limited joint torque back into the RS01 motion-mode target so
   * the motor-side Kd loop still closes at its native rate. */
  motor_target = measured_velocity_radps + torque / kd;
  motor_target = clamp_command_with_status(
      motor_target, -command_limit, command_limit, status_flags);
  torque = kd * (motor_target - measured_velocity_radps);
  torque = wd_safety_clamp(
      torque,
      -joint_torque_limit(joint, runtime),
      joint_torque_limit(joint, runtime));
  if (limited_torque_nm != NULL) {
    *limited_torque_nm = torque;
  }
  return motor_target;
}

float wd_safety_compute_pd_torque(uint32_t index,
                                  float q,
                                  float dq,
                                  const WdJointCommand *cmd,
                                  WdLimitMode limit_mode,
                                  const WdSafetyRuntimeConfig *runtime) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  const WdMotorCalibration *calibration = wd_motor_calibration(index);
  float q_des;
  float tau;
  float lower;
  float upper;
  float soft_lower;
  float soft_upper;
  float soft_zone;

  (void)limit_mode;
  if (joint == NULL || calibration == NULL || cmd == NULL ||
      joint->mode != WD_ACTUATOR_POSITION_PD ||
      calibration->raw_position_limit_enabled == 0u) {
    return 0.0f;
  }

  q = finite_or_zero(q);
  dq = finite_or_zero(dq);
  q_des = finite_or_zero(cmd->q_des);

  lower = calibration->joint_lower_rad;
  upper = calibration->joint_upper_rad;
  q_des = wd_safety_clamp(q_des, lower, upper);

  tau = finite_or_zero(cmd->kp) * (q_des - q) +
        finite_or_zero(cmd->kd) * (finite_or_zero(cmd->dq_des) - dq) +
        finite_or_zero(cmd->tau_ff);

  soft_zone = (runtime != NULL) ? runtime->limit_soft_zone_rad : WD_DEFAULT_SOFT_ZONE_RAD;
  if (soft_zone > 0.0f) {
    soft_lower = lower + soft_zone;
    soft_upper = upper - soft_zone;
    if (q < soft_lower && tau < 0.0f) {
      /* Inside the lower wall, block only torque that moves farther out.
       * An inward StandUp/RL command keeps its planned magnitude. */
      tau = 0.0f;
    }
    if (q > soft_upper && tau > 0.0f) {
      tau = 0.0f;
    }
  }

  return wd_safety_limit_torque_command(index, tau, tau, 0.0f, runtime, NULL);
}

int wd_safety_position_command_is_passive(const WdJointCommand *cmd) {
  if (cmd == NULL ||
      wd_safety_is_finite(cmd->kp) == 0 ||
      wd_safety_is_finite(cmd->kd) == 0 ||
      wd_safety_is_finite(cmd->tau_ff) == 0) {
    return 0;
  }
  return (cmd->kp == 0.0f && cmd->kd == 0.0f && cmd->tau_ff == 0.0f) ? 1 : 0;
}

float wd_safety_enforce_raw_position_limit(
    uint32_t index,
    float aligned_raw_position_rad,
    float raw_velocity_radps,
    float desired_joint_torque_nm,
    const WdSafetyRuntimeConfig *runtime,
    uint32_t *status_flags) {
  const WdMotorCalibration *calibration = wd_motor_calibration(index);
  float soft_zone;
  float soft_lower;
  float soft_upper;
  float motor_torque;
  float original_motor_torque;
  uint8_t wall_active = 0u;

  if (calibration == NULL || calibration->raw_position_limit_enabled == 0u) {
    return desired_joint_torque_nm;
  }
  if (wd_safety_is_finite(aligned_raw_position_rad) == 0 ||
      wd_safety_is_finite(raw_velocity_radps) == 0 ||
      wd_safety_is_finite(desired_joint_torque_nm) == 0) {
    add_status(status_flags, WD_STATUS_BAD_PACKET);
    return 0.0f;
  }

  soft_zone = (runtime != NULL) ?
      runtime->limit_soft_zone_rad : WD_DEFAULT_SOFT_ZONE_RAD;
  if (soft_zone < 0.0f) {
    soft_zone = 0.0f;
  }
  if ((2.0f * soft_zone) >=
      (calibration->raw_hard_upper_rad -
       calibration->raw_hard_lower_rad)) {
    soft_zone = 0.25f *
        (calibration->raw_hard_upper_rad -
         calibration->raw_hard_lower_rad);
  }
  soft_lower = calibration->raw_hard_lower_rad + soft_zone;
  soft_upper = calibration->raw_hard_upper_rad - soft_zone;
  motor_torque = calibration->direction_sign * desired_joint_torque_nm;
  original_motor_torque = motor_torque;

  if (aligned_raw_position_rad < soft_lower) {
    /* At the numeric raw lower wall, positive motor torque is inward. Never
     * synthesize a restoring impulse here: cancel only outward torque. */
    motor_torque = fmaxf(motor_torque, 0.0f);
    wall_active = 1u;
  }
  if (aligned_raw_position_rad > soft_upper) {
    /* At the numeric raw upper wall, negative motor torque is inward. */
    motor_torque = fminf(motor_torque, 0.0f);
    wall_active = 1u;
  }

  if (wall_active != 0u) {
    add_status(status_flags, WD_STATUS_RAW_POSITION_LIMIT);
  }
  if (motor_torque != original_motor_torque) {
    add_status(status_flags, WD_STATUS_COMMAND_CLIPPED);
  }
  return calibration->direction_sign * motor_torque;
}
