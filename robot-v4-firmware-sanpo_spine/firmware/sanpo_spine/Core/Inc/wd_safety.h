#ifndef INC_WD_SAFETY_H_
#define INC_WD_SAFETY_H_

#include "wd_protocol.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  WdActuatorMode mode;
  float effort_limit_nm;
  float velocity_limit_radps;
} WdJointSafetyConfig;

typedef struct {
  float torque_limit_nm;
  float velocity_limit_radps;
  float torque_slew_rate_nm_per_s;
  float limit_soft_zone_rad;
} WdSafetyRuntimeConfig;

/* Three fresh 500 Hz samples reject a one-frame leg-velocity spike while
 * still stopping a sustained >=10 rad/s leg event within roughly 4-6 ms.
 * Continuous wheel actuators are excluded from this trip. */
#define WD_OVERSPEED_CONSECUTIVE_SAMPLES (3u)

/* Qualification excitation remains deliberately slow. Normal live control
 * exposes the full instantaneous torque envelope for both RS06 legs and RS01
 * wheels; a zero active rate disables the former 600 Nm/s wheel ramp. */
#define WD_ENABLE_TORQUE_SLEW_RATE_NM_PER_S (50.0f)
#define WD_ACTIVE_TORQUE_SLEW_RATE_NM_PER_S (0.0f)

void wd_safety_default_runtime_config(WdSafetyRuntimeConfig *config);
const WdJointSafetyConfig *wd_safety_joint_config(uint32_t index);
void wd_safety_default_modes(uint8_t modes[WD_MOTOR_COUNT]);
uint32_t wd_safety_sanitize_setpoint(WdSetpointPayload *setpoint,
                                     const WdSafetyRuntimeConfig *runtime);
float wd_safety_compute_pd_torque(uint32_t index,
                                  float q,
                                  float dq,
                                  const WdJointCommand *cmd,
                                  WdLimitMode limit_mode,
                                  const WdSafetyRuntimeConfig *runtime);

/* An explicitly passive position command must remain zero-current even when
 * the measured joint position is inside a limit-wall zone. */
int wd_safety_position_command_is_passive(const WdJointCommand *cmd);
float wd_safety_limit_torque_command(uint32_t index,
                                     float desired_tau,
                                     float previous_tau,
                                     float dt_s,
                                     const WdSafetyRuntimeConfig *runtime,
                                     uint32_t *status_flags);
float wd_safety_limit_torque_command_motion_aware(
    uint32_t index,
    float desired_tau,
    float previous_tau,
    float measured_velocity_radps,
    float dt_s,
    const WdSafetyRuntimeConfig *runtime,
    uint32_t *status_flags);
/* Convert the training-aligned wheel virtual velocity target into an RS01
 * motion-mode target whose implied Kd torque respects the 17 Nm peak ceiling
 * and the deployment's +/-20 rad/s target range. */
float wd_safety_compute_velocity_motion_target(
    uint32_t index,
    float virtual_velocity_desired_radps,
    float measured_velocity_radps,
    float kd,
    float previous_torque_nm,
    float dt_s,
    const WdSafetyRuntimeConfig *runtime,
    uint32_t *status_flags,
    float *limited_torque_nm);
float wd_safety_limit_velocity_command(uint32_t index,
                                       float desired_dq,
                                       const WdSafetyRuntimeConfig *runtime,
                                       uint32_t *status_flags);
float wd_safety_effective_torque_limit(uint32_t index,
                                       const WdSafetyRuntimeConfig *runtime);
float wd_safety_effective_velocity_limit(uint32_t index,
                                         const WdSafetyRuntimeConfig *runtime);
int wd_safety_velocity_is_overspeed(uint32_t index,
                                    float measured_velocity_radps,
                                    const WdSafetyRuntimeConfig *runtime);
int wd_safety_update_overspeed_trip(uint32_t index,
                                    float measured_velocity_radps,
                                    const WdSafetyRuntimeConfig *runtime,
                                    uint8_t *consecutive_count);
float wd_safety_derate_torque_for_velocity(
    uint32_t index,
    float measured_velocity_radps,
    float desired_torque_nm,
    const WdSafetyRuntimeConfig *runtime);
float wd_safety_enforce_raw_position_limit(
    uint32_t index,
    float aligned_raw_position_rad,
    float raw_velocity_radps,
    float desired_joint_torque_nm,
    const WdSafetyRuntimeConfig *runtime,
    uint32_t *status_flags);
float wd_safety_clamp(float value, float low, float high);
int wd_safety_is_finite(float value);

#ifdef __cplusplus
}
#endif

#endif /* INC_WD_SAFETY_H_ */
