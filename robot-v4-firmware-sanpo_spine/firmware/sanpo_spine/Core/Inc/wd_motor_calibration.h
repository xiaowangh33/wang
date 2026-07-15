#ifndef INC_WD_MOTOR_CALIBRATION_H_
#define INC_WD_MOTOR_CALIBRATION_H_

#include "wd_protocol.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WD_MOTOR_RAW_WRAP_PERIOD_RAD (6.28318530717958647692f)

typedef struct {
  const char *symbol;
  uint8_t can_number;
  uint8_t motor_id;
  WdActuatorMode mode;
  float direction_sign;
  float raw_zero_rad;
  float raw_hard_lower_rad;
  float raw_hard_upper_rad;
  float joint_lower_rad;
  float joint_upper_rad;
  uint8_t raw_position_limit_enabled;
} WdMotorCalibration;

/*
 * Authoritative, mechanically calibrated motor-symbol table.
 *
 * q_joint = direction_sign * (align_2pi(raw, raw_zero) - raw_zero)
 * motor torque/velocity command = direction_sign * joint command
 *
 * The four wheel/Ankle entries are continuous velocity actuators: their
 * position is unwrapped for observation, but no raw position boundary is
 * applied. All finite leg joints have an independent raw-domain boundary.
 */
const WdMotorCalibration *wd_motor_calibration(uint32_t index);
float wd_motor_direction_sign(uint32_t index);
float wd_motor_raw_wrap_delta(float current_rad, float reference_rad);
float wd_motor_align_raw(uint32_t index, float raw_rad);
float wd_motor_raw_to_joint(uint32_t index, float raw_rad);
int wd_motor_calibration_validate_all(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_WD_MOTOR_CALIBRATION_H_ */
