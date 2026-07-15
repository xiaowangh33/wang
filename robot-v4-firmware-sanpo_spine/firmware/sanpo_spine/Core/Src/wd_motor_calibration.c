#include "wd_motor_calibration.h"

#include <math.h>
#include <stddef.h>

static const WdMotorCalibration kMotorCalibration[WD_MOTOR_COUNT] = {
#define WD_MOTOR_CALIBRATION_ENTRY(index, can, id, symbol, mode, sign, zero, raw_lo, raw_hi, joint_lo, joint_hi, enabled) \
    {symbol, (uint8_t)(can), (uint8_t)(id), (WdActuatorMode)(mode), \
     (float)(sign), (float)(zero), (float)(raw_lo), (float)(raw_hi), \
     (float)(joint_lo), (float)(joint_hi), (uint8_t)(enabled)},
#include "../../../../../wheeldog/description/motor_absolute_calibration_table.inc"
#undef WD_MOTOR_CALIBRATION_ENTRY
};

const WdMotorCalibration *wd_motor_calibration(uint32_t index) {
  return (index < WD_MOTOR_COUNT) ? &kMotorCalibration[index] : NULL;
}

float wd_motor_direction_sign(uint32_t index) {
  const WdMotorCalibration *calibration = wd_motor_calibration(index);
  return (calibration != NULL && calibration->direction_sign < 0.0f) ?
      -1.0f : 1.0f;
}

float wd_motor_raw_wrap_delta(float current_rad, float reference_rad) {
  float delta;
  const float period = WD_MOTOR_RAW_WRAP_PERIOD_RAD;
  const float half_period = 0.5f * period;

  if (!isfinite(current_rad) || !isfinite(reference_rad)) {
    return current_rad - reference_rad;
  }
  delta = fmodf(current_rad - reference_rad, period);
  if (delta > half_period) {
    delta -= period;
  } else if (delta < -half_period) {
    delta += period;
  }
  return delta;
}

float wd_motor_align_raw(uint32_t index, float raw_rad) {
  const WdMotorCalibration *calibration = wd_motor_calibration(index);
  if (calibration == NULL || calibration->raw_position_limit_enabled == 0u) {
    return raw_rad;
  }
  return calibration->raw_zero_rad +
      wd_motor_raw_wrap_delta(raw_rad, calibration->raw_zero_rad);
}

float wd_motor_raw_to_joint(uint32_t index, float raw_rad) {
  const WdMotorCalibration *calibration = wd_motor_calibration(index);
  if (calibration == NULL) {
    return 0.0f;
  }
  return calibration->direction_sign *
      (wd_motor_align_raw(index, raw_rad) - calibration->raw_zero_rad);
}

int wd_motor_calibration_validate_all(void) {
  uint32_t index;
  for (index = 0u; index < WD_MOTOR_COUNT; ++index) {
    const WdMotorCalibration *entry = &kMotorCalibration[index];
    if (entry->symbol == NULL || entry->can_number != (index / 4u) + 1u ||
        entry->motor_id != (index % 4u) + 1u ||
        (entry->direction_sign != 1.0f && entry->direction_sign != -1.0f)) {
      return 0;
    }
    if (entry->mode == WD_ACTUATOR_POSITION_PD) {
      if (entry->raw_position_limit_enabled == 0u ||
          !isfinite(entry->raw_zero_rad) ||
          entry->raw_hard_lower_rad >= entry->raw_hard_upper_rad ||
          (entry->raw_hard_upper_rad - entry->raw_hard_lower_rad) >=
              WD_MOTOR_RAW_WRAP_PERIOD_RAD ||
          entry->raw_zero_rad <= entry->raw_hard_lower_rad ||
          entry->raw_zero_rad >= entry->raw_hard_upper_rad ||
          entry->joint_lower_rad >= entry->joint_upper_rad) {
        return 0;
      }
    } else if (entry->mode != WD_ACTUATOR_VELOCITY ||
               entry->raw_position_limit_enabled != 0u) {
      return 0;
    }
  }
  return 1;
}
