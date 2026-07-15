#include "wd_actuator_units.h"

#include <math.h>

float wd_rs06_torque_nm_to_iq_peak_a(float torque_nm) {
  if (!isfinite(torque_nm)) {
    return 0.0f;
  }
  return torque_nm * WD_PHASE_CURRENT_PEAK_PER_RMS /
         WD_RS06_KT_NM_PER_ARMS;
}
