#include "wd_actuator_units.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int nearly_equal(float lhs, float rhs) {
  return fabsf(lhs - rhs) < 1.0e-5f;
}

int main(void) {
  assert(nearly_equal(wd_rs06_torque_nm_to_iq_peak_a(0.0f), 0.0f));
  assert(nearly_equal(
      wd_rs06_torque_nm_to_iq_peak_a(WD_RS06_KT_NM_PER_ARMS),
      WD_PHASE_CURRENT_PEAK_PER_RMS));
  assert(nearly_equal(wd_rs06_torque_nm_to_iq_peak_a(36.0f),
                      46.707970f));
  assert(nearly_equal(wd_rs06_torque_nm_to_iq_peak_a(-36.0f),
                      -46.707970f));
  assert(nearly_equal(wd_rs06_torque_nm_to_iq_peak_a(NAN), 0.0f));
  assert(nearly_equal(wd_rs06_torque_nm_to_iq_peak_a(INFINITY), 0.0f));
  puts("wd_actuator_units: all tests passed");
  return 0;
}
