#ifndef INC_WD_ACTUATOR_UNITS_H_
#define INC_WD_ACTUATOR_UNITS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* The RS06 data sheet specifies Kt against RMS phase current, while iq_ref
 * (0x7006) and i_limit (0x7018) use peak phase-current amperes. */
#define WD_RS06_KT_NM_PER_ARMS (1.09f)
#define WD_PHASE_CURRENT_PEAK_PER_RMS (1.41421356237309504880f)

float wd_rs06_torque_nm_to_iq_peak_a(float torque_nm);

#ifdef __cplusplus
}
#endif

#endif /* INC_WD_ACTUATOR_UNITS_H_ */
