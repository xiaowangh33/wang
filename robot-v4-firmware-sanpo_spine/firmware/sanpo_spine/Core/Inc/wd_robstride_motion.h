#ifndef INC_WD_ROBSTRIDE_MOTION_H_
#define INC_WD_ROBSTRIDE_MOTION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RobStride private protocol communication type 1 (motion-control mode).
 *
 * The RS01 manual defines the motor-side control law as:
 *   torque = kd * (velocity_desired - velocity_measured)
 *          + kp * (position_desired - position_measured)
 *          + torque_feedforward.
 *
 * The wheel deployment uses kp=0, kd=0.4 and torque_feedforward=0 so this
 * matches RobotLab's explicit DCMotor velocity actuator without the separate
 * speed-mode PI controller.
 */
#define WD_ROBSTRIDE_MOTION_COMM_TYPE (1u)
#define WD_RS01_MOTION_POSITION_MAX_RAD (12.57f)
#define WD_RS01_MOTION_VELOCITY_MAX_RADPS (44.0f)
#define WD_RS01_MOTION_TORQUE_MAX_NM (17.0f)
#define WD_RS01_MOTION_KP_MAX (500.0f)
#define WD_RS01_MOTION_KD_MAX (5.0f)

/* Build the complete 29-bit extended CAN identifier and eight-byte payload.
 * Returns 1 on success and 0 for an invalid motor id, pointer or non-finite
 * input. Finite values outside the documented RS01 ranges are clamped. */
int wd_robstride_build_motion_command(uint8_t motor_id,
                                      float torque_feedforward_nm,
                                      float position_desired_rad,
                                      float velocity_desired_radps,
                                      float kp,
                                      float kd,
                                      uint32_t *extended_can_id,
                                      uint8_t payload[8]);

#ifdef __cplusplus
}
#endif

#endif /* INC_WD_ROBSTRIDE_MOTION_H_ */
