/*
 * WheelDog authoritative motor symbol / absolute encoder calibration table.
 * Edit this one table when a motor or mechanical calibration changes; it is
 * consumed by both MCU C and PC C++ configuration.
 *
 * Fields:
 *   index, CAN, ID, symbol, mode(0=position/1=velocity), direction,
 *   raw_zero, raw_hard_lower, raw_hard_upper,
 *   joint_lower, joint_upper, raw_position_limit_enabled
 *
 * q_joint = direction * (align_2pi(raw, raw_zero) - raw_zero)
 *
 * Hip raw endpoints are measured mechanical stops (2026-07-11). Knee raw_zero
 * is measured at URDF q=0; the confirmed knee travel is +/-2.65 rad.
 * Knee directions were validated by the suspended StandUp test: left +1,
 * right -1. Ankle/wheel entries are continuous velocity actuators: phase is
 * unwrapped, with no raw position boundary. The left wheel motors are mounted
 * with the opposite electrical/mechanical polarity, so their direction is -1.
 * This table already compensates the physical left/right motor mounting
 * polarity. The policy adapter therefore uses identity wheel-frame signs
 * [FL +1, FR +1, HL +1, HR +1] on both observations and actions.
 */
WD_MOTOR_CALIBRATION_ENTRY( 0, 1, 1, "FL_HipX",  0, -1.0,  2.199250,  1.407500, 2.991000, -0.7854, 0.7854, 1)
WD_MOTOR_CALIBRATION_ENTRY( 1, 1, 2, "FL_HipY",  0, -1.0,  5.717500,  3.102000, 8.333000, -2.6180, 2.6180, 1)
WD_MOTOR_CALIBRATION_ENTRY( 2, 1, 3, "FL_Knee",  0,  1.0, -0.123200, -2.773200, 2.526800, -2.6500, 2.6500, 1)
WD_MOTOR_CALIBRATION_ENTRY( 3, 1, 4, "FL_Ankle", 1, -1.0,  0.000000,  0.000000, 0.000000,  0.0000, 0.0000, 0)

WD_MOTOR_CALIBRATION_ENTRY( 4, 2, 1, "FR_HipX",  0, -1.0,  2.698700,  1.910000, 3.487400, -0.7854, 0.7854, 1)
WD_MOTOR_CALIBRATION_ENTRY( 5, 2, 2, "FR_HipY",  0,  1.0,  1.736150, -0.881200, 4.353500, -2.6180, 2.6180, 1)
WD_MOTOR_CALIBRATION_ENTRY( 6, 2, 3, "FR_Knee",  0, -1.0,  2.875100,  0.225100, 5.525100, -2.6500, 2.6500, 1)
WD_MOTOR_CALIBRATION_ENTRY( 7, 2, 4, "FR_Ankle", 1,  1.0,  0.000000,  0.000000, 0.000000,  0.0000, 0.0000, 0)

WD_MOTOR_CALIBRATION_ENTRY( 8, 3, 1, "HL_HipX",  0,  1.0,  2.380600,  1.588700, 3.172500, -0.7854, 0.7854, 1)
WD_MOTOR_CALIBRATION_ENTRY( 9, 3, 2, "HL_HipY",  0, -1.0,  2.246850, -0.367500, 4.861200, -2.6180, 2.6180, 1)
WD_MOTOR_CALIBRATION_ENTRY(10, 3, 3, "HL_Knee",  0,  1.0,  5.830000,  3.180000, 8.480000, -2.6500, 2.6500, 1)
WD_MOTOR_CALIBRATION_ENTRY(11, 3, 4, "HL_Ankle", 1, -1.0,  0.000000,  0.000000, 0.000000,  0.0000, 0.0000, 0)

WD_MOTOR_CALIBRATION_ENTRY(12, 4, 1, "HR_HipX",  0,  1.0,  5.340450,  4.555400, 6.125500, -0.7854, 0.7854, 1)
WD_MOTOR_CALIBRATION_ENTRY(13, 4, 2, "HR_HipY",  0,  1.0,  2.294450, -0.323500, 4.912400, -2.6180, 2.6180, 1)
WD_MOTOR_CALIBRATION_ENTRY(14, 4, 3, "HR_Knee",  0, -1.0, -2.326500, -4.976500, 0.323500, -2.6500, 2.6500, 1)
WD_MOTOR_CALIBRATION_ENTRY(15, 4, 4, "HR_Ankle", 1,  1.0,  0.000000,  0.000000, 0.000000,  0.0000, 0.0000, 0)
