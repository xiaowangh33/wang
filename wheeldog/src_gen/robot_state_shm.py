import ctypes

#  共享内存结构体类型为： ShmRobotDataAll
DOF_COUNT = 16

class ShmJointCmd(ctypes.Structure):
    _fields_ = [
        ("kp", ctypes.c_float),
        ("goal_joint_pos", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("goal_joint_vel", ctypes.c_float),
        ("torque_feedforward", ctypes.c_float),
    ]

class ShmRobotData(ctypes.Structure):
    _fields_ = [
        ("JointPosition", ctypes.c_float * DOF_COUNT),
        ("JointVelocity", ctypes.c_float * DOF_COUNT),
        ("JointTorque", ctypes.c_float * DOF_COUNT),
        ("ImuRpy", ctypes.c_float * 3),
        ("ImuAcc", ctypes.c_float * 3),
        ("ImuOmega", ctypes.c_float * 3),
    ]

class ShmRobotDataAll(ctypes.Structure):
    _fields_ = [
        ("robot_data", ShmRobotData),
        ("joint_cmd", ShmJointCmd * DOF_COUNT),
        ("actuator_mode", ctypes.c_uint8 * DOF_COUNT),
        ("kp_base", ctypes.c_float * 3),
        ("kd_base", ctypes.c_float * 3),
        ("front_leg_kp_scale", ctypes.c_float),
        ("back_leg_kp_scale", ctypes.c_float),
        ("front_leg_kd_scale", ctypes.c_float),
        ("back_leg_kd_scale", ctypes.c_float),
    ]

# variables (module-level ctypes)
