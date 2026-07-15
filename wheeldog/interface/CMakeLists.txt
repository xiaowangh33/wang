cmake_minimum_required(VERSION 3.10)

add_compile_options(-fPIC)

include_directories(
    ./robot
    ./robot/hal
    ./robot/hardware
    ./user_command
)
# file(GLOB_RECURSE SIMULATOIN_SRC "robot/simulation/*.hpp")

# ==== Gamepad SDK ====
file(GLOB_RECURSE USER_COMMAND_SRC "user_command/*.cpp")

set(GAMEPAD_SDK_PATH "${THIRD_PARTY}/gamepad")
include_directories(${GAMEPAD_SDK_PATH}/include)
file(GLOB_RECURSE GAMEPAD_SDK_SRC "${GAMEPAD_SDK_PATH}/src/*.cpp")


# ==== HiPNUC IMU driver (hardware mode only) ====
set(HARDWARE_SRC "")
if (NOT BUILD_SIM)
  set(HIPNUC_PRODUCTS_ROOT
      "/home/gu/new_dog_robot/四足机器人/QuardLegAllPrograms/imu/products-master"
      CACHE PATH "HiPNUC products-master root")
  set(HIPNUC_DEC_SRC "${HIPNUC_PRODUCTS_ROOT}/drivers/hipnuc_dec.c")
  set(HIPNUC_SERIAL_SRC "${HIPNUC_PRODUCTS_ROOT}/examples/C/serial_port.c")
  if (NOT EXISTS "${HIPNUC_DEC_SRC}" OR NOT EXISTS "${HIPNUC_SERIAL_SRC}")
    message(FATAL_ERROR
      "HiPNUC IMU source not found. Set HIPNUC_PRODUCTS_ROOT to products-master. "
      "Current: ${HIPNUC_PRODUCTS_ROOT}")
  endif()
  include_directories(
    "${HIPNUC_PRODUCTS_ROOT}/drivers"
    "${HIPNUC_PRODUCTS_ROOT}/examples/C"
  )
  list(APPEND HARDWARE_SRC
    robot/hardware/wheeled_dog_hardware_interface.cpp
    robot/hardware/hipnuc_imu_hal.cpp
    "${HIPNUC_DEC_SRC}"
    "${HIPNUC_SERIAL_SRC}"
  )
endif()


# ==== Mujoco Interface (仅仿真模式) ====
if (USE_MJCPP AND BUILD_SIM)
  message(STATUS "✅ USE_MJCPP is ON (simulation). Including MujocoInterface headers.")
  set(MUJOCO_ROOT "${THIRD_PARTY}/mujoco/${BUILD_PLATFORM}")
  include_directories(${MUJOCO_ROOT}/include)
  link_directories(${MUJOCO_ROOT}/lib)
endif()



# ==== BUILD STATIC LIBRARY ====
add_library(interface STATIC ${USER_COMMAND_SRC} ${GAMEPAD_SDK_SRC} ${HARDWARE_SRC})
target_compile_features(interface PUBLIC cxx_std_17)
target_link_libraries(interface PUBLIC -lpthread -lm -lrt -ldl -lstdc++)
