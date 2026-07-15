#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
HIPNUC_PRODUCTS_ROOT="${HIPNUC_PRODUCTS_ROOT:-/home/gu/new_dog_robot/四足机器人/QuardLegAllPrograms/imu/products-master}"
BUILD_DIR="${DIR}/.imu_mapping_check_build"

mkdir -p "${BUILD_DIR}"

cc -O2 -std=c99 -D_DEFAULT_SOURCE \
    -I"${HIPNUC_PRODUCTS_ROOT}/drivers" \
    -c "${HIPNUC_PRODUCTS_ROOT}/drivers/hipnuc_dec.c" \
    -o "${BUILD_DIR}/hipnuc_dec.o"

cc -O2 -std=c99 -D_DEFAULT_SOURCE \
    -I"${HIPNUC_PRODUCTS_ROOT}/examples/C" \
    -c "${HIPNUC_PRODUCTS_ROOT}/examples/C/serial_port.c" \
    -o "${BUILD_DIR}/serial_port.o"

g++ -O2 -std=c++17 -Wall -Wextra \
    -o "${DIR}/imu_mapping_check" \
    "${DIR}/imu_mapping_check.cpp" \
    "${BUILD_DIR}/hipnuc_dec.o" \
    "${BUILD_DIR}/serial_port.o" \
    -I"${HIPNUC_PRODUCTS_ROOT}/drivers" \
    -I"${HIPNUC_PRODUCTS_ROOT}/examples/C" \
    -lm

echo "built: ${DIR}/imu_mapping_check"
