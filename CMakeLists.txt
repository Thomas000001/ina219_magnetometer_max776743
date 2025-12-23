# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ina219_magnetometer_max77643)

target_sources(app PRIVATE 
    src/main.c
    src/icm20948_zephyr.c
    src/xyzFloat_c.c
    src/ina219.c
    src/max77643.c
    src/signal_reconstruction.c
    src/motor_period_detector.c
)

target_include_directories(app PRIVATE
    src
    ${ZEPHYR_BASE}/include
    ${ZEPHYR_BASE}/drivers
)

zephyr_library_include_directories(${ZEPHYR_BASE}/include)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/logging)