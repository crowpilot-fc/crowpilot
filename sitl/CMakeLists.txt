# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Nitin Kumar
#
# Host build for the CrowPilot SITL target. Compiles the firmware logic
# and the simulated HAL into a native executable. The firmware sources
# are unchanged from the Arduino build; only the HAL implementation and
# the Arduino shim differ.
#
#   cmake -B build -S .
#   cmake --build build
#   ./build/crowpilot_sitl

cmake_minimum_required(VERSION 3.16)
project(crowpilot_sitl CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_executable(crowpilot_sitl
  # Host harness and the non-HAL sim stubs.
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SimStorage.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SimTelemetry.cpp

  # Simulated HAL and the rigid-body model behind it.
  ${REPO_ROOT}/src/hal/sim/ImuSim.cpp
  ${REPO_ROOT}/src/hal/sim/BaroSim.cpp
  ${REPO_ROOT}/src/hal/sim/RxSim.cpp
  ${REPO_ROOT}/src/hal/sim/OutSim.cpp
  ${REPO_ROOT}/src/hal/sim/LedSim.cpp
  ${REPO_ROOT}/src/hal/sim/SimPhysicsTailsitter.cpp
  ${REPO_ROOT}/src/hal/sim/SimPhysicsPlane.cpp
  ${REPO_ROOT}/src/hal/sim/SimPhysicsQuad.cpp

  # Firmware logic, identical to the Arduino build.
  ${REPO_ROOT}/src/core/Loop.cpp
  ${REPO_ROOT}/src/sensors/Imu.cpp
  ${REPO_ROOT}/src/sensors/ImuCalibrate.cpp
  ${REPO_ROOT}/src/sensors/Barometer.cpp
  ${REPO_ROOT}/src/radio/Receiver.cpp
  ${REPO_ROOT}/src/estimation/Attitude.cpp
  ${REPO_ROOT}/src/estimation/DynamicNotch.cpp
  ${REPO_ROOT}/src/failsafe/Failsafe.cpp
  ${REPO_ROOT}/src/modes/FlightMode.cpp
  ${REPO_ROOT}/src/control/DesiredState.cpp
  ${REPO_ROOT}/src/control/Pid.cpp
  ${REPO_ROOT}/src/control/RatePid.cpp
  ${REPO_ROOT}/src/control/PlaneStab.cpp
  ${REPO_ROOT}/src/control/LaunchAssist.cpp
  ${REPO_ROOT}/src/airframes/Airframe.cpp
  ${REPO_ROOT}/src/airframes/PlaneSingle.cpp
  ${REPO_ROOT}/src/airframes/PlaneTwinCargo.cpp
  ${REPO_ROOT}/src/airframes/PlaneFlyingWing.cpp
  ${REPO_ROOT}/src/airframes/QuadX.cpp
  ${REPO_ROOT}/src/actuators/Output.cpp
  ${REPO_ROOT}/src/actuators/EscCalibrate.cpp
  ${REPO_ROOT}/src/params/Params.cpp
  ${REPO_ROOT}/src/params/LiveTune.cpp
  ${REPO_ROOT}/src/cli/Cli.cpp
  ${REPO_ROOT}/src/user_hook/UserHook.cpp
  ${REPO_ROOT}/src/user_hook/Budget.cpp
  ${REPO_ROOT}/src/user_hook/PinGuard.cpp
)

target_include_directories(crowpilot_sitl PRIVATE
  ${REPO_ROOT}
  ${CMAKE_CURRENT_SOURCE_DIR}/arduino_shim
)

# Selects the simulated HAL across Config.h and the HAL source guards.
target_compile_definitions(crowpilot_sitl PRIVATE
  BUILD_TARGET=BUILD_TARGET_SITL
)

# Optional airframe override for the host build, e.g.
#   cmake -B build -S . -DSITL_AIRFRAME=AIRFRAME_QUAD_X
# Defaults to the AIRFRAME selected in Config.h.
if(DEFINED SITL_AIRFRAME)
  target_compile_definitions(crowpilot_sitl PRIVATE AIRFRAME=${SITL_AIRFRAME})
endif()

target_compile_options(crowpilot_sitl PRIVATE -Wall)
