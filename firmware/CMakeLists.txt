# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Nitin Kumar

cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "Set PICO_SDK_PATH to a Raspberry Pi Pico SDK checkout")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(crowpilot_firmware VERSION 0.1.0 LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

pico_sdk_init()

add_executable(crowpilot
    ../core/src/output.cpp
    ../core/src/safety.cpp
    ../core/src/sbus.cpp
    src/main.cpp
    src/pwm_outputs.cpp
    src/sbus_uart.cpp
)

target_include_directories(crowpilot PRIVATE
    ../core/include
    include
)

target_compile_options(crowpilot PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-Wconversion>
    $<$<COMPILE_LANGUAGE:CXX>:-Wsign-conversion>
    $<$<COMPILE_LANGUAGE:CXX>:-Werror>
)

target_link_libraries(crowpilot PRIVATE
    pico_stdlib
    hardware_clocks
    hardware_gpio
    hardware_pwm
    hardware_uart
    hardware_watchdog
)

pico_enable_stdio_usb(crowpilot 1)
pico_enable_stdio_uart(crowpilot 0)
pico_add_extra_outputs(crowpilot)
