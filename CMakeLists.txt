# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Nitin Kumar

cmake_minimum_required(VERSION 3.20)

project(crowpilot_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(crowpilot_core STATIC
    core/src/output.cpp
    core/src/safety.cpp
    core/src/sbus.cpp
)

target_include_directories(crowpilot_core PUBLIC core/include)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(crowpilot_core PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Werror
    )
endif()

enable_testing()

add_executable(crowpilot_core_tests tests/core_tests.cpp)
target_link_libraries(crowpilot_core_tests PRIVATE crowpilot_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(crowpilot_core_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Werror
    )
endif()

add_test(NAME crowpilot_core_tests COMMAND crowpilot_core_tests)
