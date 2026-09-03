// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace crowpilot::board {

inline constexpr std::uint32_t kBoardId = 0x43505731U;
inline constexpr const char* kBoardName = "weact-rp2350a-v10";

inline constexpr unsigned kMpuInterruptPin = 0;
inline constexpr unsigned kSbusRxPin = 1;
inline constexpr std::array<unsigned, 8> kPwmOutputPins = {
    2, 3, 4, 5, 6, 7, 8, 9,
};
inline constexpr unsigned kMpuSckPin = 10;
inline constexpr unsigned kMpuMosiPin = 11;
inline constexpr unsigned kMpuMisoPin = 12;
inline constexpr unsigned kMpuCsPin = 13;
inline constexpr unsigned kBmpSdaPin = 14;
inline constexpr unsigned kBmpSclPin = 15;
inline constexpr unsigned kSdMisoPin = 16;
inline constexpr unsigned kSdCsPin = 17;
inline constexpr unsigned kSdSckPin = 18;
inline constexpr unsigned kSdMosiPin = 19;
inline constexpr unsigned kCompanionTxPin = 20;
inline constexpr unsigned kCompanionRxPin = 21;
inline constexpr unsigned kCompanionEnablePin = 22;
inline constexpr unsigned kStatusLedPin = 25;
inline constexpr std::array<unsigned, 2> kExpansionPins = {26, 27};
inline constexpr unsigned kBatteryAdcPin = 28;

consteval bool assigned_pins_are_unique() {
    constexpr std::array<unsigned, 24> assigned = {
        kMpuInterruptPin,
        kSbusRxPin,
        kPwmOutputPins[0],
        kPwmOutputPins[1],
        kPwmOutputPins[2],
        kPwmOutputPins[3],
        kPwmOutputPins[4],
        kPwmOutputPins[5],
        kPwmOutputPins[6],
        kPwmOutputPins[7],
        kMpuSckPin,
        kMpuMosiPin,
        kMpuMisoPin,
        kMpuCsPin,
        kBmpSdaPin,
        kBmpSclPin,
        kSdMisoPin,
        kSdCsPin,
        kSdSckPin,
        kSdMosiPin,
        kCompanionTxPin,
        kCompanionRxPin,
        kCompanionEnablePin,
        kBatteryAdcPin,
    };

    for (std::size_t left = 0; left < assigned.size(); ++left) {
        for (std::size_t right = left + 1U; right < assigned.size(); ++right) {
            if (assigned[left] == assigned[right]) {
                return false;
            }
        }
    }
    return true;
}

static_assert(assigned_pins_are_unique());
static_assert(kSbusRxPin == 1U, "GP1 is UART0 RX");
static_assert(kMpuSckPin == 10U && kMpuMosiPin == 11U &&
              kMpuMisoPin == 12U,
              "MPU6500 must use the SPI1 alternate functions");
static_assert(kBmpSdaPin == 14U && kBmpSclPin == 15U,
              "BMP388 must use I2C1");
static_assert(kSdMisoPin == 16U && kSdSckPin == 18U &&
              kSdMosiPin == 19U,
              "microSD must use the SPI0 alternate functions");
static_assert(kCompanionTxPin == 20U && kCompanionRxPin == 21U,
              "Companion must use UART1");

}  // namespace crowpilot::board
