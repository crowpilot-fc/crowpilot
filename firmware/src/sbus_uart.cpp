// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#include "crowpilot/sbus_uart.hpp"

#include "crowpilot/board.hpp"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/time.h"

namespace crowpilot {

void SbusUart::initialize() {
    uart_init(uart0, 100'000U);
    uart_set_format(uart0, 8U, 2U, UART_PARITY_EVEN);
    uart_set_fifo_enabled(uart0, true);
    gpio_set_function(board::kSbusRxPin, GPIO_FUNC_UART);
    gpio_set_inover(board::kSbusRxPin, GPIO_OVERRIDE_INVERT);
}

bool SbusUart::poll(SbusFrame& latest_frame) {
    bool completed = false;
    while (uart_is_readable(uart0)) {
        const auto byte = static_cast<std::uint8_t>(uart_getc(uart0));
        SbusFrame candidate{};
        if (parser_.push(byte, time_us_64(), candidate)) {
            latest_frame = candidate;
            completed = true;
        }
    }
    return completed;
}

}  // namespace crowpilot
