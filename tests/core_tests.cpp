// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#include "crowpilot/output.hpp"
#include "crowpilot/safety.hpp"
#include "crowpilot/sbus.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

int failures = 0;

void expect(const bool condition, const char* description) {
    if (!condition) {
        std::cerr << "FAIL: " << description << '\n';
        ++failures;
    }
}

std::array<std::uint8_t, crowpilot::kSbusFrameSize> encode_sbus(
    const std::array<std::uint16_t, crowpilot::kSbusChannelCount>& channels,
    const std::uint8_t flags = 0U) {
    std::array<std::uint8_t, crowpilot::kSbusFrameSize> bytes{};
    bytes[0] = 0x0fU;
    for (std::size_t channel = 0; channel < channels.size(); ++channel) {
        const std::size_t first_bit = channel * 11U;
        for (std::size_t bit = 0; bit < 11U; ++bit) {
            if ((channels[channel] & (1U << bit)) != 0U) {
                const std::size_t payload_bit = first_bit + bit;
                const std::size_t byte_index = 1U + payload_bit / 8U;
                const std::size_t bit_index = payload_bit % 8U;
                bytes[byte_index] = static_cast<std::uint8_t>(
                    bytes[byte_index] | (1U << bit_index));
            }
        }
    }
    bytes[23] = flags;
    bytes[24] = 0x00U;
    return bytes;
}

void test_sbus_parser() {
    std::array<std::uint16_t, crowpilot::kSbusChannelCount> channels{};
    for (std::size_t index = 0; index < channels.size(); ++index) {
        channels[index] = static_cast<std::uint16_t>(172U + index * 97U);
    }

    const auto bytes = encode_sbus(channels, 0x05U);
    crowpilot::SbusParser parser;
    crowpilot::SbusFrame frame{};
    bool completed = false;
    for (const auto byte : bytes) {
        completed = parser.push(byte, 123'456U, frame) || completed;
    }

    expect(completed, "SBUS frame completes");
    expect(frame.channels == channels, "SBUS channels decode exactly");
    expect(frame.digital_channel_17, "SBUS digital channel 17 decodes");
    expect(!frame.digital_channel_18, "SBUS digital channel 18 decodes");
    expect(frame.frame_lost, "SBUS frame-lost flag decodes");
    expect(!frame.failsafe, "SBUS failsafe flag decodes");
    expect(frame.received_at_us == 123'456U, "SBUS timestamp is retained");

    expect(!crowpilot::sbus_frame_healthy(frame, 123'457U, 100'000U),
           "Frame-lost flag makes an SBUS frame unhealthy");

    const auto healthy_bytes = encode_sbus(channels);
    for (const auto byte : healthy_bytes) {
        static_cast<void>(parser.push(byte, 1'000U, frame));
    }
    expect(crowpilot::sbus_frame_healthy(frame, 101'000U, 100'000U),
           "SBUS frame is healthy at the timeout boundary");
    expect(!crowpilot::sbus_frame_healthy(frame, 101'001U, 100'000U),
           "Stale SBUS frame is unhealthy");

    const auto failsafe_bytes = encode_sbus(channels, 0x08U);
    for (const auto byte : failsafe_bytes) {
        static_cast<void>(parser.push(byte, 200'000U, frame));
    }
    expect(frame.failsafe, "SBUS failsafe flag decodes");
    expect(!crowpilot::sbus_frame_healthy(frame, 200'001U, 100'000U),
           "Failsafe flag makes an SBUS frame unhealthy");

    auto malformed_bytes = encode_sbus(channels);
    malformed_bytes[24] = 0xffU;
    const auto prior_timestamp = frame.received_at_us;
    bool malformed_completed = false;
    for (const auto byte : malformed_bytes) {
        malformed_completed =
            parser.push(byte, 300'000U, frame) || malformed_completed;
    }
    expect(!malformed_completed, "Invalid SBUS end byte rejects the frame");
    expect(frame.received_at_us == prior_timestamp,
           "Rejected SBUS frame does not replace the last valid frame");

    bool recovered = false;
    for (const auto byte : healthy_bytes) {
        recovered = parser.push(byte, 400'000U, frame) || recovered;
    }
    expect(recovered, "SBUS parser recovers after a malformed frame");
}

void test_output_mapping() {
    const auto routes = crowpilot::trainer_output_routes();
    expect(crowpilot::validate_output_routes(routes),
           "Default trainer output routes validate");

    crowpilot::SbusFrame frame{};
    frame.channels.fill(992U);
    frame.channels[2] = crowpilot::kSbusNominalMin;
    const auto pulses = crowpilot::manual_output_pulses(frame, routes);
    expect(pulses[0] == 1000U, "Minimum throttle maps to 1000 us");
    expect(pulses[1] == 1500U, "Centered aileron maps to 1500 us");
    expect(pulses[2] == 1500U, "Reversed centered aileron remains centered");

    frame.channels[0] = crowpilot::kSbusNominalMin;
    const auto endpoint_pulses = crowpilot::manual_output_pulses(frame, routes);
    expect(endpoint_pulses[1] == 1000U, "Normal servo minimum maps correctly");
    expect(endpoint_pulses[2] == 2000U, "Reversed servo minimum maps to maximum");

    expect(crowpilot::sbus_to_pulse(0U, routes[1]) == 1000U,
           "Values below the SBUS nominal range clamp low");
    expect(crowpilot::sbus_to_pulse(2047U, routes[1]) == 2000U,
           "Values above the SBUS nominal range clamp high");

    auto invalid_routes = routes;
    invalid_routes[0].reversed = true;
    expect(!crowpilot::validate_output_routes(invalid_routes),
           "Reversed motor route is rejected");
    invalid_routes = routes;
    invalid_routes[1].source_channel =
        static_cast<std::uint8_t>(crowpilot::kSbusChannelCount);
    expect(!crowpilot::validate_output_routes(invalid_routes),
           "Out-of-range source channel is rejected");

    const auto safe = crowpilot::safe_output_pulses(routes);
    expect(safe[0] == 1000U, "Motor safe pulse is low throttle");
    for (std::size_t index = 1; index < safe.size(); ++index) {
        expect(safe[index] == 1500U, "Servo safe pulse is centered");
    }
}

void test_safety_machine() {
    crowpilot::SafetyMachine safety;
    crowpilot::SafetyInputs inputs{};

    expect(safety.state() == crowpilot::FlightState::boot,
           "Safety machine begins in boot");
    expect(safety.update(inputs) == crowpilot::FlightState::self_test,
           "Boot advances to self-test");

    inputs.self_tests_complete = true;
    inputs.self_tests_ok = true;
    expect(safety.update(inputs) == crowpilot::FlightState::disarmed,
           "Passing self-tests reaches disarmed");

    inputs.arm_requested = false;
    inputs.receiver_healthy = false;
    static_cast<void>(safety.update(inputs));
    inputs.receiver_healthy = true;
    inputs.arm_requested = true;
    inputs.throttle_low = true;
    expect(safety.update(inputs) == crowpilot::FlightState::disarmed,
           "Receiver absence cannot satisfy the arm-low interlock");

    inputs.arm_requested = false;
    static_cast<void>(safety.update(inputs));
    inputs.throttle_low = false;
    inputs.arm_requested = true;
    expect(safety.update(inputs) == crowpilot::FlightState::disarmed,
           "High throttle prevents arming");

    inputs.throttle_low = true;
    expect(safety.update(inputs) == crowpilot::FlightState::armed,
           "Deliberate low-to-high arm transition arms with low throttle");
    expect(safety.output_authority_enabled(),
           "Only armed state grants output authority");

    inputs.receiver_healthy = false;
    expect(safety.update(inputs) == crowpilot::FlightState::failsafe,
           "Receiver loss while armed enters failsafe");
    expect(!safety.output_authority_enabled(),
           "Failsafe removes output authority");

    inputs.receiver_healthy = true;
    expect(safety.update(inputs) == crowpilot::FlightState::failsafe,
           "Receiver recovery with arm high remains failsafe");

    inputs.arm_requested = false;
    expect(safety.update(inputs) == crowpilot::FlightState::disarmed,
           "Arm release after receiver recovery returns to disarmed");

    inputs.arm_requested = true;
    expect(safety.update(inputs) == crowpilot::FlightState::armed,
           "A new deliberate arm transition can arm again");
    inputs.arm_requested = false;
    expect(safety.update(inputs) == crowpilot::FlightState::disarmed,
           "Arm switch low explicitly disarms");
}

void test_failed_self_test_latches_fault() {
    crowpilot::SafetyMachine safety;
    crowpilot::SafetyInputs inputs{};
    static_cast<void>(safety.update(inputs));
    inputs.self_tests_complete = true;
    inputs.self_tests_ok = false;
    expect(safety.update(inputs) == crowpilot::FlightState::fault,
           "Failed self-test enters fault");
    inputs.self_tests_ok = true;
    expect(safety.update(inputs) == crowpilot::FlightState::fault,
           "Fault remains latched");
}

}  // namespace

int main() {
    test_sbus_parser();
    test_output_mapping();
    test_safety_machine();
    test_failed_self_test_latches_fault();

    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "All CrowPilot core tests passed\n";
    return EXIT_SUCCESS;
}
