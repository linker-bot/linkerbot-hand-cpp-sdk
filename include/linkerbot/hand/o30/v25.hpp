#pragma once
#include <array>
#include <cmath>
#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/hand/o30/transaction.hpp>
namespace linkerbot::hand::o30::v25 {
    // Public order -> compact wire slots, distinct from logical V36 joint IDs.
    inline constexpr std::array<size_t, 20> kSlots{0,  5,  6,  7,  8,  9,  10, 11, 12, 13,
                                                   14, 16, 17, 18, 19, 20, 21, 22, 23, 24};
    inline constexpr uint8_t kWireSize = 25;
    inline constexpr uint8_t kMiMotionTime = 0x08;
    inline std::vector<uint8_t> encode(const std::vector<double> &values) {
        if (values.size() != kSlots.size()) {
            throw ValidationError("O30 V25 requires twenty values");
        }
        std::vector<uint8_t> wire(kWireSize);
        for (size_t i = 0; i < values.size(); ++i) {
            if (!std::isfinite(values[i]) || std::trunc(values[i]) != values[i] || values[i] < 0 || values[i] > 255) {
                throw ValidationError("O30 V25 values must be integer bytes");
            }
            wire[kSlots[i]] = static_cast<uint8_t>(values[i]);
        }
        return wire;
    }
    inline std::vector<double> decode(const std::vector<uint8_t> &wire) {
        if (wire.size() != kWireSize) {
            throw ProtocolError("O30 V25 response must contain 25 bytes");
        }
        std::vector<double> values;
        values.reserve(kSlots.size());
        for (const auto slot : kSlots) {
            values.push_back(wire[slot]);
        }
        return values;
    }
    struct Object {
        uint8_t mi;
        bool rts = false;
    };
    inline Object object(CommandSemantic semantic) {
        switch (semantic) {
        case CommandSemantic::kAngles:
            return {protocol::kMiPosition};
        case CommandSemantic::kSpeeds:
            return {protocol::kMiSpeed};
        case CommandSemantic::kAccelerations:
            return {protocol::kMiAcceleration};
        case CommandSemantic::kCurrentLimits:
            return {protocol::kMiCurrent, true};
        case CommandSemantic::kTorqueLimits:
            return {protocol::kMiTorque};
        case CommandSemantic::kMotionTimes:
            return {kMiMotionTime};
        case CommandSemantic::kStallDetectionTimes:
            return {protocol::kMiStallTime};
        case CommandSemantic::kStallThresholds:
            return {protocol::kMiStallThreshold};
        case CommandSemantic::kStallHoldCurrents:
            return {protocol::kMiStallHoldCurrent};
        default:
            throw UnsupportedOperationError("unsupported O30 V25 command");
        }
    }
    inline Object object(TelemetrySemantic semantic) {
        switch (semantic) {
        case TelemetrySemantic::kAngles:
            return {protocol::kMiPosition};
        case TelemetrySemantic::kMeasuredSpeeds:
            return {protocol::kMiSpeed};
        case TelemetrySemantic::kConfiguredSpeeds:
            return {protocol::kMiSpeed, true};
        case TelemetrySemantic::kConfiguredAccelerations:
            return {protocol::kMiAcceleration, true};
        case TelemetrySemantic::kCurrents:
            return {protocol::kMiCurrent};
        case TelemetrySemantic::kCurrentLimits:
            return {protocol::kMiCurrent, true};
        case TelemetrySemantic::kTorques:
            return {protocol::kMiTorque};
        case TelemetrySemantic::kTorqueLimits:
            return {protocol::kMiTorque, true};
        case TelemetrySemantic::kMotionTimes:
            return {kMiMotionTime, true};
        case TelemetrySemantic::kTemperatures:
            return {protocol::kMiTemperature};
        case TelemetrySemantic::kVoltages:
            return {protocol::kMiVoltage};
        case TelemetrySemantic::kFaults:
            return {protocol::kMiFaultV004};
        case TelemetrySemantic::kStallDetectionTimes:
            return {protocol::kMiStallTime, true};
        case TelemetrySemantic::kStallThresholds:
            return {protocol::kMiStallThreshold, true};
        case TelemetrySemantic::kStallHoldCurrents:
            return {protocol::kMiStallHoldCurrent, true};
        default:
            throw UnsupportedOperationError("unsupported O30 V25 telemetry");
        }
    }
    inline std::string domain(Object object) {
        return "v25/" + std::to_string(object.mi) + (object.rts ? "/set" : "/actual");
    }
    inline hand::protocol::TransactionPlan transaction(uint32_t id, Object object,
                                                       const std::vector<uint8_t> &payload = {}) {
        if (id > 0x3FE) {
            throw ValidationError("O30 V25 request ID must be 0..0x3FE");
        }
        if (!payload.empty() && payload.size() != kWireSize) {
            throw ValidationError("O30 V25 writes require 25 bytes");
        }
        auto plan = protocol::transaction(id, object.mi, 0, kWireSize, payload, object.rts);
        plan.domain = domain(object);
        plan.late_guard = std::chrono::milliseconds{10};
        plan.poison_on_timeout = false;
        // A short fragment remains pending. Coverage, rather than fragment
        // length or an echoed target, proves completion (Python HOPRangeAssembler).
        struct Assembly {
            std::vector<uint8_t> bytes = std::vector<uint8_t>(kWireSize);
            std::array<bool, kWireSize> covered{};
        };
        plan.consume = [response_id = protocol::expected_response_id(id), object,
                        assembly = Assembly{}](const can::Message &message) mutable {
            using Progress = hand::protocol::Progress;
            if (message.id != response_id || !message.is_fd || message.is_extended) {
                return Progress::unmatched();
            }
            if (message.dlc > can::kMaxDataLen) {
                throw ProtocolError("invalid V25 frame length");
            }
            const auto frame = protocol::parse_frame(message.data.data(), message.dlc);
            if (frame.mi == protocol::kMiCommError) {
                if (frame.payload.size() != 1) {
                    throw ProtocolError("V25 communication error must contain one byte");
                }
                if (frame.payload.front() == 0) {
                    return Progress::unmatched();
                }
                throw ProtocolError("V25 HOP communication error " + protocol::hex_byte(frame.payload.front()));
            }
            if (frame.mi != object.mi || frame.rts != object.rts) {
                return Progress::unmatched();
            }
            if (frame.payload.empty()) {
                throw ProtocolError("empty V25 response fragment");
            }
            const auto offset = static_cast<size_t>(frame.si);
            const auto end = offset + frame.payload.size();
            if (offset >= kWireSize) {
                return Progress::unmatched();
            }
            if (end > kWireSize) {
                throw ProtocolError("V25 response overlaps outside object");
            }
            for (size_t i = 0; i < frame.payload.size(); ++i) {
                if (assembly.covered[offset + i] && assembly.bytes[offset + i] != frame.payload[i]) {
                    throw ProtocolError("conflicting V25 response fragment");
                }
                assembly.covered[offset + i] = true;
                assembly.bytes[offset + i] = frame.payload[i];
            }
            if (!std::all_of(assembly.covered.begin(), assembly.covered.end(), [](bool covered) { return covered; })) {
                return Progress::pending();
            }
            return Progress::complete(assembly.bytes);
        };
        return plan;
    }
}
