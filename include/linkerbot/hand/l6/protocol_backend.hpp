#include <linkerbot/hand/errors.hpp>
#pragma once
#include <algorithm>
#include <array>
#include <linkerbot/hand/l6/angle.hpp>
#include <linkerbot/hand/l6/config.hpp>
#include <linkerbot/hand/l6/current.hpp>
#include <linkerbot/hand/l6/current_limit.hpp>
#include <linkerbot/hand/l6/fault.hpp>
#include <linkerbot/hand/l6/force_sensor.hpp>
#include <linkerbot/hand/l6/frame.hpp>
#include <linkerbot/hand/l6/stall.hpp>
#include <linkerbot/hand/l6/temperature.hpp>
#include <linkerbot/hand/l6/torque.hpp>
#include <linkerbot/hand/l6/version.hpp>
#include <linkerbot/hand/protocol/transaction.hpp>
#include <stdexcept>
#include <string>

namespace linkerbot::hand::l6::protocol {
    namespace common = ::linkerbot::hand::protocol;
    namespace argument_detail {
        // Keep on-wire identifiers, selectors and lengths in one named record.
        struct FrameArguments {
            uint32_t id;
            uint8_t opcode;
            const std::vector<uint8_t> &payload;
        };
        inline can::Message frame(const FrameArguments &arguments) {
            if (arguments.payload.size() > 7) {
                throw ValidationError("L6 classic payload exceeds 8 bytes");
            }
            can::Message out{};
            out.format = can::FrameFormat::kClassic;
            out.id = arguments.id;
            out.dlc = static_cast<uint8_t>(1 + arguments.payload.size());
            out.data[0] = arguments.opcode;
            std::copy(arguments.payload.begin(), arguments.payload.end(), out.data.begin() + 1);
            return out;
        }
    }
    inline can::Message frame(uint32_t id, uint8_t opcode, const std::vector<uint8_t> &payload = {}) {
        return argument_detail::frame({id, opcode, payload});
    }
    inline common::TransactionPlan command(uint32_t id, uint8_t opcode, const std::vector<uint8_t> &payload = {}) {
        common::TransactionPlan plan;
        plan.frames = {frame(id, opcode, payload)};
        return plan;
    }
    inline const char *telemetry_domain(uint8_t opcode) {
        switch (opcode) {
        case kCurrentLimitOpcode:
            return "current_limit";
        case kTorqueOpcode:
            return "torque";
        case kCurrentOpcode:
            return "current";
        case kTemperatureOpcode:
            return "temperature";
        case kFaultOpcode:
            return "fault";
        default:
            throw ValidationError("unknown L6 telemetry opcode");
        }
    }
    inline common::Progress telemetry(uint32_t id, uint8_t opcode, const can::Message &msg,
                                      common::Origin origin = common::Origin::response) {
        if (!is_classic_response(msg, id, 7, opcode)) {
            return common::Progress::unmatched();
        }
        std::array<uint8_t, 6> raw{};
        std::copy_n(msg.data.begin() + 1, 6, raw.begin());
        if (opcode == kFaultOpcode) {
            // Progress owns this payload through std::any; covered by ownership regression tests.
            // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
            return common::Progress::complete(FaultData{Fault::from_raw(raw), {}}, origin);
        }
        switch (opcode) {
        case kCurrentLimitOpcode:
            // Progress owns this payload through std::any; covered by ownership regression tests.
            // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
            return common::Progress::complete(CurrentLimitData{CurrentLimit::from_array(raw), {}}, origin);
        case kTorqueOpcode:
            // Progress owns this payload through std::any; covered by ownership regression tests.
            // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
            return common::Progress::complete(TorqueData{Torque::from_array(raw), {}}, origin);
        case kCurrentOpcode:
            // Progress owns this payload through std::any; covered by ownership regression tests.
            // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
            return common::Progress::complete(CurrentData{Current::from_array(raw), {}}, origin);
        case kTemperatureOpcode: {
            std::array<float, 6> values{};
            std::copy(raw.begin(), raw.end(), values.begin());
            // Progress owns this payload through std::any; covered by ownership regression tests.
            // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
            return common::Progress::complete(TemperatureData{Temperature::from_array(values), {}}, origin);
        }
        default:
            return common::Progress::unmatched();
        }
    }
    inline common::TransactionPlan read_telemetry(uint32_t id, uint8_t opcode) {
        auto plan = command(id, opcode);
        plan.domain = telemetry_domain(opcode);
        plan.keys = {plan.domain};
        plan.consume = [id, opcode](const can::Message &msg) { return telemetry(id, opcode, msg); };
        return plan;
    }
    inline common::TransactionPlan read_version(uint32_t id, uint8_t opcode) {
        auto plan = command(id, opcode, opcode == kPcbVersionOpcode ? std::vector<uint8_t>{1} : std::vector<uint8_t>{});
        plan.domain = "version/" + std::to_string(opcode);
        plan.keys = {plan.domain};
        plan.consume = [id, opcode](const can::Message &msg) {
            const uint8_t n = opcode == kPcbVersionOpcode ? 5 : 4;
            if (!is_classic_response(msg, id, n, opcode) || (n == 5 && msg.data[1] != 1)) {
                return common::Progress::unmatched();
            }
            const size_t first = n == 5 ? 2 : 1;
            return common::Progress::complete(Version{msg.data[first], msg.data[first + 1], msg.data[first + 2]});
        };
        return plan;
    }
    inline common::TransactionPlan read_serial(uint32_t id) {
        auto plan = command(id, kSerialOpcode);
        plan.domain = "version.serial";
        plan.keys = {plan.domain};
        struct Assembly {
            std::array<std::optional<std::array<uint8_t, 6>>, 4> rows;
            // Slot zero is shared by both encodings; subsequent markers select one.
            enum class Encoding { unknown, indices, offsets } encoding = Encoding::unknown;
        };
        plan.consume = [id, assembly = Assembly{}](const can::Message &msg) mutable {
            // Firmware may omit padding from the last fragment. Earlier
            // fragments must still carry their complete six-byte payload.
            if (!is_classic_frame(msg, id) || msg.dlc < 3 || msg.dlc > 8 || msg.data[0] != kSerialOpcode ||
                (msg.dlc < 8 && msg.data[1] != 3 && msg.data[1] != 18)) {
                return common::Progress::unmatched();
            }
            const auto marker = msg.data[1];
            size_t slot;
            if (marker == 0) {
                slot = 0;
            } else if (marker <= 3) {
                if (assembly.encoding == Assembly::Encoding::offsets) {
                    throw ::linkerbot::hand::ProtocolError("mixed L6 serial fragment encodings");
                }
                assembly.encoding = Assembly::Encoding::indices;
                slot = marker;
            } else if (marker % 6 == 0 && marker <= 18) {
                if (assembly.encoding == Assembly::Encoding::indices) {
                    throw ::linkerbot::hand::ProtocolError("mixed L6 serial fragment encodings");
                }
                assembly.encoding = Assembly::Encoding::offsets;
                slot = marker / 6;
            } else {
                throw ::linkerbot::hand::ProtocolError("invalid L6 serial fragment marker");
            }
            std::array<uint8_t, 6> bytes{};
            std::copy_n(msg.data.begin() + 2, msg.dlc - 2, bytes.begin());
            if (assembly.rows[slot] && *assembly.rows[slot] != bytes) {
                throw ::linkerbot::hand::ProtocolError("conflicting L6 serial fragment");
            }
            assembly.rows[slot] = bytes;
            const bool head = assembly.rows[0] && assembly.rows[1] && assembly.rows[2];
            if (!head || (assembly.encoding != Assembly::Encoding::offsets && !assembly.rows[3])) {
                return common::Progress::pending();
            }
            std::string serial;
            for (const auto &row : assembly.rows) {
                if (!row) {
                    break;
                }
                serial.append(row->begin(), row->end());
            }
            while (!serial.empty() && serial.back() == 0) {
                serial.pop_back();
            }
            return common::Progress::complete(std::move(serial));
        };
        return plan;
    }
    inline common::TransactionPlan set_id(uint32_t endpoint, uint8_t subcommand, uint8_t id) {
        auto plan = command(endpoint, kSetIdOpcode, {subcommand, id});
        plan.domain = "config.id";
        plan.keys = {plan.domain};
        plan.command_family = "configuration";
        plan.consume = [endpoint](const can::Message &msg) {
            if (!is_classic_response(msg, endpoint, 3, kSetIdOpcode)) {
                return common::Progress::unmatched();
            }
            return common::Progress::complete(CanIds{msg.data[1], msg.data[2]});
        };
        return plan;
    }
    inline common::TransactionPlan filled_command(uint32_t endpoint, uint8_t opcode, uint8_t last, uint8_t dlc) {
        if (!dlc || dlc > 8) {
            throw ValidationError("invalid L6 filled command length");
        }
        std::vector<uint8_t> payload(dlc - 1, opcode);
        if (!payload.empty()) {
            payload.back() = last;
        }
        return command(endpoint, opcode, payload);
    }
    inline const char *stall_domain(uint8_t opcode) {
        switch (opcode) {
        case kStallTimeOpcode:
            return "stall.time";
        case kStallThresholdOpcode:
            return "stall.threshold";
        case kStallTorqueOpcode:
            return "stall.torque";
        default:
            throw ValidationError("invalid L6 stall opcode");
        }
    }
    inline common::Progress stall(uint32_t id, uint8_t opcode, const can::Message &msg,
                                  common::Origin origin = common::Origin::response) {
        if (!is_classic_response(msg, id, 7, opcode)) {
            return common::Progress::unmatched();
        }
        std::array<float, 6> values{};
        const float scale = opcode == kStallTimeOpcode ? 10.0f : 1400.0f / 255.0f;
        for (size_t i = 0; i < 6; ++i) {
            values[i] = static_cast<float>(msg.data[i + 1]) * scale;
        }
        if (opcode == kStallTimeOpcode) {
            return common::Progress::complete(StallTimeData{StallTime::from_array(values), {}}, origin);
        }
        return common::Progress::complete(StallLimitData{StallLimit::from_array(values), {}}, origin);
    }
    inline common::TransactionPlan read_stall(uint32_t id, uint8_t opcode) {
        auto plan = command(id, opcode);
        plan.domain = stall_domain(opcode);
        plan.keys = {plan.domain};
        plan.consume = [id, opcode](const can::Message &msg) { return stall(id, opcode, msg); };
        return plan;
    }
    inline common::TransactionPlan angle(uint32_t id, const std::optional<std::array<uint8_t, 6>> &raw, bool feedback,
                                         std::chrono::milliseconds late_guard,
                                         std::optional<std::chrono::microseconds> optional_window = {}) {
        auto plan =
            command(id, kAngleOpcode, raw ? std::vector<uint8_t>(raw->begin(), raw->end()) : std::vector<uint8_t>{});
        plan.domain = "angle";
        plan.keys = {"angle"};
        plan.exclusion_groups = {"l6.actuation"};
        if (raw) {
            plan.command_family = "angle_targets";
        }
        if (!feedback) {
            return plan;
        }
        plan.domain = "angle";
        plan.late_guard = late_guard;
        plan.poison_on_timeout = false;
        plan.optional_response_window = optional_window;
        plan.consume = [id, control = raw.has_value()](const can::Message &msg) {
            if (msg.id != id || msg.dlc == 0 || msg.data[0] != kAngleOpcode) {
                return common::Progress::unmatched();
            }
            if (!is_classic_response(msg, id, 7, kAngleOpcode)) {
                auto progress = common::Progress::pending();
                progress.protocol_errors = 1;
                return progress;
            }
            std::array<uint8_t, 6> bytes{};
            for (size_t i = 0; i < 6; ++i) {
                bytes[i] = msg.data[i + 1];
            }
            AngleData data{Angle::from_array(raw_to_percents(bytes)),
                           {},
                           0,
                           0,
                           control ? SampleOrigin::CommandFeedback : SampleOrigin::Query,
                           {},
                           bytes};
            return common::Progress::complete(data,
                                              control ? common::Origin::command_feedback : common::Origin::response);
        };
        return plan;
    }
    inline std::string force_domain(uint8_t opcode) { return "tactile/" + std::to_string(opcode); }
    inline common::TransactionPlan read_force(uint32_t id, uint8_t opcode, std::chrono::milliseconds guard) {
        auto plan = command(id, opcode, {kForceSelectByte});
        plan.domain = force_domain(opcode);
        plan.keys = {plan.domain};
        plan.exclusion_groups = {"l6.actuation"};
        plan.poison_on_timeout = false;
        plan.late_guard = guard;
        struct Assembly {
            std::array<std::optional<std::array<uint8_t, 6>>, 12> rows{};
            size_t count = 0;
        };
        plan.consume = [id, opcode, assembly = Assembly{}](const can::Message &msg) mutable {
            if (msg.id != id || msg.dlc == 0 || msg.data[0] != opcode) {
                return common::Progress::unmatched();
            }
            auto fail = [](const char *text, bool conflict = false) {
                auto progress = common::Progress::pending();
                progress.kind = common::ProgressKind::failed;
                progress.error = std::make_exception_ptr(::linkerbot::hand::ProtocolError(text));
                progress.conflicting_duplicates = conflict ? 1 : 0;
                return progress;
            };
            if (!is_classic_can(msg) || msg.dlc != 8) {
                return fail("Invalid L6 tactile frame type or DLC");
            }
            const auto marker = msg.data[1];
            uint8_t index = marker;
            if (marker >= 12) {
                if ((marker & 15) != 0) {
                    return fail("Invalid L6 tactile row marker");
                }
                index = marker >> 4;
            }
            if (index >= 12) {
                return fail("L6 tactile row is out of range");
            }
            std::array<uint8_t, 6> bytes{};
            std::copy_n(msg.data.begin() + 2, 6, bytes.begin());
            auto &row = assembly.rows[index];
            if (row) {
                if (*row != bytes) {
                    return fail("Force sensor row conflicts with duplicate", true);
                }
                auto progress = common::Progress::pending();
                progress.duplicates = 1;
                return progress;
            }
            row = bytes;
            ++assembly.count;
            if (assembly.count != 12) {
                return common::Progress::pending();
            }
            ForceSensorData data;
            for (size_t i = 0; i < 12; ++i) {
                data.values[i] = *assembly.rows[i];
            }
            return common::Progress::complete(data);
        };
        return plan;
    }
} // namespace linkerbot::hand::l6::protocol
