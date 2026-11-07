#include <linkerbot/hand/errors.hpp>
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <linkerbot/hand/o6/acceleration.hpp>
#include <linkerbot/hand/o6/angle.hpp>
#include <linkerbot/hand/o6/current.hpp>
#include <linkerbot/hand/o6/fault.hpp>
#include <linkerbot/hand/o6/force_sensor.hpp>
#include <linkerbot/hand/o6/speed.hpp>
#include <linkerbot/hand/o6/stall.hpp>
#include <linkerbot/hand/o6/temperature.hpp>
#include <linkerbot/hand/o6/version.hpp>
#include <linkerbot/hand/protocol/transaction.hpp>
#include <stdexcept>
#include <string>

namespace linkerbot::hand::o6::protocol {
    namespace common = ::linkerbot::hand::protocol;

    // Some O6 firmware revisions transmit one trailing reserved byte on tactile
    // rows (DLC=7) even though the business payload remains 10 x 4. Keep the
    // descriptor shape stable while accepting both wire forms.
    inline bool valid_force_sensor_row(const can::Message &input) { return input.dlc == 6 || input.dlc == 7; }

    inline ForceSensorData
    compact_force_sensor_rows(const std::array<std::array<uint8_t, ForceSensorData::kBytesPerFrame>, 16> &rows,
                              const std::array<bool, 16> &seen) {
        ForceSensorData data;
        size_t output = 0;
        for (size_t marker = 0; marker < seen.size(); ++marker) {
            if (seen[marker]) {
                data.values[output++] = rows[marker];
            }
        }
        if (output != ForceSensorData::kFrameCount) {
            throw ::linkerbot::hand::ProtocolError("O6 tactile row compaction requires 10 unique markers");
        }
        return data;
    }

    inline common::Progress decode_angle(uint32_t id, const can::Message &message,
                                         SampleOrigin origin = SampleOrigin::Query) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 7 ||
            message.data[0] != 0x01) {
            return common::Progress::unmatched();
        }
        std::array<uint8_t, Angle::kJointCount> raw{};
        for (size_t i = 0; i < raw.size(); ++i) {
            raw[i] = message.data[i + 1];
        }
        // Progress owns this payload through std::any; covered by ownership regression tests.
        // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
        return common::Progress::complete(AngleData{Angle::from_array(raw_to_percents(raw)), {}, origin, raw},
                                          origin == SampleOrigin::CommandFeedback ? common::Origin::command_feedback
                                                                                  : common::Origin::response);
    }
    inline common::TransactionPlan read_angle(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0x01;
        common::TransactionPlan plan;
        plan.domain = "angle";
        plan.keys = {"angle"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_angle(id, input, SampleOrigin::Query); };
        return plan;
    }
    inline common::TransactionPlan write_angle(uint32_t id, const std::array<uint8_t, Angle::kJointCount> &raw) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        message.data[0] = 0x01;
        for (size_t i = 0; i < raw.size(); ++i) {
            message.data[i + 1] = raw[i];
        }
        common::TransactionPlan plan;
        plan.keys = {"angle"};
        plan.frames = {message};
        return plan;
    }

    inline common::TransactionPlan
    command_angle_feedback(uint32_t id, const std::array<uint8_t, Angle::kJointCount> &raw,
                           std::chrono::milliseconds late_guard = std::chrono::milliseconds{5}) {
        auto plan = write_angle(id, raw);
        plan.domain = "angle";
        plan.keys = {"angle"};
        plan.exclusion_groups = {"o6.actuation"};
        plan.late_guard = late_guard;
        plan.poison_on_timeout = false;
        plan.consume = [id](const can::Message &input) {
            return decode_angle(id, input, SampleOrigin::CommandFeedback);
        };
        return plan;
    }

    // 0x02 is measured current for a bare query and current-limit feedback for a write.
    inline common::Progress decode_current(uint32_t id, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 7 ||
            message.data[0] != 0x02) {
            return common::Progress::unmatched();
        }
        std::array<uint8_t, kCurrentJointCount> values{};
        std::copy_n(message.data.begin() + 1, values.size(), values.begin());
        return common::Progress::complete(CurrentData{values, {}});
    }
    inline common::TransactionPlan read_current(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0x02;
        common::TransactionPlan plan;
        plan.domain = "current";
        plan.keys = {"o6.0x02"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_current(id, input); };
        return plan;
    }
    inline common::TransactionPlan write_current_limit(uint32_t id,
                                                       const std::array<uint8_t, kCurrentJointCount> &values) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        message.data[0] = 0x02;
        std::copy(values.begin(), values.end(), message.data.begin() + 1);
        common::TransactionPlan plan;
        plan.domain = "current_limit";
        plan.keys = {"o6.0x02"};
        plan.command_family = "current_limits";
        plan.frames = {message};
        plan.publish_after_send = true;
        plan.consume = [id](const can::Message &input) {
            auto decoded = decode_current(id, input);
            if (decoded.kind == common::ProgressKind::complete) {
                const auto data = std::any_cast<CurrentData>(decoded.value);
                decoded.value = CurrentLimitData{data.currents, {}};
                decoded.origin = common::Origin::command_feedback;
            }
            return decoded;
        };
        return plan;
    }

    inline common::Progress decode_speed(uint32_t id, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 7 ||
            message.data[0] != 0x05) {
            return common::Progress::unmatched();
        }
        std::array<float, Speed::kJointCount> values{};
        for (size_t i = 0; i < values.size(); ++i) {
            values[i] = static_cast<float>(message.data[i + 1]) * 100.0f / 255.0f;
        }
        // Progress owns this payload through std::any; covered by ownership regression tests.
        // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
        return common::Progress::complete(SpeedData{Speed::from_array(values), {}});
    }
    inline common::TransactionPlan read_speed(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0x05;
        common::TransactionPlan plan;
        plan.domain = "speed";
        plan.keys = {"speed"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_speed(id, input); };
        return plan;
    }
    inline common::TransactionPlan write_speed(uint32_t id, const std::array<uint8_t, Speed::kJointCount> &raw) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        message.data[0] = 0x05;
        for (size_t i = 0; i < raw.size(); ++i) {
            message.data[i + 1] = raw[i];
        }
        common::TransactionPlan plan;
        plan.keys = {"speed"};
        plan.frames = {message};
        return plan;
    }

    // Acceleration raw is nonlinear: 0 -> 100%, 1-254 -> 0-99%.
    inline common::Progress decode_acceleration(uint32_t id, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 7 ||
            message.data[0] != 0x87) {
            return common::Progress::unmatched();
        }
        std::array<float, Acceleration::kJointCount> values{};
        for (size_t i = 0; i < values.size(); ++i) {
            const uint8_t raw = message.data[i + 1];
            values[i] = raw == 0 ? 100.0f : static_cast<float>(raw - 1) * 99.0f / 253.0f;
        }
        // Progress owns this payload through std::any; covered by ownership regression tests.
        // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
        return common::Progress::complete(AccelerationData{Acceleration::from_array(values), {}});
    }
    inline common::TransactionPlan read_acceleration(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0x87;
        common::TransactionPlan plan;
        plan.domain = "acceleration";
        plan.keys = {"acceleration"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_acceleration(id, input); };
        return plan;
    }
    inline common::TransactionPlan write_acceleration(uint32_t id,
                                                      const std::array<uint8_t, Acceleration::kJointCount> &raw) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        message.data[0] = 0x87;
        for (size_t i = 0; i < raw.size(); ++i) {
            message.data[i + 1] = raw[i];
        }
        common::TransactionPlan plan;
        plan.keys = {"acceleration"};
        plan.frames = {message};
        return plan;
    }

    // --- sense-only (no write) ---

    inline common::Progress decode_temperature(uint32_t id, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 7 ||
            message.data[0] != 0x33) {
            return common::Progress::unmatched();
        }
        std::array<float, Temperature::kJointCount> values{};
        for (size_t i = 0; i < values.size(); ++i) {
            values[i] = static_cast<float>(message.data[i + 1]);
        }
        // Progress owns this payload through std::any; covered by ownership regression tests.
        // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
        return common::Progress::complete(TemperatureData{Temperature::from_array(values), {}});
    }
    inline common::TransactionPlan read_temperature(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0x33;
        common::TransactionPlan plan;
        plan.domain = "temperature";
        plan.keys = {"temperature"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_temperature(id, input); };
        return plan;
    }

    inline common::Progress decode_fault(uint32_t id, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 7 ||
            message.data[0] != 0x35) {
            return common::Progress::unmatched();
        }
        std::array<uint8_t, Fault::kJointCount> raw{};
        for (size_t i = 0; i < raw.size(); ++i) {
            raw[i] = message.data[i + 1];
        }
        // Progress owns this payload through std::any; covered by ownership regression tests.
        // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
        return common::Progress::complete(FaultData{Fault::from_raw(raw), {}});
    }
    inline common::TransactionPlan read_fault(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0x35;
        common::TransactionPlan plan;
        plan.domain = "fault";
        plan.keys = {"fault"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_fault(id, input); };
        return plan;
    }

    // --- stall: threshold 0xC5, time 0xC6, torque 0xC7, sharing this dlc=7 pattern ---

    inline common::Progress decode_stall_time(uint32_t id, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 7 ||
            message.data[0] != 0xC6) {
            return common::Progress::unmatched();
        }
        std::array<float, StallTime::kJointCount> values{};
        for (size_t i = 0; i < values.size(); ++i) {
            values[i] = static_cast<float>(message.data[i + 1]) * 10.0f;
        }
        // Progress owns this payload through std::any; covered by ownership regression tests.
        // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
        return common::Progress::complete(StallTimeData{StallTime::from_array(values), {}});
    }
    inline common::TransactionPlan read_stall_time(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0xC6;
        common::TransactionPlan plan;
        plan.domain = "stall_time";
        plan.keys = {"stall_time"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_stall_time(id, input); };
        return plan;
    }
    inline common::TransactionPlan write_stall_time(uint32_t id,
                                                    const std::array<uint8_t, StallTime::kJointCount> &raw) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        message.data[0] = 0xC6;
        for (size_t i = 0; i < raw.size(); ++i) {
            message.data[i + 1] = raw[i];
        }
        common::TransactionPlan plan;
        plan.keys = {"stall_time"};
        plan.frames = {message};
        return plan;
    }

    inline common::Progress decode_stall_threshold(uint32_t id, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 7 ||
            message.data[0] != 0xC5) {
            return common::Progress::unmatched();
        }
        std::array<float, StallLimit::kJointCount> values{};
        for (size_t i = 0; i < values.size(); ++i) {
            values[i] = static_cast<float>(message.data[i + 1]) * 1000.0f / 255.0f;
        }
        // Progress owns this payload through std::any; covered by ownership regression tests.
        // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
        return common::Progress::complete(StallLimitData{StallLimit::from_array(values), {}});
    }
    inline common::TransactionPlan read_stall_threshold(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0xC5;
        common::TransactionPlan plan;
        plan.domain = "stall_threshold";
        plan.keys = {"stall_threshold"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_stall_threshold(id, input); };
        return plan;
    }
    inline common::TransactionPlan write_stall_threshold(uint32_t id,
                                                         const std::array<uint8_t, StallLimit::kJointCount> &raw) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        message.data[0] = 0xC5;
        for (size_t i = 0; i < raw.size(); ++i) {
            message.data[i + 1] = raw[i];
        }
        common::TransactionPlan plan;
        plan.keys = {"stall_threshold"};
        plan.frames = {message};
        return plan;
    }

    inline common::Progress decode_stall_torque(uint32_t id, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 7 ||
            message.data[0] != 0xC7) {
            return common::Progress::unmatched();
        }
        std::array<float, StallLimit::kJointCount> values{};
        for (size_t i = 0; i < values.size(); ++i) {
            values[i] = static_cast<float>(message.data[i + 1]) * 1000.0f / 255.0f;
        }
        // Progress owns this payload through std::any; covered by ownership regression tests.
        // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
        return common::Progress::complete(StallLimitData{StallLimit::from_array(values), {}});
    }
    inline common::TransactionPlan read_stall_torque(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0xC7;
        common::TransactionPlan plan;
        plan.domain = "stall_torque";
        plan.keys = {"stall_torque"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_stall_torque(id, input); };
        return plan;
    }
    inline common::TransactionPlan write_stall_torque(uint32_t id,
                                                      const std::array<uint8_t, StallLimit::kJointCount> &raw) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        message.data[0] = 0xC7;
        for (size_t i = 0; i < raw.size(); ++i) {
            message.data[i + 1] = raw[i];
        }
        common::TransactionPlan plan;
        plan.keys = {"stall_torque"};
        plan.frames = {message};
        return plan;
    }

    // --- config: fire-and-forget writes, no response consumption ---

    namespace argument_detail {
        // Keep on-wire identifiers, selectors and lengths in one named record.
        struct WriteConfigSetIdArguments {
            uint32_t id;
            uint8_t subcommand;
            uint8_t target;
        };
        inline common::TransactionPlan write_config_set_id(const WriteConfigSetIdArguments &arguments) {
            can::Message message{};
            message.format = can::FrameFormat::kClassic;
            message.id = arguments.id;
            message.dlc = 3;
            message.data[0] = 0xC3;
            message.data[1] = arguments.subcommand;
            message.data[2] = arguments.target;
            common::TransactionPlan plan;
            plan.keys = {"config"};
            plan.frames = {message};
            return plan;
        }
    }
    inline common::TransactionPlan write_config_set_id(uint32_t id, uint8_t subcommand, uint8_t target) {
        return argument_detail::write_config_set_id({id, subcommand, target});
    }
    inline common::TransactionPlan write_config_restore_factory(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 8;
        for (auto &byte : message.data) {
            byte = 0xCE;
        }
        common::TransactionPlan plan;
        plan.keys = {"config"};
        plan.frames = {message};
        return plan;
    }
    inline common::TransactionPlan write_config_save(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 8;
        for (auto &byte : message.data) {
            byte = 0xCF;
        }
        common::TransactionPlan plan;
        plan.keys = {"config"};
        plan.frames = {message};
        return plan;
    }

    // --- force sensor: per-finger 10-frame dot-matrix assembler (transaction-local) ---

    inline common::TransactionPlan read_force_sensor(uint32_t id, uint8_t prefix, const std::string &domain) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 2;
        message.data[0] = prefix;
        message.data[1] = 0xA4;
        common::TransactionPlan plan;
        plan.domain = domain;
        plan.exclusion_groups = {"o6.tactile"};
        plan.keys = {domain};
        plan.frames = {message};
        plan.consume = [id, prefix, rows = std::array<std::array<uint8_t, ForceSensorData::kBytesPerFrame>, 16>{},
                        seen = std::array<bool, 16>{}, count = size_t{0}](const can::Message &input) mutable {
            if (input.id != id || input.is_extended || input.is_fd || input.is_brs || !valid_force_sensor_row(input) ||
                input.data[0] != prefix) {
                return common::Progress::unmatched();
            }
            const uint8_t row = input.data[1] >> 4;
            if ((input.data[1] & 0x0f) != 0) {
                return common::Progress::unmatched();
            }
            std::array<uint8_t, ForceSensorData::kBytesPerFrame> incoming{};
            for (size_t j = 0; j < ForceSensorData::kBytesPerFrame; ++j) {
                incoming[j] = input.data[2 + j];
            }

            uint64_t duplicates = 0;
            if (seen[row]) {
                if (rows[row] == incoming) {
                    duplicates = 1;
                } else {
                    common::Progress failure;
                    failure.kind = common::ProgressKind::failed;
                    failure.conflicting_duplicates = 1;
                    failure.terminal_error = false;
                    failure.error = std::make_exception_ptr(::linkerbot::hand::ProtocolError(
                        "force sensor prefix=" + std::to_string(prefix) +
                        " row=" + std::to_string(static_cast<int>(row)) + " conflicting duplicate"));
                    return failure;
                }
            } else {
                rows[row] = incoming;
                seen[row] = true;
                ++count;
            }

            if (count < ForceSensorData::kFrameCount) {
                auto progress = common::Progress::pending();
                progress.duplicates = duplicates;
                return progress;
            }
            auto data = compact_force_sensor_rows(rows, seen);
            auto progress = common::Progress::complete(data);
            progress.duplicates = duplicates;
            return progress;
        };
        return plan;
    }

    // --- force sensor: 5-finger aggregate, one transaction, transaction-local assembler ---

    inline common::TransactionPlan read_force_sensor_all(uint32_t id) {
        static constexpr std::array<uint8_t, 5> kPrefixes{0xB1, 0xB2, 0xB3, 0xB4, 0xB5};
        common::TransactionPlan plan;
        plan.domain = "force_sensor_all";
        plan.exclusion_groups = {"o6.tactile"};
        plan.preserve_last_success = true;
        plan.publish_after_send = true;
        plan.keys = {"force_sensor_thumb", "force_sensor_index", "force_sensor_middle", "force_sensor_ring",
                     "force_sensor_pinky"};
        plan.frames.reserve(kPrefixes.size());
        for (uint8_t prefix : kPrefixes) {
            can::Message message{};
            message.format = can::FrameFormat::kClassic;
            message.id = id;
            message.dlc = 2;
            message.data[0] = prefix;
            message.data[1] = 0xA4;
            plan.frames.push_back(message);
        }
        plan.consume = [id,
                        rows = std::array<std::array<std::array<uint8_t, ForceSensorData::kBytesPerFrame>, 16>, 5>{},
                        seen = std::array<std::array<bool, 16>, 5>{}, counts = std::array<size_t, 5>{},
                        published = std::array<bool, 5>{}](const can::Message &input) mutable {
            if (input.id != id || input.is_extended || input.is_fd || input.is_brs || !valid_force_sensor_row(input)) {
                return common::Progress::unmatched();
            }
            size_t finger = kPrefixes.size();
            for (size_t i = 0; i < kPrefixes.size(); ++i) {
                if (input.data[0] == kPrefixes[i]) {
                    finger = i;
                    break;
                }
            }
            if (finger == kPrefixes.size()) {
                return common::Progress::unmatched();
            }
            const uint8_t row = input.data[1] >> 4;
            if ((input.data[1] & 0x0f) != 0) {
                return common::Progress::unmatched();
            }
            std::array<uint8_t, ForceSensorData::kBytesPerFrame> incoming{};
            for (size_t j = 0; j < ForceSensorData::kBytesPerFrame; ++j) {
                incoming[j] = input.data[2 + j];
            }

            uint64_t duplicates = 0;
            if (seen[finger][row]) {
                if (rows[finger][row] == incoming) {
                    duplicates = 1;
                } else {
                    common::Progress failure;
                    failure.kind = common::ProgressKind::failed;
                    failure.conflicting_duplicates = 1;
                    failure.terminal_error = false;
                    failure.error = std::make_exception_ptr(::linkerbot::hand::ProtocolError(
                        "force sensor batch finger=" + std::to_string(finger) +
                        " row=" + std::to_string(static_cast<int>(row)) + " conflicting duplicate"));
                    return failure;
                }
            } else {
                rows[finger][row] = incoming;
                seen[finger][row] = true;
                ++counts[finger];
            }

            auto progress = common::Progress::pending();
            progress.duplicates = duplicates;
            const bool complete_finger = counts[finger] == ForceSensorData::kFrameCount;
            if (complete_finger && !published[finger]) {
                static constexpr const char *domains[]{"force_sensor_thumb", "force_sensor_index",
                                                       "force_sensor_middle", "force_sensor_ring",
                                                       "force_sensor_pinky"};
                auto value = compact_force_sensor_rows(rows[finger], seen[finger]);
                progress.updates.emplace_back(domains[finger], value);
                published[finger] = true;
            }
            for (size_t count : counts) {
                if (count != ForceSensorData::kFrameCount) {
                    return progress;
                }
            }
            AllFingersData data;
            data.thumb = compact_force_sensor_rows(rows[0], seen[0]);
            data.index = compact_force_sensor_rows(rows[1], seen[1]);
            data.middle = compact_force_sensor_rows(rows[2], seen[2]);
            data.ring = compact_force_sensor_rows(rows[3], seen[3]);
            data.pinky = compact_force_sensor_rows(rows[4], seen[4]);
            progress.kind = common::ProgressKind::complete;
            progress.value = data;
            return progress;
        };
        return plan;
    }

    // --- version: SN is a 4-frame ASCII assembler (transaction-local); others are single-frame ---

    inline common::TransactionPlan read_version_sn(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0xC0;
        common::TransactionPlan plan;
        plan.domain = "version_sn";
        plan.keys = {"version_sn"};
        plan.frames = {message};
        plan.consume = [id, buf = std::array<std::array<char, 6>, 4>{},
                        seen = std::array<bool, 4>{}](const can::Message &input) mutable {
            if (input.id != id || input.is_extended || input.is_fd || input.is_brs || input.dlc < 3 || input.dlc > 8 ||
                input.data[0] != 0xC0) {
                return common::Progress::unmatched();
            }
            const uint8_t key = input.data[1];
            // Firmware may omit trailing padding in the final indexed fragment.
            if (key >= 4 || (key != 3 && input.dlc != 8)) {
                return common::Progress::unmatched();
            }
            std::array<char, 6> incoming{};
            for (size_t i = 0; i < input.dlc - 2U; ++i) {
                incoming[i] = static_cast<char>(input.data[2 + i]);
            }

            uint64_t duplicates = 0;
            if (seen[key]) {
                if (buf[key] == incoming) {
                    duplicates = 1;
                } else {
                    common::Progress failure;
                    failure.kind = common::ProgressKind::failed;
                    failure.conflicting_duplicates = 1;
                    failure.terminal_error = false;
                    failure.error = std::make_exception_ptr(::linkerbot::hand::ProtocolError(
                        "version_sn key=" + std::to_string(key) + " conflicting duplicate"));
                    return failure;
                }
            } else {
                buf[key] = incoming;
                seen[key] = true;
            }

            for (bool s : seen) {
                if (!s) {
                    auto progress = common::Progress::pending();
                    progress.duplicates = duplicates;
                    return progress;
                }
            }
            std::string sn;
            for (const auto &slot : buf) {
                sn.append(slot.data(), slot.size());
            }
            while (!sn.empty() && sn.back() == '\0') {
                sn.pop_back();
            }
            auto progress = common::Progress::complete(sn);
            progress.duplicates = duplicates;
            return progress;
        };
        return plan;
    }
    inline common::Progress decode_version_single(uint32_t id, uint8_t cmd, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 4 ||
            message.data[0] != cmd) {
            return common::Progress::unmatched();
        }
        return common::Progress::complete(Version{message.data[1], message.data[2], message.data[3]});
    }
    inline common::TransactionPlan read_version_pcb(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0xC1;
        common::TransactionPlan plan;
        plan.domain = "version_pcb";
        plan.keys = {"version_pcb"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_version_single(id, 0xC1, input); };
        return plan;
    }
    inline common::TransactionPlan read_version_firmware(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0xC2;
        common::TransactionPlan plan;
        plan.domain = "version_firmware";
        plan.keys = {"version_firmware"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_version_single(id, 0xC2, input); };
        return plan;
    }
    inline common::TransactionPlan read_version_mechanical(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = 0xC4;
        common::TransactionPlan plan;
        plan.domain = "version_mechanical";
        plan.keys = {"version_mechanical"};
        plan.frames = {message};
        plan.consume = [id](const can::Message &input) { return decode_version_single(id, 0xC4, input); };
        return plan;
    }

    class Backend final : public common::ProtocolBackend {
    public:
        explicit Backend(uint32_t id) : id_(id) {}
        common::TransactionPlan make_read(const std::string &operation, const std::any & = {}) const override {
            if (operation == "angle") {
                return read_angle(id_);
            }
            if (operation == "current") {
                return read_current(id_);
            }
            if (operation == "speed") {
                return read_speed(id_);
            }
            if (operation == "acceleration") {
                return read_acceleration(id_);
            }
            if (operation == "temperature") {
                return read_temperature(id_);
            }
            if (operation == "fault") {
                return read_fault(id_);
            }
            if (operation == "stall_time") {
                return read_stall_time(id_);
            }
            if (operation == "stall_threshold") {
                return read_stall_threshold(id_);
            }
            if (operation == "stall_torque") {
                return read_stall_torque(id_);
            }
            if (operation == "force_sensor_thumb") {
                return read_force_sensor(id_, 0xB1, "force_sensor_thumb");
            }
            if (operation == "force_sensor_index") {
                return read_force_sensor(id_, 0xB2, "force_sensor_index");
            }
            if (operation == "force_sensor_middle") {
                return read_force_sensor(id_, 0xB3, "force_sensor_middle");
            }
            if (operation == "force_sensor_ring") {
                return read_force_sensor(id_, 0xB4, "force_sensor_ring");
            }
            if (operation == "force_sensor_pinky") {
                return read_force_sensor(id_, 0xB5, "force_sensor_pinky");
            }
            if (operation == "force_sensor_all") {
                return read_force_sensor_all(id_);
            }
            if (operation == "version_sn") {
                return read_version_sn(id_);
            }
            if (operation == "version_pcb") {
                return read_version_pcb(id_);
            }
            if (operation == "version_firmware") {
                return read_version_firmware(id_);
            }
            if (operation == "version_mechanical") {
                return read_version_mechanical(id_);
            }
            throw ValidationError("unknown O6 protocol operation: " + operation);
        }
        common::TransactionPlan make_command(const std::string &operation,
                                             const std::any &argument = {}) const override {
            if (operation == "angle") {
                return write_angle(id_, std::any_cast<std::array<uint8_t, Angle::kJointCount>>(argument));
            }
            if (operation == "current_limit") {
                return write_current_limit(id_, std::any_cast<std::array<uint8_t, kCurrentJointCount>>(argument));
            }
            if (operation == "speed") {
                return write_speed(id_, std::any_cast<std::array<uint8_t, Speed::kJointCount>>(argument));
            }
            if (operation == "acceleration") {
                return write_acceleration(id_, std::any_cast<std::array<uint8_t, Acceleration::kJointCount>>(argument));
            }
            if (operation == "stall_time") {
                return write_stall_time(id_, std::any_cast<std::array<uint8_t, StallTime::kJointCount>>(argument));
            }
            if (operation == "stall_threshold") {
                return write_stall_threshold(id_,
                                             std::any_cast<std::array<uint8_t, StallLimit::kJointCount>>(argument));
            }
            if (operation == "stall_torque") {
                return write_stall_torque(id_, std::any_cast<std::array<uint8_t, StallLimit::kJointCount>>(argument));
            }
            if (operation == "config_set_id") {
                return write_config_set_id(id_, 0x01, std::any_cast<uint8_t>(argument));
            }
            if (operation == "config_set_device_id") {
                return write_config_set_id(id_, 0x02, std::any_cast<uint8_t>(argument));
            }
            if (operation == "config_set_transmit_id") {
                return write_config_set_id(id_, 0x03, std::any_cast<uint8_t>(argument));
            }
            if (operation == "config_restore_factory") {
                return write_config_restore_factory(id_);
            }
            if (operation == "config_save") {
                return write_config_save(id_);
            }
            throw ValidationError("unknown O6 protocol operation: " + operation);
        }

    private:
        uint32_t id_;
    };
} // namespace linkerbot::hand::o6::protocol
