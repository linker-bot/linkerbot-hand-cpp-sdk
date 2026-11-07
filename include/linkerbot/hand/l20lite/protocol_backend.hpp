#include <linkerbot/hand/errors.hpp>
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <linkerbot/hand/l20lite/acceleration.hpp>
#include <linkerbot/hand/l20lite/angle.hpp>
#include <linkerbot/hand/l20lite/config.hpp>
#include <linkerbot/hand/l20lite/current.hpp>
#include <linkerbot/hand/l20lite/fault.hpp>
#include <linkerbot/hand/l20lite/force_sensor.hpp>
#include <linkerbot/hand/l20lite/frame_layout.hpp>
#include <linkerbot/hand/l20lite/joints.hpp>
#include <linkerbot/hand/l20lite/speed.hpp>
#include <linkerbot/hand/l20lite/stall.hpp>
#include <linkerbot/hand/l20lite/temperature.hpp>
#include <linkerbot/hand/l20lite/tip_force.hpp>
#include <linkerbot/hand/l20lite/version.hpp>
#include <linkerbot/hand/protocol/transaction.hpp>
#include <optional>
#include <stdexcept>
#include <string>

namespace linkerbot::hand::l20lite::protocol {
    namespace common = ::linkerbot::hand::protocol;

    // --- dual-frame (angle/current/speed/acceleration/temperature/fault) ---
    // Each domain is one on-wire request split across two physical CAN frames
    // (FramePart A/B from frame_layout.hpp). A single TransactionPlan carries both
    // outgoing frames; the transaction-local consume lambda assembles both replies
    // before completing, mirroring how force_sensor/version_sn assemble multiple
    // incoming frames on O6.

    // Each transaction owns its assembly state; no data crosses request boundaries.
    inline std::function<common::Progress(const can::Message &)>
    make_dual_frame_decoder(uint32_t id, FramePart part_a, FramePart part_b,
                            std::function<common::Progress(const std::array<uint8_t, kJointCount> &)> finish) {
        return [id, part_a = std::move(part_a), part_b = std::move(part_b), finish = std::move(finish),
                raw = std::array<uint8_t, kJointCount>{},
                have = std::array<bool, 2>{}](const can::Message &input) mutable {
            if (input.id != id || input.is_extended || input.is_fd || input.is_brs) {
                return common::Progress::unmatched();
            }
            int idx = -1;
            const FramePart *part = nullptr;
            if (input.data[0] == part_a.cmd) {
                idx = 0;
                part = &part_a;
            } else if (input.data[0] == part_b.cmd) {
                idx = 1;
                part = &part_b;
            } else {
                return common::Progress::unmatched();
            }
            if (input.dlc != 1 + part->count) {
                return common::Progress::unmatched();
            }

            std::array<uint8_t, kJointCount> incoming = raw;
            for (size_t i = 0; i < part->count; ++i) {
                incoming[part->joints[i]] = input.data[i + 1];
            }

            uint64_t duplicates = 0;
            if (have[static_cast<size_t>(idx)]) {
                bool matches = true;
                for (size_t i = 0; i < part->count; ++i) {
                    if (incoming[part->joints[i]] != raw[part->joints[i]]) {
                        matches = false;
                        break;
                    }
                }
                if (matches) {
                    duplicates = 1;
                } else {
                    common::Progress failure;
                    failure.kind = common::ProgressKind::failed;
                    failure.conflicting_duplicates = 1;
                    failure.terminal_error = false;
                    failure.error = std::make_exception_ptr(::linkerbot::hand::ProtocolError(
                        "dual-frame part " + std::to_string(static_cast<int>(part->cmd)) + " conflicting duplicate"));
                    return failure;
                }
            } else {
                raw = incoming;
                have[static_cast<size_t>(idx)] = true;
            }

            if (!have[0] || !have[1]) {
                auto progress = common::Progress::pending();
                progress.duplicates = duplicates;
                return progress;
            }
            have[0] = false;
            have[1] = false;
            auto progress = finish(raw);
            progress.duplicates = duplicates;
            return progress;
        };
    }

    inline common::TransactionPlan
    read_dual_frame(uint32_t id, FramePart part_a, FramePart part_b, std::string domain, std::chrono::microseconds gap,
                    std::function<common::Progress(const std::array<uint8_t, kJointCount> &)> finish) {
        can::Message message_a{};
        message_a.format = can::FrameFormat::kClassic;
        message_a.id = id;
        message_a.dlc = 1;
        message_a.data[0] = part_a.cmd;
        can::Message message_b{};
        message_b.format = can::FrameFormat::kClassic;
        message_b.id = id;
        message_b.dlc = 1;
        message_b.data[0] = part_b.cmd;
        common::TransactionPlan plan;
        plan.domain = domain;
        plan.keys = {domain};
        plan.frames = {message_a, message_b};
        plan.gap = gap;
        plan.initial_delay = gap;
        plan.consume = make_dual_frame_decoder(id, std::move(part_a), std::move(part_b), std::move(finish));
        return plan;
    }

    inline common::TransactionPlan write_dual_frame(uint32_t id, const FramePart &part_a, const FramePart &part_b,
                                                    const std::string &domain, std::chrono::microseconds gap,
                                                    const std::array<uint8_t, kJointCount> &raw) {
        can::Message message_a{};
        message_a.format = can::FrameFormat::kClassic;
        message_a.id = id;
        message_a.dlc = static_cast<uint8_t>(1 + part_a.count);
        message_a.data[0] = part_a.cmd;
        for (size_t i = 0; i < part_a.count; ++i) {
            message_a.data[i + 1] = raw[part_a.joints[i]];
        }
        can::Message message_b{};
        message_b.format = can::FrameFormat::kClassic;
        message_b.id = id;
        message_b.dlc = static_cast<uint8_t>(1 + part_b.count);
        message_b.data[0] = part_b.cmd;
        for (size_t i = 0; i < part_b.count; ++i) {
            message_b.data[i + 1] = raw[part_b.joints[i]];
        }
        common::TransactionPlan plan;
        plan.domain = domain;
        plan.keys = {domain};
        plan.frames = {message_a, message_b};
        plan.gap = gap;
        plan.initial_delay = gap;
        return plan;
    }

    inline common::Progress finish_angle(const std::array<uint8_t, kJointCount> &raw) {
        return common::Progress::complete(AngleData{Angle::from_array(raw_to_percents(raw)), {}});
    }
    inline common::TransactionPlan read_angle(uint32_t id) {
        return read_dual_frame(id, kAngleFrameA, kAngleFrameB, "angle", kAngleInterFrameDelay, finish_angle);
    }
    inline common::TransactionPlan write_angle(uint32_t id, const std::array<uint8_t, kJointCount> &raw) {
        return write_dual_frame(id, kAngleFrameA, kAngleFrameB, "angle", kAngleInterFrameDelay, raw);
    }

    inline common::Progress finish_current(const std::array<uint8_t, kJointCount> &raw) {
        return common::Progress::complete(CurrentData{raw, {}});
    }
    inline common::TransactionPlan read_current(uint32_t id) {
        auto plan =
            read_dual_frame(id, kCurrentFrameA, kCurrentFrameB, "current", kVectorInterFrameDelay, finish_current);
        plan.keys = {"l20lite.0x02-0x03"};
        return plan;
    }
    inline common::Progress finish_current_limit(const std::array<uint8_t, kJointCount> &raw) {
        return common::Progress::complete(CurrentLimitData{raw, {}});
    }
    inline common::TransactionPlan write_current_limit(uint32_t id, const std::array<uint8_t, kJointCount> &raw) {
        auto plan = write_dual_frame(id, kCurrentFrameA, kCurrentFrameB, "current_limit", kVectorInterFrameDelay, raw);
        plan.command_family = "current_limits";
        plan.keys = {"l20lite.0x02-0x03"};
        plan.consume = read_dual_frame(id, kCurrentFrameA, kCurrentFrameB, "current_limit", kVectorInterFrameDelay,
                                       finish_current_limit)
                           .consume;
        plan.publish_after_send = true;
        auto decode = std::move(plan.consume);
        plan.consume = [decode = std::move(decode)](const can::Message &message) mutable {
            auto result = decode(message);
            result.origin = common::Origin::command_feedback;
            return result;
        };
        return plan;
    }

    inline common::Progress finish_speed(const std::array<uint8_t, kJointCount> &raw) {
        return common::Progress::complete(SpeedData{Speed::from_array(raw_to_percents(raw)), {}});
    }
    inline common::TransactionPlan read_speed(uint32_t id) {
        return read_dual_frame(id, kSpeedFrameA, kSpeedFrameB, "speed", kVectorInterFrameDelay, finish_speed);
    }
    inline common::TransactionPlan write_speed(uint32_t id, const std::array<uint8_t, kJointCount> &raw) {
        return write_dual_frame(id, kSpeedFrameA, kSpeedFrameB, "speed", kVectorInterFrameDelay, raw);
    }

    // Acceleration raw is linear (new protocol 0x87/0x88): 0-254 <-> 0-100%.
    inline common::Progress finish_acceleration(const std::array<uint8_t, kJointCount> &raw) {
        return common::Progress::complete(AccelerationData{Acceleration::from_array(accel_raw_to_percents(raw)), {}});
    }
    inline common::TransactionPlan read_acceleration(uint32_t id) {
        return read_dual_frame(id, kAccelFrameA, kAccelFrameB, "acceleration", kVectorInterFrameDelay,
                               finish_acceleration);
    }
    inline common::TransactionPlan write_acceleration(uint32_t id, const std::array<uint8_t, kJointCount> &raw) {
        return write_dual_frame(id, kAccelFrameA, kAccelFrameB, "acceleration", kVectorInterFrameDelay, raw);
    }

    // --- sense-only (no write) ---

    inline common::Progress finish_temperature(const std::array<uint8_t, kJointCount> &raw) {
        std::array<float, kJointCount> values{};
        for (size_t i = 0; i < kJointCount; ++i) {
            values[i] = static_cast<float>(raw[i]);
        }
        return common::Progress::complete(TemperatureData{Temperature::from_array(values), {}});
    }
    inline common::TransactionPlan read_temperature(uint32_t id) {
        return read_dual_frame(id, kTempFrameA, kTempFrameB, "temperature", kVectorInterFrameDelay, finish_temperature);
    }

    inline common::Progress finish_fault(const std::array<uint8_t, kJointCount> &raw) {
        return common::Progress::complete(FaultData{Fault::from_raw(raw), {}});
    }
    inline common::TransactionPlan read_fault(uint32_t id) {
        return read_dual_frame(id, kFaultFrameA, kFaultFrameB, "fault", kVectorInterFrameDelay, finish_fault);
    }

    // --- stall: threshold 0xC5, time 0xC6, torque 0xC7, two 5-joint banks per
    // command. Firmware only echoes 7-byte bank writes, so a "read" resends the
    // caller's last-set raw bytes (owned by the manager) and decodes the echo. ---

    inline constexpr uint8_t kStallThresholdCmd = 0xC5;
    inline constexpr uint8_t kStallTimeCmd = 0xC6;
    inline constexpr uint8_t kStallTorqueCmd = 0xC7;

    inline uint8_t stall_ms_to_raw(float ms) {
        if (ms <= 0.0f) {
            return 0;
        }
        const auto rounded = std::lround(static_cast<double>(ms) / 10.0);
        if (rounded > 255) {
            return 255;
        }
        return static_cast<uint8_t>(rounded);
    }
    inline float stall_raw_to_ms(uint8_t raw) { return static_cast<float>(raw) * 10.0f; }

    inline std::array<uint8_t, kJointCount> stall_times_to_raw(const StallTime &times) {
        auto arr = times.to_array();
        std::array<uint8_t, kJointCount> raw{};
        for (size_t i = 0; i < kJointCount; ++i) {
            raw[i] = stall_ms_to_raw(arr[i]);
        }
        return raw;
    }
    inline StallTime stall_times_from_raw(const std::array<uint8_t, kJointCount> &raw) {
        std::array<float, kJointCount> values{};
        for (size_t i = 0; i < kJointCount; ++i) {
            values[i] = stall_raw_to_ms(raw[i]);
        }
        return StallTime::from_array(values);
    }

    namespace argument_detail {
        // Keep on-wire identifiers, selectors and lengths in one named record.
        struct BuildStallBankFramesArguments {
            uint32_t id;
            uint8_t cmd;
            const std::array<uint8_t, kJointCount> &raw;
        };
        inline std::array<can::Message, 2> build_stall_bank_frames(const BuildStallBankFramesArguments &arguments) {
            can::Message bank1{};
            bank1.format = can::FrameFormat::kClassic;
            bank1.id = arguments.id;
            bank1.dlc = 7;
            bank1.data[0] = arguments.cmd;
            bank1.data[1] = 1;
            for (size_t i = 0; i < 5; ++i) {
                bank1.data[i + 2] = arguments.raw[i];
            }
            can::Message bank2{};
            bank2.format = can::FrameFormat::kClassic;
            bank2.id = arguments.id;
            bank2.dlc = 7;
            bank2.data[0] = arguments.cmd;
            bank2.data[1] = 2;
            for (size_t i = 0; i < 5; ++i) {
                bank2.data[i + 2] = arguments.raw[i + 5];
            }
            return {bank1, bank2};
        }
    }
    inline std::array<can::Message, 2> build_stall_bank_frames(uint32_t id, uint8_t cmd,
                                                               const std::array<uint8_t, kJointCount> &raw) {
        return argument_detail::build_stall_bank_frames({id, cmd, raw});
    }

    inline std::function<common::Progress(const can::Message &)> make_stall_bank_decoder(uint32_t id, uint8_t cmd) {
        return [id, cmd, bank1 = std::optional<std::array<uint8_t, 5>>{},
                bank2 = std::optional<std::array<uint8_t, 5>>{}](const can::Message &input) mutable {
            if (input.id != id || input.is_extended || input.is_fd || input.is_brs || input.dlc != 7 ||
                input.data[0] != cmd) {
                return common::Progress::unmatched();
            }
            const uint8_t bank = input.data[1];
            if (bank != 1 && bank != 2) {
                return common::Progress::unmatched();
            }
            std::array<uint8_t, 5> payload{};
            for (size_t i = 0; i < 5; ++i) {
                payload[i] = input.data[i + 2];
            }

            auto &slot = (bank == 1 ? bank1 : bank2);
            uint64_t duplicates = 0;
            if (slot) {
                if (*slot == payload) {
                    duplicates = 1;
                } else {
                    common::Progress failure;
                    failure.kind = common::ProgressKind::failed;
                    failure.conflicting_duplicates = 1;
                    failure.terminal_error = false;
                    failure.error = std::make_exception_ptr(::linkerbot::hand::ProtocolError(
                        "stall bank " + std::to_string(static_cast<int>(bank)) + " conflicting duplicate"));
                    return failure;
                }
            } else {
                slot = payload;
            }

            if (!bank1 || !bank2) {
                auto progress = common::Progress::pending();
                progress.duplicates = duplicates;
                return progress;
            }
            std::array<uint8_t, kJointCount> raw{};
            for (size_t i = 0; i < 5; ++i) {
                raw[i] = (*bank1)[i];
                raw[i + 5] = (*bank2)[i];
            }
            common::Progress progress =
                (cmd == kStallTimeCmd)
                    ? common::Progress::complete(StallTimeData{stall_times_from_raw(raw), {}})
                    : common::Progress::complete(StallLimitData{StallLimit::from_array(raw_to_percents(raw)), {}});
            progress.duplicates = duplicates;
            return progress;
        };
    }

    inline common::TransactionPlan write_stall_bank(uint32_t id, uint8_t cmd, const std::string &domain,
                                                    const std::array<uint8_t, kJointCount> &raw) {
        const auto frames = build_stall_bank_frames(id, cmd, raw);
        common::TransactionPlan plan;
        plan.domain = domain;
        plan.keys = {domain};
        plan.frames = {frames[0], frames[1]};
        plan.command_family = domain;
        plan.publish_after_send = true;
        plan.preserve_last_success = true;
        plan.consume = [decode = make_stall_bank_decoder(id, cmd)](const can::Message &input) mutable {
            auto progress = decode(input);
            progress.origin = common::Origin::command_feedback;
            return progress;
        };
        return plan;
    }

    // Firmware has no read-only stall query. Retain the legacy entry point but
    // reject it rather than silently replaying a configuration write.
    inline common::TransactionPlan read_stall_bank(uint32_t, uint8_t, const std::string &,
                                                   const std::array<uint8_t, kJointCount> &) {
        throw UnsupportedOperationError("L20Lite stall parameters are write-response cache only");
    }

    // --- force sensor: per-finger 12-row dot-matrix assembler (transaction-local).
    // Request byte is 0xC6 (as data[1] of a dlc=2 request), distinct from the
    // stall time command 0xC6 (which appears as data[0] of a dlc=7 message). ---

    inline common::TransactionPlan read_force_sensor(uint32_t id, uint8_t prefix, const std::string &domain) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 2;
        message.data[0] = prefix;
        message.data[1] = 0xC6;
        common::TransactionPlan plan;
        plan.domain = domain;
        plan.exclusion_groups = {"l20lite.tactile"};
        plan.domain = domain;
        plan.keys = {domain};
        plan.frames = {message};
        plan.consume =
            [id, prefix,
             rows = std::array<std::array<uint8_t, ForceSensorData::kBytesPerFrame>, ForceSensorData::kFrameCount>{},
             seen = std::array<bool, ForceSensorData::kFrameCount>{}](const can::Message &input) mutable {
                if (input.id != id || input.is_extended || input.is_fd || input.is_brs || input.dlc != 8 ||
                    input.data[0] != prefix) {
                    return common::Progress::unmatched();
                }
                const uint8_t row = input.data[1] >> 4;
                if ((input.data[1] & 0x0f) != 0 || row >= ForceSensorData::kFrameCount) {
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
                }

                for (bool s : seen) {
                    if (!s) {
                        auto progress = common::Progress::pending();
                        progress.duplicates = duplicates;
                        return progress;
                    }
                }
                ForceSensorData data;
                data.values = rows;
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
        plan.gap = kForceInterRequestDelay;
        plan.exclusion_groups = {"l20lite.tactile"};
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
            message.data[1] = 0xC6;
            plan.frames.push_back(message);
        }
        plan.consume =
            [id,
             rows = std::array<
                 std::array<std::array<uint8_t, ForceSensorData::kBytesPerFrame>, ForceSensorData::kFrameCount>, 5>{},
             seen = std::array<std::array<bool, ForceSensorData::kFrameCount>, 5>{},
             published = std::array<bool, 5>{}](const can::Message &input) mutable {
                if (input.id != id || input.is_extended || input.is_fd || input.is_brs || input.dlc != 8) {
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
                if ((input.data[1] & 0x0f) != 0 || row >= ForceSensorData::kFrameCount) {
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
                }

                auto progress = common::Progress::pending();
                progress.duplicates = duplicates;
                const bool complete_finger =
                    std::all_of(seen[finger].begin(), seen[finger].end(), [](bool b) { return b; });
                if (complete_finger && !published[finger]) {
                    static constexpr const char *domains[]{"force_sensor_thumb", "force_sensor_index",
                                                           "force_sensor_middle", "force_sensor_ring",
                                                           "force_sensor_pinky"};
                    ForceSensorData value;
                    value.values = rows[finger];
                    progress.updates.emplace_back(domains[finger], value);
                    published[finger] = true;
                }
                for (const auto &finger_seen : seen) {
                    for (bool b : finger_seen) {
                        if (!b) {
                            return progress;
                        }
                    }
                }
                AllFingersData data;
                data.thumb.values = rows[0];
                data.index.values = rows[1];
                data.middle.values = rows[2];
                data.ring.values = rows[3];
                data.pinky.values = rows[4];
                progress.kind = common::ProgressKind::complete;
                progress.value = data;
                return progress;
            };
        return plan;
    }

    // --- version: SN is a byte-offset-keyed assembler needing only 3 of 4 frames
    // (offsets 0/6/12 mandatory, 18 optional); others are single-frame. ---

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
        plan.consume = [id, buf = std::array<uint8_t, 24>{},
                        seen = std::array<bool, 4>{}](const can::Message &input) mutable {
            if (input.id != id || input.is_extended || input.is_fd || input.is_brs || input.dlc != 8 ||
                input.data[0] != 0xC0) {
                return common::Progress::unmatched();
            }
            const uint8_t offset = input.data[1];
            if (offset != 0 && offset != 6 && offset != 12 && offset != 18) {
                return common::Progress::unmatched();
            }
            const size_t slot = static_cast<size_t>(offset / 6);
            const size_t available = 6;
            std::array<uint8_t, 6> incoming{};
            for (size_t i = 0; i < available; ++i) {
                incoming[i] = input.data[i + 2];
            }

            uint64_t duplicates = 0;
            if (seen[slot]) {
                bool matches = true;
                for (size_t i = 0; i < 6; ++i) {
                    if (buf[offset + i] != incoming[i]) {
                        matches = false;
                        break;
                    }
                }
                if (matches) {
                    duplicates = 1;
                } else {
                    common::Progress failure;
                    failure.kind = common::ProgressKind::failed;
                    failure.conflicting_duplicates = 1;
                    failure.terminal_error = false;
                    failure.error = std::make_exception_ptr(::linkerbot::hand::ProtocolError(
                        "version_sn offset=" + std::to_string(offset) + " conflicting duplicate"));
                    return failure;
                }
            } else {
                for (size_t i = 0; i < 6; ++i) {
                    buf[offset + i] = incoming[i];
                }
                seen[slot] = true;
            }

            if (!(seen[0] && seen[1] && seen[2])) {
                auto progress = common::Progress::pending();
                progress.duplicates = duplicates;
                return progress;
            }
            auto raw = buf;
            size_t len = raw.size();
            while (len > 0 && raw[len - 1] == 0) {
                --len;
            }
            auto progress = common::Progress::complete(std::string(raw.begin(), raw.begin() + len));
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

    // --- config: set_id is a blocking read-after-write (0xC3 with a subcommand
    // byte); reply_enabled/restore_factory/save are fire-and-forget 7-byte fills. ---

    inline common::TransactionPlan read_config_set_id(uint32_t id, uint8_t mode, uint8_t target) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 3;
        message.data[0] = 0xC3;
        message.data[1] = mode;
        message.data[2] = target;
        common::TransactionPlan plan;
        plan.domain = "config";
        plan.keys = {"config"};
        plan.command_family = "configuration";
        plan.frames = {message};
        plan.consume = [id, mode, target](const can::Message &input) {
            if (input.id != id || input.is_extended || input.is_fd || input.is_brs || input.dlc != 3 ||
                input.data[0] != 0xC3) {
                return common::Progress::unmatched();
            }
            if ((mode != 3 && input.data[1] != target) || (mode != 2 && input.data[2] != target)) {
                return common::Progress::unmatched();
            }
            return common::Progress::complete(CanIds{input.data[1], input.data[2]});
        };
        return plan;
    }
    inline common::TransactionPlan write_config_reply_enabled(uint32_t id, bool enabled) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        for (auto &byte : message.data) {
            byte = 0xCC;
        }
        message.data[6] = enabled ? uint8_t{0x01} : uint8_t{0xCC};
        common::TransactionPlan plan;
        plan.domain = "config";
        plan.keys = {"config"};
        plan.frames = {message};
        return plan;
    }
    inline common::TransactionPlan write_config_restore_factory(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        for (auto &byte : message.data) {
            byte = 0xCE;
        }
        common::TransactionPlan plan;
        plan.domain = "config";
        plan.keys = {"config"};
        plan.frames = {message};
        return plan;
    }
    inline common::TransactionPlan write_config_save(uint32_t id) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 7;
        for (auto &byte : message.data) {
            byte = 0xCF;
        }
        common::TransactionPlan plan;
        plan.domain = "config";
        plan.keys = {"config"};
        plan.frames = {message};
        return plan;
    }

    // --- tip force: a 4-channel composite (0x20-0x23, one byte per finger) used
    // by get_blocking(), plus 5 separate per-finger reads (0x28-0x32) with a
    // different single-frame [normal,tangential,direction,proximity] layout. ---

    inline constexpr uint8_t kTipNormalCmd = 0x20;
    inline constexpr uint8_t kTipTangentialCmd = 0x21;
    inline constexpr uint8_t kTipDirectionCmd = 0x22;
    inline constexpr uint8_t kTipProximityCmd = 0x23;

    inline common::Progress decode_tip_channel(uint32_t id, uint8_t cmd, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 6 ||
            message.data[0] != cmd) {
            return common::Progress::unmatched();
        }
        std::array<uint8_t, TipForce::kFingerCount> raw{};
        for (size_t i = 0; i < TipForce::kFingerCount; ++i) {
            raw[i] = message.data[i + 1];
        }
        return common::Progress::complete(raw);
    }
    inline common::TransactionPlan read_tip_channel(uint32_t id, uint8_t cmd, const std::string &domain) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = cmd;
        common::TransactionPlan plan;
        plan.domain = domain;
        plan.keys = {domain};
        plan.frames = {message};
        plan.consume = [id, cmd](const can::Message &input) { return decode_tip_channel(id, cmd, input); };
        return plan;
    }

    inline common::Progress decode_tip_finger(uint32_t id, uint8_t cmd, const can::Message &message) {
        if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc != 5 ||
            message.data[0] != cmd) {
            return common::Progress::unmatched();
        }
        return common::Progress::complete(
            TipFinger{message.data[1], message.data[2], message.data[3], message.data[4]});
    }
    inline common::TransactionPlan read_tip_finger(uint32_t id, uint8_t cmd, const std::string &domain) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 1;
        message.data[0] = cmd;
        common::TransactionPlan plan;
        plan.domain = domain;
        plan.keys = {domain};
        plan.frames = {message};
        plan.consume = [id, cmd](const can::Message &input) { return decode_tip_finger(id, cmd, input); };
        return plan;
    }

    inline common::TransactionPlan read_tip_force(uint32_t id) {
        common::TransactionPlan plan;
        plan.domain = "tip_force";
        plan.preserve_last_success = true;
        plan.keys = {"tip_force_normal", "tip_force_tangential", "tip_force_direction", "tip_force_proximity"};
        for (uint8_t cmd = 0x20; cmd <= 0x23; ++cmd) {
            can::Message m{};
            m.format = can::FrameFormat::kClassic;
            m.id = id;
            m.dlc = 1;
            m.data[0] = cmd;
            plan.frames.push_back(m);
        }
        plan.consume = [id, values = std::array<std::array<uint8_t, 5>, 4>{},
                        seen = std::array<bool, 4>{}](const can::Message &m) mutable {
            if (m.id != id || m.is_extended || m.is_fd || m.is_brs || m.dlc != 6 || m.data[0] < 0x20 ||
                m.data[0] > 0x23) {
                return common::Progress::unmatched();
            }
            const size_t channel = m.data[0] - 0x20;
            std::array<uint8_t, 5> incoming{};
            std::copy_n(m.data.begin() + 1, 5, incoming.begin());
            if (seen[channel] && values[channel] != incoming) {
                common::Progress failure;
                failure.kind = common::ProgressKind::failed;
                failure.terminal_error = false;
                failure.conflicting_duplicates = 1;
                failure.error =
                    std::make_exception_ptr(::linkerbot::hand::ProtocolError("tip force conflicting duplicate"));
                return failure;
            }
            const bool duplicate = seen[channel];
            values[channel] = incoming;
            seen[channel] = true;
            auto result = common::Progress::pending();
            result.duplicates = duplicate ? 1 : 0;
            if (std::all_of(seen.begin(), seen.end(), [](bool b) { return b; })) {
                std::array<TipFinger, 5> fingers{};
                for (size_t i = 0; i < 5; ++i) {
                    fingers[i] = TipFinger{values[0][i], values[1][i], values[2][i], values[3][i]};
                }
                result.kind = common::ProgressKind::complete;
                result.value = TipForceData{TipForce::from_array(fingers), {}};
            }
            return result;
        };
        return plan;
    }

    class Backend final : public common::ProtocolBackend {
    public:
        explicit Backend(uint32_t id) : id_(id) {}

        common::TransactionPlan make_read(const std::string &operation, const std::any &argument = {}) const override {
            if (operation == "tip_force") {
                return read_tip_force(id_);
            }
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
            if (operation == "stall_time" || operation == "stall_threshold" || operation == "stall_torque") {
                throw UnsupportedOperationError("L20Lite stall parameters are write-response cache only");
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
            if (operation == "config_set_id") {
                return read_config_set_id(id_, 0x01, std::any_cast<uint8_t>(argument));
            }
            if (operation == "config_set_device_id") {
                return read_config_set_id(id_, 0x02, std::any_cast<uint8_t>(argument));
            }
            if (operation == "config_set_transmit_id") {
                return read_config_set_id(id_, 0x03, std::any_cast<uint8_t>(argument));
            }
            if (operation == "tip_force_normal") {
                return read_tip_channel(id_, kTipNormalCmd, "tip_force_normal");
            }
            if (operation == "tip_force_tangential") {
                return read_tip_channel(id_, kTipTangentialCmd, "tip_force_tangential");
            }
            if (operation == "tip_force_direction") {
                return read_tip_channel(id_, kTipDirectionCmd, "tip_force_direction");
            }
            if (operation == "tip_force_proximity") {
                return read_tip_channel(id_, kTipProximityCmd, "tip_force_proximity");
            }
            if (operation == "tip_force_finger_thumb") {
                return read_tip_finger(id_, 0x28, "tip_force_finger_thumb");
            }
            if (operation == "tip_force_finger_index") {
                return read_tip_finger(id_, 0x29, "tip_force_finger_index");
            }
            if (operation == "tip_force_finger_middle") {
                return read_tip_finger(id_, 0x30, "tip_force_finger_middle");
            }
            if (operation == "tip_force_finger_ring") {
                return read_tip_finger(id_, 0x31, "tip_force_finger_ring");
            }
            if (operation == "tip_force_finger_pinky") {
                return read_tip_finger(id_, 0x32, "tip_force_finger_pinky");
            }
            throw ValidationError("unknown L20 Lite protocol operation: " + operation);
        }

        common::TransactionPlan make_command(const std::string &operation,
                                             const std::any &argument = {}) const override {
            if (operation == "angle") {
                return write_angle(id_, std::any_cast<std::array<uint8_t, kJointCount>>(argument));
            }
            if (operation == "current_limit") {
                return write_current_limit(id_, std::any_cast<std::array<uint8_t, kJointCount>>(argument));
            }
            if (operation == "speed") {
                return write_speed(id_, std::any_cast<std::array<uint8_t, kJointCount>>(argument));
            }
            if (operation == "acceleration") {
                return write_acceleration(id_, std::any_cast<std::array<uint8_t, kJointCount>>(argument));
            }
            if (operation == "stall_time") {
                return write_stall_bank(id_, kStallTimeCmd, "stall_time",
                                        std::any_cast<std::array<uint8_t, kJointCount>>(argument));
            }
            if (operation == "stall_threshold") {
                return write_stall_bank(id_, kStallThresholdCmd, "stall_threshold",
                                        std::any_cast<std::array<uint8_t, kJointCount>>(argument));
            }
            if (operation == "stall_torque") {
                return write_stall_bank(id_, kStallTorqueCmd, "stall_torque",
                                        std::any_cast<std::array<uint8_t, kJointCount>>(argument));
            }
            if (operation == "config_reply_enabled") {
                return write_config_reply_enabled(id_, std::any_cast<bool>(argument));
            }
            if (operation == "config_restore_factory") {
                return write_config_restore_factory(id_);
            }
            if (operation == "config_save") {
                return write_config_save(id_);
            }
            throw ValidationError("unknown L20 Lite protocol operation: " + operation);
        }

    private:
        uint32_t id_;
    };

} // namespace linkerbot::hand::l20lite::protocol
