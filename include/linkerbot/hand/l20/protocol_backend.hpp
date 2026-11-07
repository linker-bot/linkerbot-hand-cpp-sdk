#include <linkerbot/hand/errors.hpp>
#pragma once
#include <algorithm>
#include <linkerbot/hand/l20/angle.hpp>
#include <linkerbot/hand/l20/current.hpp>
#include <linkerbot/hand/l20/fault.hpp>
#include <linkerbot/hand/l20/force_sensor.hpp>
#include <linkerbot/hand/l20/frame_layout.hpp>
#include <linkerbot/hand/l20/speed.hpp>
#include <linkerbot/hand/l20/temperature.hpp>
#include <linkerbot/hand/l20/version.hpp>
#include <linkerbot/hand/protocol/transaction.hpp>
#include <stdexcept>
namespace linkerbot::hand::l20::protocol {
    namespace common = ::linkerbot::hand::protocol;
    inline common::TransactionPlan
    read_joints(uint32_t id, const FrameParts &parts, std::string domain,
                std::function<common::Progress(const std::array<uint8_t, kJointCount> &)> finish) {
        common::TransactionPlan plan;
        plan.domain = domain;
        plan.keys = {domain};
        plan.frames = joint_messages(id, parts, nullptr);
        plan.gap = kSenseInterFrameDelay;
        plan.consume = [id, parts, finish, slots = std::array<std::array<uint8_t, kPayloadBytes>, 5>{},
                        seen = std::array<bool, 5>{}](const can::Message &m) mutable {
            if (m.id != id || m.is_extended || m.is_fd || m.is_brs || m.dlc != 7) {
                return common::Progress::unmatched();
            }
            size_t index = 0;
            while (index < parts.size() && parts[index].cmd != m.data[0]) {
                ++index;
            }
            if (index == parts.size()) {
                return common::Progress::unmatched();
            }
            std::array<uint8_t, kPayloadBytes> incoming{};
            std::copy_n(m.data.begin() + 1, kPayloadBytes, incoming.begin());
            if (seen[index] && slots[index] != incoming) {
                common::Progress result;
                result.kind = common::ProgressKind::failed;
                result.conflicting_duplicates = 1;
                result.error =
                    std::make_exception_ptr(::linkerbot::hand::ProtocolError("L20 conflicting joint fragment"));
                return result;
            }
            const bool duplicate = seen[index];
            slots[index] = incoming;
            seen[index] = true;
            auto result = common::Progress::pending();
            if (std::all_of(seen.begin(), seen.end(), [](bool b) { return b; })) {
                std::array<uint8_t, kJointCount> raw{};
                for (size_t frame = 0; frame < parts.size(); ++frame) {
                    for (size_t byte = 0; byte < parts[frame].count; ++byte) {
                        if (parts[frame].joints[byte] != kNoJoint) {
                            raw[parts[frame].joints[byte]] = slots[frame][byte];
                        }
                    }
                }
                result = finish(raw);
            }
            result.duplicates = duplicate ? 1 : 0;
            return result;
        };
        return plan;
    }
    inline common::TransactionPlan read_angle(uint32_t id) {
        return read_joints(id, kAngleFrames, "angle", [](const auto &raw) {
            return common::Progress::complete(AngleData{Angle::from_array(raw_to_percents(raw)), {}});
        });
    }
    inline common::TransactionPlan read_speed(uint32_t id) {
        return read_joints(id, kSpeedFrames, "speed", [](const auto &raw) {
            return common::Progress::complete(SpeedData{Speed::from_array(raw_to_percents(raw)), {}});
        });
    }
    inline common::TransactionPlan read_current(uint32_t id) {
        auto plan = read_joints(id, kCurrentFrames, "current",
                                [](const auto &raw) { return common::Progress::complete(CurrentData{raw, {}}); });
        plan.keys = {"l20.0x51-0x55"};
        return plan;
    }
    inline common::TransactionPlan write_current_limit(uint32_t id, const std::array<uint8_t, kJointCount> &raw) {
        // These opcodes return live current even after a limit write. Keep the
        // optional feedback in the current domain; it is not a limit readback.
        auto plan = read_current(id);
        plan.frames = joint_messages(id, kCurrentFrames, &raw);
        plan.keys = {"l20.0x51-0x55"};
        // The MCU can omit replies when configuration frames arrive back-to-back.
        plan.gap = kSenseInterFrameDelay;
        plan.optional_response_window = std::chrono::milliseconds(200);
        plan.supersede_optional_feedback = true;
        plan.publish_after_send = true;
        plan.command_family = "current_limits";
        auto decode = std::move(plan.consume);
        plan.consume = [decode = std::move(decode)](const can::Message &message) mutable {
            auto result = decode(message);
            result.origin = common::Origin::command_feedback;
            return result;
        };
        return plan;
    }
    inline common::TransactionPlan read_temperature(uint32_t id) {
        return read_joints(id, kTemperatureFrames, "temperature", [](const auto &raw) {
            std::array<float, kJointCount> values{};
            std::copy(raw.begin(), raw.end(), values.begin());
            return common::Progress::complete(TemperatureData{Temperature::from_array(values), {}});
        });
    }
    inline common::TransactionPlan read_fault(uint32_t id) {
        return read_joints(id, kFaultFrames, "fault", [](const auto &raw) {
            return common::Progress::complete(FaultData{Fault::from_raw(raw), {}});
        });
    }
    inline common::TransactionPlan read_force_sensor(uint32_t id, uint8_t prefix, const std::string &domain) {
        can::Message message{};
        message.format = can::FrameFormat::kClassic;
        message.id = id;
        message.dlc = 2;
        message.data[0] = prefix;
        message.data[1] = 0xC6;
        common::TransactionPlan plan;
        plan.domain = domain;
        plan.exclusion_groups = {"l20.tactile"};
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
        plan.exclusion_groups = {"l20.tactile"};
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
            // Only the final serial fragment may omit trailing padding.
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
            if (operation == "speed") {
                return read_speed(id_);
            }
            if (operation == "current") {
                return read_current(id_);
            }
            if (operation == "temperature") {
                return read_temperature(id_);
            }
            if (operation == "fault") {
                return read_fault(id_);
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
            if (operation == "force_sensor_all") {
                return read_force_sensor_all(id_);
            }
            if (operation == "force_sensor_thumb") {
                return read_force_sensor(id_, 0xB1, operation);
            }
            if (operation == "force_sensor_index") {
                return read_force_sensor(id_, 0xB2, operation);
            }
            if (operation == "force_sensor_middle") {
                return read_force_sensor(id_, 0xB3, operation);
            }
            if (operation == "force_sensor_ring") {
                return read_force_sensor(id_, 0xB4, operation);
            }
            if (operation == "force_sensor_pinky") {
                return read_force_sensor(id_, 0xB5, operation);
            }
            throw ValidationError("unknown L20 read: " + operation);
        }
        common::TransactionPlan make_command(const std::string &operation,
                                             const std::any &argument = {}) const override {
            if (operation == "current_limit") {
                return write_current_limit(id_, std::any_cast<std::array<uint8_t, kJointCount>>(argument));
            }
            FrameParts parts;
            if (operation == "angle") {
                parts = kAngleFrames;
            } else if (operation == "speed") {
                parts = kSpeedFrames;
            } else if (operation == "fault") {
                parts = kFaultFrames;
            } else {
                throw ValidationError("unknown L20 command: " + operation);
            }
            auto plan = make_read(operation);
            std::array<uint8_t, kJointCount> raw{};
            if (operation != "fault") {
                raw = std::any_cast<decltype(raw)>(argument);
            }
            plan.frames = joint_messages(id_, parts, &raw);
            if (operation == "fault") {
                for (auto &frame : plan.frames) {
                    std::fill_n(frame.data.begin() + 1, 6, 1);
                }
            }
            plan.gap = operation == "angle" ? kAngleInterFrameDelay : kSenseInterFrameDelay;
            plan.optional_response_window = std::chrono::milliseconds(200);
            plan.supersede_optional_feedback = true;
            plan.publish_after_send = true;
            plan.command_family = operation;
            auto decode = std::move(plan.consume);
            plan.consume = [decode = std::move(decode)](const can::Message &m) mutable {
                auto result = decode(m);
                result.origin = common::Origin::command_feedback;
                return result;
            };
            return plan;
        }

    private:
        uint32_t id_;
    };
}
