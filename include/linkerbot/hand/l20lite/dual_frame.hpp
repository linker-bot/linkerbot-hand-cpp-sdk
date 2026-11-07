#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>

#include <linkerbot/can/dispatcher.hpp>
#include <linkerbot/can/message.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/l20lite/frame_layout.hpp>
#include <linkerbot/hand/o6/legacy_request.hpp>

namespace linkerbot::hand::l20lite {

    class DualFrameAssembler {
    public:
        DualFrameAssembler(FramePart first, FramePart second) : parts_{std::move(first), std::move(second)} {}

        [[nodiscard]] const std::array<FramePart, 2> &parts() const { return parts_; }

        // Returns false if a previous sense is still in flight (polling overlap).
        bool begin_sense(bool force = false) {
            const std::scoped_lock lock(mutex_);
            const auto now = std::chrono::steady_clock::now();
            (void)force; // force cannot supersede a response still on the wire.
            if (in_flight_ && now >= deadline_) {
                poisoned_ = true;
            }
            if (poisoned_) {
                throw DeviceNotReadyError("dual-frame key quarantined; rebuild manager");
            }
            if (in_flight_) {
                return false;
            }
            deadline_ = classic_detail::current_send ? classic_detail::current_send->deadline
                                                     : now + std::chrono::milliseconds(200);
            in_flight_ = true;
            in_flight_since_ = now;
            have_first_ = false;
            have_second_ = false;
            return true;
        }

        std::optional<std::array<uint8_t, kJointCount>> put(uint8_t cmd, const uint8_t *payload, size_t len) {
            const std::scoped_lock lock(mutex_);
            if (in_flight_ && std::chrono::steady_clock::now() >= deadline_) {
                poisoned_ = true;
            }
            if (!in_flight_ || poisoned_) {
                return std::nullopt;
            }
            const int idx = index_of(cmd);
            if (idx < 0) {
                return std::nullopt;
            }
            const auto &part = parts_[static_cast<size_t>(idx)];
            if (len != part.count) {
                return std::nullopt;
            }

            auto &slot = (idx == 0) ? first_ : second_;
            if (idx == 0 ? have_first_ : have_second_) {
                for (size_t i = 0; i < part.count; ++i) {
                    if (slot[i] != payload[i]) {
                        poisoned_ = true;
                        return std::nullopt;
                    }
                }
                return std::nullopt;
            }
            for (size_t i = 0; i < part.count; ++i) {
                slot[i] = payload[i];
            }
            (idx == 0 ? have_first_ : have_second_) = true;

            if (!have_first_ || !have_second_) {
                return std::nullopt;
            }

            std::array<uint8_t, kJointCount> raw{};
            fill(raw, parts_[0], first_);
            fill(raw, parts_[1], second_);
            have_first_ = false;
            have_second_ = false;
            in_flight_ = false;
            return raw;
        }

    private:
        int index_of(uint8_t cmd) const {
            if (parts_[0].cmd == cmd) {
                return 0;
            }
            if (parts_[1].cmd == cmd) {
                return 1;
            }
            return -1;
        }

        static void fill(std::array<uint8_t, kJointCount> &raw, const FramePart &part,
                         const std::array<uint8_t, 6> &payload) {
            for (size_t i = 0; i < part.count; ++i) {
                raw[part.joints[i]] = payload[i];
            }
        }

        std::array<FramePart, 2> parts_;
        std::mutex mutex_;
        std::array<uint8_t, 6> first_{};
        std::array<uint8_t, 6> second_{};
        bool have_first_ = false;
        bool have_second_ = false;
        bool in_flight_ = false;
        bool poisoned_ = false;
        std::chrono::steady_clock::time_point deadline_{};
        std::chrono::steady_clock::time_point in_flight_since_{};
    };

    inline void send_joint_frames(can::CANDispatcher &dispatcher, uint32_t id, const std::array<FramePart, 2> &parts,
                                  const std::array<uint8_t, kJointCount> &raw, std::chrono::microseconds gap = {}) {
        std::vector<can::Message> frames;
        for (const auto &part : parts) {
            can::Message msg{};
            msg.format = can::FrameFormat::kClassic;
            msg.id = id;
            msg.dlc = static_cast<uint8_t>(1 + part.count);
            msg.data[0] = part.cmd;
            for (size_t i = 0; i < part.count; ++i) {
                msg.data[i + 1] = raw[part.joints[i]];
            }
            frames.push_back(msg);
        }
        classic_detail::send_frames(dispatcher, frames, gap, gap);
    }

    inline void send_sense_frames(can::CANDispatcher &dispatcher, uint32_t id, const std::array<FramePart, 2> &parts,
                                  std::chrono::microseconds gap = {}) {
        std::vector<can::Message> frames;
        for (const auto &part : parts) {
            can::Message msg{};
            msg.format = can::FrameFormat::kClassic;
            msg.id = id;
            msg.dlc = 1;
            msg.data[0] = part.cmd;
            frames.push_back(msg);
        }
        classic_detail::send_frames(dispatcher, frames, gap, gap);
    }

} // namespace linkerbot::hand::l20lite
