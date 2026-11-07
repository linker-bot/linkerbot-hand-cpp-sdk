#pragma once
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <linkerbot/can/message.hpp>
#include <linkerbot/hand/l20/joints.hpp>
#include <vector>
namespace linkerbot::hand::l20 {
    // Sentinel meaning "this byte of the payload is a protocol-reserved 0x00
    // that does not map to any joint" (e.g. the two reserved slots between
    // root1 and tip, or the `yaw` slot on non-thumb fingers).
    inline constexpr size_t kNoJoint = static_cast<size_t>(-1);

    inline constexpr size_t kFramesPerDomain = 5;
    inline constexpr size_t kPayloadBytes = 6;

    struct FramePart {
        uint8_t cmd = 0;
        std::array<size_t, kPayloadBytes> joints{};
        size_t count = 0;
    };

    using FrameParts = std::array<FramePart, kFramesPerDomain>;

    // Payload layout per finger frame:
    //   thumb          : [abd, yaw,      root1, 0x00, 0x00, tip]
    //   index/middle/  : [abd, reserved, root1, 0x00, 0x00, tip]
    //   ring /pinky    :
    // Only the thumb populates the `yaw` slot with a real joint value.

    // Angle: 0x41..0x45 (thumb..pinky)
    inline constexpr FrameParts kAngleFrames = {{
        FramePart{0x41, {0, 1, 2, kNoJoint, kNoJoint, 3}, kPayloadBytes},
        FramePart{0x42, {4, kNoJoint, 5, kNoJoint, kNoJoint, 6}, kPayloadBytes},
        FramePart{0x43, {7, kNoJoint, 8, kNoJoint, kNoJoint, 9}, kPayloadBytes},
        FramePart{0x44, {10, kNoJoint, 11, kNoJoint, kNoJoint, 12}, kPayloadBytes},
        FramePart{0x45, {13, kNoJoint, 14, kNoJoint, kNoJoint, 15}, kPayloadBytes},
    }};

    // Speed: 0x49..0x4D
    inline constexpr FrameParts kSpeedFrames = {{
        FramePart{0x49, {0, 1, 2, kNoJoint, kNoJoint, 3}, kPayloadBytes},
        FramePart{0x4A, {4, kNoJoint, 5, kNoJoint, kNoJoint, 6}, kPayloadBytes},
        FramePart{0x4B, {7, kNoJoint, 8, kNoJoint, kNoJoint, 9}, kPayloadBytes},
        FramePart{0x4C, {10, kNoJoint, 11, kNoJoint, kNoJoint, 12}, kPayloadBytes},
        FramePart{0x4D, {13, kNoJoint, 14, kNoJoint, kNoJoint, 15}, kPayloadBytes},
    }};

    // Measured current / current limit: 0x51..0x55
    inline constexpr FrameParts kCurrentFrames = {{
        FramePart{0x51, {0, 1, 2, kNoJoint, kNoJoint, 3}, kPayloadBytes},
        FramePart{0x52, {4, kNoJoint, 5, kNoJoint, kNoJoint, 6}, kPayloadBytes},
        FramePart{0x53, {7, kNoJoint, 8, kNoJoint, kNoJoint, 9}, kPayloadBytes},
        FramePart{0x54, {10, kNoJoint, 11, kNoJoint, kNoJoint, 12}, kPayloadBytes},
        FramePart{0x55, {13, kNoJoint, 14, kNoJoint, kNoJoint, 15}, kPayloadBytes},
    }};

    // Fault: 0x59..0x5D
    inline constexpr FrameParts kFaultFrames = {{
        FramePart{0x59, {0, 1, 2, kNoJoint, kNoJoint, 3}, kPayloadBytes},
        FramePart{0x5A, {4, kNoJoint, 5, kNoJoint, kNoJoint, 6}, kPayloadBytes},
        FramePart{0x5B, {7, kNoJoint, 8, kNoJoint, kNoJoint, 9}, kPayloadBytes},
        FramePart{0x5C, {10, kNoJoint, 11, kNoJoint, kNoJoint, 12}, kPayloadBytes},
        FramePart{0x5D, {13, kNoJoint, 14, kNoJoint, kNoJoint, 15}, kPayloadBytes},
    }};

    // Temperature: 0x61..0x65
    inline constexpr FrameParts kTemperatureFrames = {{
        FramePart{0x61, {0, 1, 2, kNoJoint, kNoJoint, 3}, kPayloadBytes},
        FramePart{0x62, {4, kNoJoint, 5, kNoJoint, kNoJoint, 6}, kPayloadBytes},
        FramePart{0x63, {7, kNoJoint, 8, kNoJoint, kNoJoint, 9}, kPayloadBytes},
        FramePart{0x64, {10, kNoJoint, 11, kNoJoint, kNoJoint, 12}, kPayloadBytes},
        FramePart{0x65, {13, kNoJoint, 14, kNoJoint, kNoJoint, 15}, kPayloadBytes},
    }};

    // Angle-set frames need firmware pacing between frames (500us). Force-sensor
    // requests also honour a small delay between per-finger request frames.
    // Fault / current / temperature / speed sense without a gap: L20 MCU drops
    // some of the five DLC=1 replies (candump on right-hand 0x27: missing 0x5B
    // / 0x5A). Polling several domains at once makes speed the first to starve
    // because it used to burst 0x49–0x4D with no pacing.
    inline constexpr auto kAngleInterFrameDelay = std::chrono::microseconds{500};
    inline constexpr auto kForceInterRequestDelay = std::chrono::microseconds{2500};
    inline constexpr auto kSenseInterFrameDelay = kForceInterRequestDelay;

    inline std::vector<can::Message> joint_messages(uint32_t id, const FrameParts &parts,
                                                    const std::array<uint8_t, kJointCount> *raw) {
        std::vector<can::Message> messages;
        for (const auto &part : parts) {
            can::Message msg{};
            msg.format = can::FrameFormat::kClassic;
            msg.id = id;
            msg.dlc = raw ? static_cast<uint8_t>(1 + part.count) : 1;
            msg.data[0] = part.cmd;
            if (raw) {
                for (size_t i = 0; i < part.count; ++i) {
                    msg.data[i + 1] = part.joints[i] == kNoJoint ? 0 : (*raw)[part.joints[i]];
                }
            }
            messages.push_back(msg);
        }
        return messages;
    }

}
