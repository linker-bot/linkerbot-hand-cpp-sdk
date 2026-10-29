#include "profile_helpers.hpp"
#include "profiles.hpp"

#include <array>

#include <linkerbot/hand/l6/joints.hpp>

namespace linkerbot::hand::capabilities_detail {
    namespace {
        DeviceCapabilities make_l6_profile(HandSide side) {
            constexpr std::array<JointMotionType, l6::kJointCount> motion_types = {
                JointMotionType::kFlexion, JointMotionType::kAbduction, JointMotionType::kFlexion,
                JointMotionType::kFlexion, JointMotionType::kFlexion,   JointMotionType::kFlexion};

            const NumericRange angle_range{l6::kRawAngleMinimum, l6::kRawAngleMaximum, true, 1.0};
            auto joints = u8_joints(l6::kJointNames, motion_types, angle_range);
            const auto names = channels(l6::kJointNames);
            const auto u8 = u8_range();

            DeviceCapabilities profile;
            profile.model = Model::kL6;
            profile.side = side;
            profile.transport = classic_can_transport(side);
            profile.operations.read_device_info = true;
            profile.operations.clear_faults = true;
            profile.operations.save_parameters = true;
            profile.joints = joints;
            profile.commands = {
                angle_command(joints, "u8", AckSupport::kUnsupported, true),
                command(CommandSemantic::kSpeeds, names, u8, "u8", "count", "step-mode speed setting; not measured RPM",
                        Confidence::kDocumented, AckSupport::kRequired),
                command(CommandSemantic::kCurrentLimits, names, u8, "u8", "count",
                        "1400/255 mA per count; maximum motor-current setting", Confidence::kDocumented,
                        AckSupport::kRequired),
                command(CommandSemantic::kStallDetectionTimes, names, {1, 255, true, 1}, "u8", "count",
                        "10 ms per count; writing raw 0 has no effect", Confidence::kDocumented, AckSupport::kRequired),
                command(CommandSemantic::kStallThresholds, names, u8, "u8", "count",
                        "stall threshold; physical unit is unverified", Confidence::kUnknown, AckSupport::kRequired),
                command(CommandSemantic::kStallHoldCurrents, names, u8, "u8", "count",
                        "protocol calls this stall torque; physical current scale is unverified",
                        Confidence::kConflicting, AckSupport::kRequired),
            };
            profile.telemetry = {
                angle_telemetry(joints, "u8"),
                telemetry(TelemetrySemantic::kConfiguredSpeeds, names, ReadMode::kCommandResponse, "u8",
                          {l6::kJointCount}, u8, "count", "step-mode speed setting; not measured RPM",
                          Confidence::kDocumented, false, false),
                telemetry(TelemetrySemantic::kCurrentLimits, names, ReadMode::kQuery, "u8", {l6::kJointCount}, u8,
                          "count", "1400/255 mA per count", Confidence::kDocumented, false),
                telemetry(TelemetrySemantic::kTorques, names, ReadMode::kQuery, "u8", {l6::kJointCount}, u8, "count",
                          "torque output/mode setting; physical torque is unverified", Confidence::kUnknown, false),
                telemetry(TelemetrySemantic::kCurrents, names, ReadMode::kQuery, "u8", {l6::kJointCount}, u8, "count",
                          "1400/255 mA per count; absolute magnitude", Confidence::kDocumented),
                telemetry(TelemetrySemantic::kTemperatures, names, ReadMode::kQuery, "u8", {l6::kJointCount}, u8,
                          "degree_celsius", "1 degree Celsius per count", Confidence::kDocumented),
                telemetry(TelemetrySemantic::kFaults, names, ReadMode::kQuery, "u8_bitmask", {l6::kJointCount}, u8,
                          std::nullopt, "per-joint fault bitmask", Confidence::kDocumented),
                telemetry(TelemetrySemantic::kForce, finger_channels(), ReadMode::kQuery, "u8", {5, 12, 6}, u8, "count",
                          "raw tactile ADC count", Confidence::kUnknown),
                telemetry(TelemetrySemantic::kStallDetectionTimes, names, ReadMode::kQuery, "u8", {l6::kJointCount}, u8,
                          "count", "10 ms per count", Confidence::kDocumented, false),
                telemetry(TelemetrySemantic::kStallThresholds, names, ReadMode::kQuery, "u8", {l6::kJointCount}, u8,
                          "count", "stall threshold; physical unit is unverified", Confidence::kUnknown, false),
                telemetry(TelemetrySemantic::kStallHoldCurrents, names, ReadMode::kQuery, "u8", {l6::kJointCount}, u8,
                          "count", "protocol calls this stall torque", Confidence::kConflicting, false),
            };
            // Shared telemetry freshness contract with Python Hand SDK.
            for (auto &channel : profile.telemetry) {
                switch (channel.semantic) {
                case TelemetrySemantic::kTorques:
                case TelemetrySemantic::kTemperatures:
                case TelemetrySemantic::kStallDetectionTimes:
                case TelemetrySemantic::kStallThresholds:
                case TelemetrySemantic::kStallHoldCurrents:
                    channel.freshness_ms = 3000;
                    break;
                case TelemetrySemantic::kAngles:
                    channel.freshness_ms = 1000.0 / 30.0;
                    break;
                case TelemetrySemantic::kConfiguredSpeeds:
                    channel.freshness_ms = 1000;
                    break;
                case TelemetrySemantic::kCurrentLimits:
                    channel.freshness_ms = 100;
                    break;
                case TelemetrySemantic::kForce:
                case TelemetrySemantic::kFaults:
                    channel.freshness_ms = 500;
                    break;
                case TelemetrySemantic::kCurrents:
                    channel.freshness_ms = 300;
                    break;
                default:
                    break;
                }
            }
            return profile;
        }
    }

    const DeviceCapabilities &l6_profile(HandSide side) {
        return select_profile(side, [](HandSide selected) { return make_l6_profile(selected); });
    }

} // namespace linkerbot::hand::capabilities_detail
