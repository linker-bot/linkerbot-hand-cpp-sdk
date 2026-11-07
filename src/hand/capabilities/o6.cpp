#include "profile_helpers.hpp"
#include "profiles.hpp"

#include <array>

#include <linkerbot/hand/o6/joints.hpp>

namespace linkerbot::hand::capabilities_detail {
    namespace {
        DeviceCapabilities make_o6_profile(HandSide side) {
            constexpr std::array<JointMotionType, o6::kJointCount> motion_types = {
                JointMotionType::kFlexion, JointMotionType::kAbduction, JointMotionType::kFlexion,
                JointMotionType::kFlexion, JointMotionType::kFlexion,   JointMotionType::kFlexion};

            const NumericRange angle_range{o6::kRawAngleMinimum, o6::kRawAngleMaximum, true, 1.0};
            auto joints = u8_joints(o6::kJointNames, motion_types, angle_range);
            const auto names = channels(o6::kJointNames);
            const auto u8 = u8_range();
            const auto acceleration = acceleration_u8_range();

            DeviceCapabilities profile;
            profile.model = Model::kO6;
            profile.side = side;
            profile.transport = classic_can_transport(side);
            profile.operations.read_device_info = true;
            profile.operations.save_parameters = true;
            profile.operations.save_parameters_confirmed = true;
            profile.operations.acknowledged_operations = {DeviceOperation::kSaveParametersConfirmed};
            profile.joints = joints;
            profile.commands = {
                angle_command(joints, "u8", AckSupport::kUnsupported, true),
                command(CommandSemantic::kSpeeds, names, u8, "u8", "count",
                        "configured speed; documented physical scales conflict", Confidence::kConflicting,
                        AckSupport::kRequired),
                command(CommandSemantic::kAccelerations, names, acceleration, "u8", "count",
                        "raw 0 is a special maximum-acceleration encoding", Confidence::kConflicting,
                        AckSupport::kRequired),
                command(CommandSemantic::kCurrentLimits, names, u8, "u8", "count",
                        "current-limit setting; documented physical scales conflict", Confidence::kConflicting,
                        AckSupport::kRequired),
                command(CommandSemantic::kStallDetectionTimes, names, u8, "u8", "count", "10 ms per count",
                        Confidence::kDocumented, AckSupport::kRequired),
                command(CommandSemantic::kStallThresholds, names, u8, "u8", "count", "1000/255 mA per count",
                        Confidence::kDocumented, AckSupport::kRequired),
                command(CommandSemantic::kStallHoldCurrents, names, u8, "u8", "count",
                        "legacy protocol label conflicts with hold-current meaning", Confidence::kConflicting,
                        AckSupport::kRequired),
            };
            profile.telemetry = {
                angle_telemetry(joints, "u8"),
                telemetry(TelemetrySemantic::kConfiguredSpeeds, names, ReadMode::kCommandResponse, "u8",
                          {o6::kJointCount}, u8, "count", "write-response cache; bare 0x05 resets speed on V1.3.2",
                          Confidence::kConflicting, false, false),
                telemetry(TelemetrySemantic::kConfiguredAccelerations, names, ReadMode::kQuery, "u8", {o6::kJointCount},
                          acceleration, "count", "configuration value, not measured acceleration",
                          Confidence::kConflicting, false),
                telemetry(TelemetrySemantic::kCurrents, names, ReadMode::kQuery, "u8", {o6::kJointCount}, u8, "count",
                          "measured current; documented physical scales conflict", Confidence::kConflicting),
                telemetry(TelemetrySemantic::kCurrentLimits, names, ReadMode::kCommandResponse, "u8", {o6::kJointCount},
                          u8, "count", "write-response cache, not measured current", Confidence::kConflicting, false,
                          false),
                telemetry(TelemetrySemantic::kTemperatures, names, ReadMode::kQuery, "u8", {o6::kJointCount}, u8,
                          "degree_celsius", "1 degree Celsius per count", Confidence::kDocumented),
                telemetry(TelemetrySemantic::kFaults, names, ReadMode::kQuery, "u8_bitmask", {o6::kJointCount}, u8,
                          std::nullopt, "per-joint fault bitmask", Confidence::kDocumented),
                telemetry(TelemetrySemantic::kForce, finger_channels(), ReadMode::kQuery, "u8", {5, 10, 4}, u8, "count",
                          "raw tactile normal-force count", Confidence::kUnknown),
                telemetry(TelemetrySemantic::kStallDetectionTimes, names, ReadMode::kQuery, "u8", {o6::kJointCount}, u8,
                          "count", "10 ms per count", Confidence::kDocumented, false),
                telemetry(TelemetrySemantic::kStallThresholds, names, ReadMode::kQuery, "u8", {o6::kJointCount}, u8,
                          "count", "1000/255 mA per count", Confidence::kDocumented, false),
                telemetry(TelemetrySemantic::kStallHoldCurrents, names, ReadMode::kQuery, "u8", {o6::kJointCount}, u8,
                          "count", "legacy protocol label conflicts with hold-current meaning",
                          Confidence::kConflicting, false),
            };
            // Shared telemetry freshness contract with Python Hand SDK.
            for (auto &channel : profile.telemetry) {
                switch (channel.semantic) {
                case TelemetrySemantic::kAngles:
                    channel.freshness_ms = 1000.0 / 30.0;
                    break;
                case TelemetrySemantic::kConfiguredSpeeds:
                case TelemetrySemantic::kCurrents:
                case TelemetrySemantic::kForce:
                    channel.freshness_ms = 100;
                    break;
                case TelemetrySemantic::kConfiguredAccelerations:
                    channel.freshness_ms = 300;
                    break;
                case TelemetrySemantic::kCurrentLimits:
                case TelemetrySemantic::kTemperatures:
                case TelemetrySemantic::kStallDetectionTimes:
                case TelemetrySemantic::kStallThresholds:
                case TelemetrySemantic::kStallHoldCurrents:
                    channel.freshness_ms = 3000;
                    break;
                case TelemetrySemantic::kFaults:
                    channel.freshness_ms = 500;
                    break;
                default:
                    break;
                }
            }
            return profile;
        }
    }

    const DeviceCapabilities &o6_profile(HandSide side) {
        return select_profile(side, [](HandSide selected) { return make_o6_profile(selected); });
    }

} // namespace linkerbot::hand::capabilities_detail
