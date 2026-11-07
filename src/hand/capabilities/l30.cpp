#include "profile_helpers.hpp"
#include "profiles.hpp"

#include <array>

#include <linkerbot/hand/l30/joints.hpp>

namespace linkerbot::hand::capabilities_detail {
    namespace {
        DeviceCapabilities make_l30_profile(HandSide side) {
            constexpr std::array<JointMotionType, l30::kJointCount> motion_types = {
                JointMotionType::kFlexion,   JointMotionType::kFlexion,   JointMotionType::kAbduction,
                JointMotionType::kRotation,  JointMotionType::kAbduction, JointMotionType::kFlexion,
                JointMotionType::kFlexion,   JointMotionType::kFlexion,   JointMotionType::kFlexion,
                JointMotionType::kFlexion,   JointMotionType::kFlexion,   JointMotionType::kAbduction,
                JointMotionType::kAbduction, JointMotionType::kAbduction, JointMotionType::kFlexion,
                JointMotionType::kFlexion,   JointMotionType::kWrist};

            std::vector<JointCapability> joints;
            joints.reserve(l30::kJointCount);
            for (size_t index = 0; index < l30::kJointCount; ++index) {
                const auto &spec = l30::kJointSpecs[index];
                joints.push_back(
                    {index,
                     spec.name,
                     motion_types[index],
                     {static_cast<double>(spec.minimum), static_cast<double>(spec.maximum), true, 1.0},
                     PercentMapping{static_cast<double>(spec.reverse_percent ? spec.maximum : spec.minimum),
                                    static_cast<double>(spec.reverse_percent ? spec.minimum : spec.maximum),
                                    spec.reverse_percent ? "flexed" : "raw_min",
                                    spec.reverse_percent ? "extended" : "raw_max", Confidence::kMeasured},
                     {}});
            }

            const auto names = channels(joints);
            const NumericRange speed_range{l30::kSpeedMin, l30::kSpeedMax, true, 1.0};
            const NumericRange current_limit_range{l30::kCurrentLimitMin, l30::kCurrentLimitMax, true, 1.0};
            const NumericRange i16_range{-32768.0, 32767.0, true, 1.0};
            const auto u8 = u8_range();

            DeviceCapabilities profile;
            profile.model = Model::kL30;
            profile.side = side;
            profile.transport = l30_transport();
            profile.operations.read_device_info = true;
            profile.operations.enable = true;
            profile.operations.calibrate = true;
            profile.operations.disable = true;
            profile.operations.acknowledged_operations = {DeviceOperation::kEnable, DeviceOperation::kDisable,
                                                          DeviceOperation::kCalibrate};
            profile.joints = joints;
            profile.commands = {
                angle_command(joints, "i16_be"),
                command(CommandSemantic::kSpeeds, names, speed_range, "u16_be", "count", "0.732 rpm per count",
                        Confidence::kDocumented),
                command(CommandSemantic::kCurrentLimits, names, current_limit_range, "u16_be", "count",
                        "6.5 mA per count", Confidence::kDocumented),
            };
            profile.telemetry = {
                angle_telemetry(joints, "i16_be", ReadMode::kQueryAndReport),
                telemetry(TelemetrySemantic::kConfiguredSpeeds, names, ReadMode::kCommandCache, "u16_be",
                          {l30::kJointCount}, speed_range, "count",
                          "latest SDK command target; no device confirmation or active query", Confidence::kDocumented,
                          false, false),
                telemetry(TelemetrySemantic::kMeasuredSpeeds, names, ReadMode::kQueryAndReport, "i16_be",
                          {l30::kJointCount}, i16_range, "count", "0.732 rpm per count", Confidence::kDocumented),
                telemetry(TelemetrySemantic::kCurrents, names, ReadMode::kQueryAndReport, "i16_be", {l30::kJointCount},
                          i16_range, "count", "6.5 mA per count; sign carries effort direction",
                          Confidence::kDocumented),
                telemetry(TelemetrySemantic::kCurrentLimits, names, ReadMode::kCommandCache, "u16_be",
                          {l30::kJointCount}, current_limit_range, "count",
                          "latest SDK command target; no active query", Confidence::kDocumented, false, false),
                telemetry(TelemetrySemantic::kTemperatures, names, ReadMode::kQueryAndReport, "u8", {l30::kJointCount},
                          u8, "degree_celsius", "1 degree Celsius per count", Confidence::kDocumented),
                telemetry(TelemetrySemantic::kFaults, names, ReadMode::kQueryAndReport, "u8_bitmask",
                          {l30::kJointCount}, u8, std::nullopt, "per-joint fault bitmask", Confidence::kDocumented),
                telemetry(TelemetrySemantic::kForce, finger_channels(), ReadMode::kQuery, "u8", {5, 12, 6}, u8, "count",
                          "raw tactile sensor count", Confidence::kDocumented),
            };
            // Shared telemetry freshness contract with Python Hand SDK.
            for (auto &channel : profile.telemetry) {
                switch (channel.semantic) {
                case TelemetrySemantic::kAngles:
                    channel.freshness_ms = 1000.0 / 30.0;
                    break;
                case TelemetrySemantic::kConfiguredSpeeds:
                case TelemetrySemantic::kCurrentLimits:
                    channel.freshness_ms = 1000;
                    break;
                case TelemetrySemantic::kMeasuredSpeeds:
                case TelemetrySemantic::kForce:
                    channel.freshness_ms = 100;
                    break;
                case TelemetrySemantic::kCurrents:
                    channel.freshness_ms = 300;
                    break;
                case TelemetrySemantic::kTemperatures:
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

    const DeviceCapabilities &l30_profile(HandSide side) {
        return select_profile(side, [](HandSide selected) { return make_l30_profile(selected); });
    }

} // namespace linkerbot::hand::capabilities_detail
