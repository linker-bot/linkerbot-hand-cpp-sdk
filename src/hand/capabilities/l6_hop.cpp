#include "profile_helpers.hpp"
#include <linkerbot/hand/api/config.hpp>
#include <linkerbot/hand/l6/hop_protocol.hpp>
#include <linkerbot/hand/l6/joints.hpp>

namespace linkerbot::hand {
    DeviceCapabilities declared_capabilities(const HandDeviceConfig &config) {
        if (config.o30_layout != O30Layout::kMountedV36 && config.o30_layout != O30Layout::kPythonV25) {
            throw ValidationError("unknown O30 layout");
        }
        if (config.o30_layout == O30Layout::kPythonV25 && config.model != Model::kO30) {
            throw ValidationError("V25 layout is only valid for O30");
        }
        if (config.l6_protocol == L6Protocol::kClassic) {
            auto result = declared_capabilities(config.model, config.side);
            if (config.o30_layout == O30Layout::kPythonV25) {
                if (config.model != Model::kO30) {
                    throw ValidationError("V25 layout is only valid for O30");
                }
                if (config.side != HandSide::kLeft && config.side != HandSide::kRight) {
                    throw ValidationError("O30 V25 requires an explicit side");
                }
                const auto request_id = config.address.value_or(config.side == HandSide::kLeft ? 2U : 1U);
                if (request_id > 0x3FE) {
                    throw ValidationError("O30 V25 request ID must be 0..0x3FE");
                }
                result.transport.request_id = request_id;
                result.transport.response_id = request_id | 0x400U;
                using namespace capabilities_detail;
                const auto names = channels(result.joints);
                const auto u8 = u8_range();
                for (const auto semantic : {CommandSemantic::kTorqueLimits, CommandSemantic::kMotionTimes}) {
                    result.commands.push_back(command(semantic, names, u8, "u8_v25_object", "count", std::nullopt,
                                                      Confidence::kUnknown, AckSupport::kRequired));
                }
                for (const auto semantic : {TelemetrySemantic::kTorqueLimits, TelemetrySemantic::kMotionTimes}) {
                    auto channel = telemetry(semantic, names, ReadMode::kQuery, "u8_v25_object", {20}, u8, "count",
                                             std::nullopt, Confidence::kUnknown, false);
                    channel.freshness_ms = 3000;
                    result.telemetry.push_back(std::move(channel));
                }
            }
            return result;
        }
        if (config.model != Model::kL6 || config.l6_protocol != L6Protocol::kHopV004) {
            throw ValidationError("L6 HOP protocol is only valid for L6");
        }
        // Validate the enum even when no device I/O takes place.
        (void)declared_capabilities(config.model, config.side);
        using namespace capabilities_detail;
        DeviceCapabilities result;
        result.model = Model::kL6;
        result.side = config.side;
        const auto id = config.address.value_or(l6::hop::kDefaultRequestId);
        result.transport = {TransportKind::kCanFd, 1000000U, std::nullopt, false,
                            "L6 HOP 0.0.4 standard CAN FD; response ID = request ID | 0x400; no BRS"};
        result.transport.request_id = id;
        result.transport.response_id = l6::hop::response_id(id);
        result.transport.mtu = 64;
        const auto &legacy = declared_capabilities(Model::kL6, config.side);
        result.joints = legacy.joints;
        for (auto &joint : result.joints) {
            joint.raw_angle_range = {0, 65535, true, 1};
            joint.percent_mapping = PercentMapping{0, 65535, "raw_min", "raw_max", Confidence::kUnknown};
            joint.physical_angle_mapping.reset();
        }
        const auto names = channels(l6::kJointNames);
        result.commands = {angle_command(result.joints, "u16_le_hop_object", AckSupport::kRequired, true),
                           command(CommandSemantic::kSpeeds, names, u8_range(), "u8", "count", std::nullopt,
                                   Confidence::kUnknown, AckSupport::kRequired),
                           command(CommandSemantic::kCurrentLimits, names, u8_range(), "u8", "count", std::nullopt,
                                   Confidence::kUnknown, AckSupport::kRequired)};
        result.commands.front().unit_confidence = Confidence::kDocumented;
        result.telemetry = {
            angle_telemetry(result.joints, "u16_le_hop_object"),
            telemetry(TelemetrySemantic::kConfiguredSpeeds, names, ReadMode::kQuery, "u8", {6}, u8_range(), "count",
                      std::nullopt, Confidence::kUnknown, false),
            telemetry(TelemetrySemantic::kCurrentLimits, names, ReadMode::kQuery, "u8", {6}, u8_range(), "count",
                      std::nullopt, Confidence::kUnknown, false),
            telemetry(TelemetrySemantic::kCurrents, names, ReadMode::kQuery, "u8", {6}, u8_range(), "count"),
            telemetry(TelemetrySemantic::kTemperatures, names, ReadMode::kQuery, "u8", {6}, u8_range(), "count"),
            telemetry(TelemetrySemantic::kFaults, names, ReadMode::kQuery, "u8_bitmask", {6}, u8_range()),
            telemetry(TelemetrySemantic::kForce, finger_channels(), ReadMode::kQuery, "hop_tactile_descriptor", {5},
                      std::nullopt, std::nullopt, "shape and units are supplied by sensor metadata",
                      Confidence::kDocumented)};
        result.telemetry.front().unit_confidence = Confidence::kDocumented;
        for (auto &channel : result.telemetry) {
            channel.source = TelemetrySource::kDevice;
            const auto *old = legacy.telemetry_channel(channel.semantic);
            if (old) {
                channel.freshness_ms = old->freshness_ms;
            }
        }
        result.operations.read_device_info = true;
        result.operations.save_parameters = true;
        result.operations.acknowledged_operations = {DeviceOperation::kSaveParameters};
        return result;
    }
}
