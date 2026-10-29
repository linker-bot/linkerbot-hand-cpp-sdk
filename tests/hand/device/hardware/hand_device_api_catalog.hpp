#pragma once

#include "hand_device_hardware_fixture.hpp"
#include <array>
#include <functional>

namespace linkerbot::hand::hardware_test {
    using NumericSetter = CommandHandle (HandDevice::*)(const std::vector<double> &);
    using VectorReader = RawVectorSample (*)(HandDevice &, std::chrono::milliseconds);

    template <auto Getter> RawVectorSample read_vector(HandDevice &hand, std::chrono::milliseconds timeout) {
        return (hand.*Getter)(timeout);
    }

    struct VectorApi {
        const char *name;
        TelemetrySemantic semantic;
        VectorReader read;
    };

    inline const std::array<VectorApi, 11> vector_apis{{
#define HW_GET(Name, Semantic) {#Name, TelemetrySemantic::Semantic, read_vector<&HandDevice::Name>}
        HW_GET(get_speeds, kMeasuredSpeeds), HW_GET(get_configured_speeds, kConfiguredSpeeds),
        HW_GET(get_configured_accelerations, kConfiguredAccelerations), HW_GET(get_currents, kCurrents),
        HW_GET(get_current_limits, kCurrentLimits), HW_GET(get_torques, kTorques),
        HW_GET(get_temperatures, kTemperatures), HW_GET(get_voltages, kVoltages),
        HW_GET(get_stall_detection_times, kStallDetectionTimes),
        HW_GET(get_stall_thresholds, kStallThresholds), HW_GET(get_stall_hold_currents, kStallHoldCurrents)
#undef HW_GET
    }};

    struct ParameterApi {
        const char *name;
        const char *baseline_env;
        CommandSemantic semantic;
        NumericSetter set;
        TelemetrySemantic telemetry;
        VectorReader read;
    };

#define HW_PARAM(Set, Get, Command, Telemetry, Env)                                                                    \
    {#Set, Env, CommandSemantic::Command, &HandDevice::Set, TelemetrySemantic::Telemetry, read_vector<&HandDevice::Get>}

    inline const std::array<ParameterApi, 6> parameter_apis{
        {HW_PARAM(set_speeds, get_configured_speeds, kSpeeds, kConfiguredSpeeds, "HAND_DEVICE_BASELINE_SPEEDS"),
         HW_PARAM(set_accelerations, get_configured_accelerations, kAccelerations, kConfiguredAccelerations,
                  "HAND_DEVICE_BASELINE_ACCELERATIONS"),
         HW_PARAM(set_current_limits, get_current_limits, kCurrentLimits, kCurrentLimits,
                  "HAND_DEVICE_BASELINE_CURRENT_LIMITS"),
         HW_PARAM(set_stall_detection_times, get_stall_detection_times, kStallDetectionTimes, kStallDetectionTimes,
                  "HAND_DEVICE_BASELINE_STALL_TIMES"),
         HW_PARAM(set_stall_thresholds, get_stall_thresholds, kStallThresholds, kStallThresholds,
                  "HAND_DEVICE_BASELINE_STALL_THRESHOLDS"),
         HW_PARAM(set_stall_hold_currents, get_stall_hold_currents, kStallHoldCurrents, kStallHoldCurrents,
                  "HAND_DEVICE_BASELINE_STALL_HOLD_CURRENTS")}};
#undef HW_PARAM

    inline void require_vector_sample(const RawVectorSample &sample, const TelemetryCapability &channel,
                                      uint64_t generation) {
        require_meta(sample.meta, generation);
        REQUIRE(sample.raw.size() == channel.channels.size());
        for (const auto value : sample.raw) {
            REQUIRE(std::isfinite(value));
        }
        const auto expected =
            channel.device_measured ? TelemetryValueOrigin::kDeviceMeasured
            : (channel.read_mode == ReadMode::kCommandCache || channel.read_mode == ReadMode::kCommandResponse)
                ? TelemetryValueOrigin::kCommandCache
                : TelemetryValueOrigin::kDeviceConfigured;
        REQUIRE(sample.meta.origin == expected);
        if (channel.read_mode == ReadMode::kCommandCache) {
            REQUIRE(sample.meta.delivery == TelemetryDelivery::kCommandCache);
        }
        if (channel.read_mode == ReadMode::kCommandResponse) {
            REQUIRE(sample.meta.delivery == TelemetryDelivery::kCommandResponse);
        }
    }

    struct OperationApi {
        const char *name;
        DeviceOperation semantic;
        CommandHandle (HandDevice::*call)();
    };
    inline const std::array<OperationApi, 4> operation_apis{
        {{"clear_faults", DeviceOperation::kClearFaults, &HandDevice::clear_faults},
         {"enable", DeviceOperation::kEnable, &HandDevice::enable},
         {"disable", DeviceOperation::kDisable, &HandDevice::disable},
         {"save_parameters", DeviceOperation::kSaveParameters, &HandDevice::save_parameters}}};
}
