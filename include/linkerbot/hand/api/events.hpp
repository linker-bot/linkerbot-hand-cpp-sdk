#pragma once

#include <chrono>
#include <variant>
#include <vector>

#include <linkerbot/hand/api/readings.hpp>
#include <linkerbot/hand/capabilities.hpp>

namespace linkerbot::hand {
    struct PollingChannel {
        TelemetrySemantic semantic = TelemetrySemantic::kAngles;
        std::chrono::milliseconds interval{100};
    };

    using PollingPlan = std::vector<PollingChannel>;

    using HandEventPayload =
        std::variant<AngleSample, MeasuredSpeedSample, ConfiguredSpeedSample, ConfiguredAccelerationSample,
                     CurrentSample, CurrentLimitSample, TorqueSample, TemperatureSample, ForceSample,
                     NormalForceResultantSample, FaultSample, VoltageSample, StallDetectionTimeSample,
                     StallThresholdSample, StallHoldCurrentSample, PositionKpSample, PositionKdSample, TorqueKdSample,
                     TorqueLimitSample, MotionTimeSample, LimitCompensationSample>;

    struct HandEvent {
        TelemetrySemantic semantic = TelemetrySemantic::kAngles;
        HandEventPayload sample;
    };
} // namespace linkerbot::hand
