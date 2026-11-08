#pragma once

#include <chrono>
#include <string>
#include <variant>
#include <vector>

#include <linkerbot/hand/api/readings.hpp>
#include <linkerbot/hand/capabilities.hpp>

namespace linkerbot::hand {
    struct ForceFieldPolling {
        std::string field;
        std::chrono::milliseconds interval{100};
    };

    struct PollingChannel {
        TelemetrySemantic semantic = TelemetrySemantic::kAngles;
        std::chrono::milliseconds interval{100};
        // Used for angles by the explicit Python runtime policy only.
        AngleFeedbackPolicy angle_feedback = AngleFeedbackPolicy::kAdaptive;
        // Python-only field selection on kForce. Nonempty selections
        // use their own intervals, resolved into shared queries at the fastest
        // requested interval. Empty retains the existing query-channel API.
        std::vector<ForceFieldPolling> force_fields;
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
