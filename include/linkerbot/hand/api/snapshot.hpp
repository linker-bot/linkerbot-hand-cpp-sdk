#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include <linkerbot/hand/api/readings.hpp>
#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/hand/identity.hpp>

namespace linkerbot::hand {
    struct JointState {
        size_t index = 0;
        std::string_view name;
        std::optional<double> angle_raw;
        std::optional<double> angle_percent;
        std::optional<double> physical_rad;
        std::optional<double> measured_speed_raw;
        std::optional<double> configured_speed_raw;
        std::optional<double> configured_acceleration_raw;
        std::optional<double> current_raw;
        std::optional<double> current_limit_raw;
        std::optional<double> torque_raw;
        std::optional<double> temperature_raw;
        std::optional<double> voltage_raw;
        std::optional<double> stall_detection_time_raw;
        std::optional<double> stall_threshold_raw;
        std::optional<double> stall_hold_current_raw;
        std::optional<uint8_t> fault_raw;
        std::optional<double> position_kp_raw;
        std::optional<double> position_kd_raw;
        std::optional<double> torque_kd_raw;
        std::optional<double> torque_limits_raw;
        std::optional<double> motion_times_raw;
        std::optional<double> limit_compensations_raw;
    };

    struct HandSnapshot {
        Model model = Model::kUnknown;
        HandSide side = HandSide::kUnknown;
        std::chrono::steady_clock::time_point captured_at;
        uint64_t snapshot_sequence = 0;
        uint64_t connection_generation = 0;
        std::vector<JointState> joints;
        std::optional<DeviceIdentity> device_info;
        std::optional<AngleSample> angles;
        std::optional<MeasuredSpeedSample> measured_speeds;
        std::optional<ConfiguredSpeedSample> configured_speeds;
        std::optional<ConfiguredAccelerationSample> configured_accelerations;
        std::optional<CurrentSample> currents;
        std::optional<CurrentLimitSample> current_limits;
        std::optional<TorqueSample> torques;
        std::optional<TemperatureSample> temperatures;
        std::optional<ForceSample> force;
        std::optional<NormalForceResultantSample> normal_force_resultant;
        std::optional<FaultSample> faults;
        std::optional<VoltageSample> voltages;
        std::optional<StallDetectionTimeSample> stall_detection_times;
        std::optional<StallThresholdSample> stall_thresholds;
        std::optional<StallHoldCurrentSample> stall_hold_currents;
        std::optional<PositionKpSample> position_kp;
        std::optional<PositionKdSample> position_kd;
        std::optional<TorqueKdSample> torque_kd;
        std::optional<TorqueLimitSample> torque_limits;
        std::optional<MotionTimeSample> motion_times;
        std::optional<LimitCompensationSample> limit_compensations;
    };
} // namespace linkerbot::hand
