#include "detail/force.hpp"
#include "detail/freshness.hpp"
#include "detail/model_adapters.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include <linkerbot/hand/conversion.hpp>
#include <linkerbot/hand/errors.hpp>

namespace linkerbot::hand::unified_detail {
    namespace {
        template <typename Value, size_t Size>
        [[nodiscard]] std::vector<double> dynamic_angles(const std::array<Value, Size> &values) {
            std::vector<double> result;
            result.reserve(Size);
            for (const auto value : values) {
                result.push_back(static_cast<double>(value));
            }
            return result;
        }

        [[nodiscard]] std::vector<double> angle_values(const l6::AngleData &data) {
            return dynamic_angles(data.angle.to_array());
        }

        [[nodiscard]] std::vector<double> angle_values(const o6::AngleData &data) {
            return dynamic_angles(data.angle.to_array());
        }

        [[nodiscard]] std::vector<double> angle_values(const l20lite::AngleData &data) {
            return dynamic_angles(data.angle.to_array());
        }

        [[nodiscard]] std::vector<double> angle_values(const l20::AngleData &data) {
            return dynamic_angles(data.angle.to_array());
        }

        [[nodiscard]] std::vector<double> angle_values(const l30::AngleData &data) {
            return dynamic_angles(data.angles.values);
        }

        [[nodiscard]] std::vector<double> angle_values(const o30::AngleData &data) {
            return dynamic_angles(data.angles.values);
        }

        template <typename Value, size_t Size>
        [[nodiscard]] std::vector<double> numeric_values(const std::array<Value, Size> &values) {
            std::vector<double> result;
            result.reserve(Size);
            for (const auto value : values) {
                result.push_back(static_cast<double>(value));
            }
            return result;
        }

        template <typename Value, size_t Size>
        [[nodiscard]] std::array<double, Size> scaled_values(const std::array<Value, Size> &values, double scale) {
            std::array<double, Size> result{};
            for (size_t index = 0; index < Size; ++index) {
                result[index] = std::round(static_cast<double>(values[index]) * scale);
            }
            return result;
        }

        template <size_t Size>
        [[nodiscard]] std::array<double, Size> acceleration_raw(const std::array<float, Size> &percent) {
            std::array<double, Size> result{};
            for (size_t index = 0; index < Size; ++index) {
                result[index] = percent[index] >= 100.0F
                                    ? 0.0
                                    : std::round(1.0 + static_cast<double>(percent[index]) * 253.0 / 99.0);
            }
            return result;
        }

        struct SampleMetadata {
            TelemetryValueOrigin origin = TelemetryValueOrigin::kUnknown;
            TelemetryDelivery delivery = TelemetryDelivery::kUnknown;
        };

        [[nodiscard]] SampleMetadata sample_metadata(const DeviceCapabilities &capabilities,
                                                     TelemetrySemantic semantic) {
            const auto *channel = capabilities.telemetry_channel(semantic);
            if (!channel) {
                return {};
            }

            SampleMetadata metadata;
            if (channel->device_measured) {
                metadata.origin = TelemetryValueOrigin::kDeviceMeasured;
            } else if (channel->read_mode == ReadMode::kCommandCache ||
                       channel->read_mode == ReadMode::kCommandResponse) {
                metadata.origin = TelemetryValueOrigin::kCommandCache;
            } else {
                metadata.origin = TelemetryValueOrigin::kDeviceConfigured;
            }

            switch (channel->read_mode) {
            case ReadMode::kQuery:
                metadata.delivery = TelemetryDelivery::kQueryResponse;
                break;
            case ReadMode::kReport:
                metadata.delivery = TelemetryDelivery::kPeriodicReport;
                break;
            case ReadMode::kQueryAndReport:
                metadata.delivery = TelemetryDelivery::kQueryOrReport;
                break;
            case ReadMode::kCommandResponse:
                metadata.delivery = TelemetryDelivery::kCommandResponse;
                break;
            case ReadMode::kCommandCache:
                metadata.delivery = TelemetryDelivery::kCommandCache;
                break;
            }
            return metadata;
        }

        template <typename Data>
        [[nodiscard]] decltype(std::declval<const Data &>().sequence, uint64_t{}) sample_sequence(const Data &data,
                                                                                                  int) {
            return data.sequence;
        }

        template <typename Data> [[nodiscard]] uint64_t sample_sequence(const Data &, long) { return 0; }

        template <typename Data>
        [[nodiscard]] decltype(std::declval<const Data &>().generation, uint64_t{}) sample_generation(const Data &data,
                                                                                                      int) {
            return data.generation;
        }

        template <typename Data> [[nodiscard]] uint64_t sample_generation(const Data &, long) { return 0; }

        template <typename Data>
        [[nodiscard]] SampleMeta make_sample_meta(const Data &data, const DeviceCapabilities &capabilities,
                                                  TelemetrySemantic semantic, bool within_declared_range = true) {
            const auto metadata = sample_metadata(capabilities, semantic);
            SampleMeta result;
            result.sequence = sample_sequence(data, 0);
            result.collected_at = data.timestamp;
            result.age = std::chrono::steady_clock::duration::zero();
            result.valid = true;
            result.stale = false;
            result.connection_generation = sample_generation(data, 0);
            result.origin = metadata.origin;
            result.delivery = metadata.delivery;
            result.within_declared_range = within_declared_range;
            complete_freshness(result, capabilities.telemetry_channel(semantic), std::chrono::steady_clock::now());
            return result;
        }

        [[nodiscard]] std::vector<double> raw_from_percent_feedback(const std::vector<double> &percent,
                                                                    const DeviceCapabilities &capabilities) {
            std::vector<double> result;
            result.reserve(percent.size());
            for (size_t index = 0; index < percent.size(); ++index) {
                const auto &mapping = capabilities.joints[index].percent_mapping;
                if (!mapping) {
                    result.push_back(percent[index]);
                    continue;
                }
                auto raw =
                    mapping->raw_at_zero + percent[index] / 100.0 * (mapping->raw_at_hundred - mapping->raw_at_zero);
                if (capabilities.joints[index].raw_angle_range.integer_only) {
                    raw = raw >= 0.0 ? std::floor(raw + 0.5) : std::ceil(raw - 0.5);
                }
                result.push_back(raw);
            }
            return result;
        }

        [[nodiscard]] std::vector<double> raw_angle_values(const l6::AngleData &data, const DeviceCapabilities &) {
            return numeric_values(data.device_raw);
        }

        [[nodiscard]] std::vector<double> raw_angle_values(const o6::AngleData &data, const DeviceCapabilities &) {
            return numeric_values(data.device_raw);
        }

        template <typename Data>
        [[nodiscard]] std::vector<double> raw_angle_values(const Data &data, const DeviceCapabilities &capabilities) {
            return raw_from_percent_feedback(angle_values(data), capabilities);
        }

        template <typename Sample, typename Value, size_t Size>
        [[nodiscard]] Sample vector_sample(const std::array<Value, Size> &values,
                                           std::chrono::steady_clock::time_point timestamp,
                                           const DeviceCapabilities &capabilities, TelemetrySemantic semantic) {
            struct Timestamped {
                std::chrono::steady_clock::time_point timestamp;
            };
            Sample result;
            result.raw = numeric_values(values);
            result.meta = make_sample_meta(Timestamped{timestamp}, capabilities, semantic);
            return result;
        }

        template <typename Data>
        [[nodiscard]] AngleSample make_angle_sample(const Data &data, const DeviceCapabilities &capabilities) {
            const auto raw = raw_angle_values(data, capabilities);
            AngleSample sample;
            sample.raw = raw;
            sample.percent = raw_angles_to_percent(raw, capabilities);
            sample.physical_rad = raw_angles_to_physical_rad(raw, capabilities);
            sample.meta = make_sample_meta(data, capabilities, TelemetrySemantic::kAngles,
                                           raw_angles_within_declared_range(raw, capabilities));
            return sample;
        }

        template <typename Sample>
        [[nodiscard]] Sample l6_control_sample(const l6::RawControlData &data, const DeviceCapabilities &capabilities,
                                               TelemetrySemantic semantic) {
            Sample result;
            result.raw = numeric_values(data.values);
            const auto *channel = capabilities.telemetry_channel(semantic);
            const bool within_range = !channel || !channel->raw_range ||
                                      std::all_of(result.raw.begin(), result.raw.end(),
                                                  [&](double value) { return channel->raw_range->contains(value); });
            result.meta = make_sample_meta(data, capabilities, semantic, within_range);
            if (data.command_response) {
                result.meta.delivery = TelemetryDelivery::kCommandResponse;
            }
            return result;
        }

        template <typename Data, typename Raw>
        [[nodiscard]] FaultSample fault_sample(const Data &data, const Raw &raw,
                                               const DeviceCapabilities &capabilities) {
            FaultSample result;
            result.raw.reserve(raw.size());
            result.statuses.reserve(raw.size());
            for (size_t index = 0; index < raw.size(); ++index) {
                result.raw.push_back(static_cast<uint8_t>(raw[index]));
                result.statuses.push_back(data.status(index));
            }
            result.meta = make_sample_meta(data, capabilities, TelemetrySemantic::kFaults);
            return result;
        }

        template <typename Finger> void append_force_finger(std::vector<uint8_t> &output, const Finger &finger) {
            for (const auto &row : finger.values) {
                output.insert(output.end(), row.begin(), row.end());
            }
        }

        template <typename AllFingers>
        [[nodiscard]] std::chrono::steady_clock::time_point force_timestamp(const AllFingers &data) {
            return std::max({data.thumb.timestamp, data.index.timestamp, data.middle.timestamp, data.ring.timestamp,
                             data.pinky.timestamp});
        }

        template <typename AllFingers>
        [[nodiscard]] ForceSample force_sample(const AllFingers &data, const DeviceCapabilities &capabilities) {
            ForceSample result;
            append_force_finger(result.raw, data.thumb);
            append_force_finger(result.raw, data.index);
            append_force_finger(result.raw, data.middle);
            append_force_finger(result.raw, data.ring);
            append_force_finger(result.raw, data.pinky);
            result.shape = {5, data.thumb.values.size(), data.thumb.values.front().size()};
            struct Timestamped {
                std::chrono::steady_clock::time_point timestamp;
                uint64_t sequence;
                uint64_t generation;
            };
            result.meta = make_sample_meta(
                Timestamped{force_timestamp(data), sample_sequence(data, 0), sample_generation(data, 0)}, capabilities,
                TelemetrySemantic::kForce);
            ForceFieldSample field;
            field.name = "fn";
            field.meta = result.meta;
            const auto capability = capabilities.force_field("fn");
            if (capability.raw_scale_note) {
                field.scale_note = std::string{*capability.raw_scale_note};
            }
            size_t index = 0;
            const auto append = [&](const auto &finger) {
                TactileMatrix matrix;
                matrix.name = capability.sensor_names.at(index++);
                matrix.rows = finger.values.size();
                matrix.columns = finger.values.front().size();
                std::vector<double> values;
                for (const auto &row : finger.values) {
                    matrix.raw.emplace_back(row.begin(), row.end());
                    values.insert(values.end(), row.begin(), row.end());
                }
                field.sensor_names.push_back(matrix.name);
                field.valid_values.push_back(std::vector<bool>(values.size(), true));
                field.raw.push_back(std::move(values));
                field.units.push_back(capability.raw_unit ? std::optional<std::string>{*capability.raw_unit}
                                                          : std::nullopt);
                field.encodings.emplace_back("u8");
                result.tactile.push_back(std::move(matrix));
            };
            append(data.thumb);
            append(data.index);
            append(data.middle);
            append(data.ring);
            append(data.pinky);
            result.fields.push_back(std::move(field));
            for (const auto &value : capabilities.force_fields()) {
                if (value.supported == false) {
                    result.unsupported_fields.emplace_back(value.name);
                }
            }
            return result;
        }

        template <typename Sample>
        [[nodiscard]] std::optional<double> typed_value_at(const std::optional<Sample> &sample, size_t index) {
            if (!sample || index >= sample->raw.size()) {
                return std::nullopt;
            }
            return sample->raw[index];
        }

        void populate_joint_states(HandSnapshot &snapshot, const DeviceCapabilities &capabilities) {
            snapshot.joints.clear();
            snapshot.joints.reserve(capabilities.joints.size());
            for (size_t index = 0; index < capabilities.joints.size(); ++index) {
                JointState joint;
                joint.index = index;
                joint.name = capabilities.joints[index].name;
                if (snapshot.angles && index < snapshot.angles->raw.size()) {
                    joint.angle_raw = snapshot.angles->raw[index];
                }
                if (snapshot.angles && index < snapshot.angles->percent.size()) {
                    joint.angle_percent = snapshot.angles->percent[index];
                }
                if (snapshot.angles && index < snapshot.angles->physical_rad.size()) {
                    joint.physical_rad = snapshot.angles->physical_rad[index];
                }
                joint.measured_speed_raw = typed_value_at(snapshot.measured_speeds, index);
                joint.configured_speed_raw = typed_value_at(snapshot.configured_speeds, index);
                joint.configured_acceleration_raw = typed_value_at(snapshot.configured_accelerations, index);
                joint.current_raw = typed_value_at(snapshot.currents, index);
                joint.current_limit_raw = typed_value_at(snapshot.current_limits, index);
                joint.position_kp_raw = typed_value_at(snapshot.position_kp, index);
                joint.position_kd_raw = typed_value_at(snapshot.position_kd, index);
                joint.torque_kd_raw = typed_value_at(snapshot.torque_kd, index);
                joint.torque_limits_raw = typed_value_at(snapshot.torque_limits, index);
                joint.motion_times_raw = typed_value_at(snapshot.motion_times, index);
                joint.limit_compensations_raw = typed_value_at(snapshot.limit_compensations, index);
                joint.torque_raw = typed_value_at(snapshot.torques, index);
                joint.temperature_raw = typed_value_at(snapshot.temperatures, index);
                joint.voltage_raw = typed_value_at(snapshot.voltages, index);
                joint.stall_detection_time_raw = typed_value_at(snapshot.stall_detection_times, index);
                joint.stall_threshold_raw = typed_value_at(snapshot.stall_thresholds, index);
                joint.stall_hold_current_raw = typed_value_at(snapshot.stall_hold_currents, index);
                if (snapshot.faults && index < snapshot.faults->raw.size()) {
                    joint.fault_raw = snapshot.faults->raw[index];
                }
                snapshot.joints.push_back(joint);
            }
        }

        template <typename ConcreteSnapshot>
        [[nodiscard]] HandSnapshot snapshot_base(const ConcreteSnapshot &source, Model model, HandSide side,
                                                 const DeviceCapabilities &capabilities) {
            HandSnapshot result;
            result.model = model;
            result.side = side;
            if (source.angle) {
                result.angles = angle_sample(*source.angle, capabilities);
            }
            result.captured_at = source.timestamp;
            return result;
        }
    } // namespace

    AngleSample angle_sample(const l6::AngleData &data, const DeviceCapabilities &capabilities) {
        return make_angle_sample(data, capabilities);
    }

    AngleSample angle_sample(const o6::AngleData &data, const DeviceCapabilities &capabilities) {
        return make_angle_sample(data, capabilities);
    }

    AngleSample angle_sample(const l20lite::AngleData &data, const DeviceCapabilities &capabilities) {
        return make_angle_sample(data, capabilities);
    }

    AngleSample angle_sample(const l20::AngleData &data, const DeviceCapabilities &capabilities) {
        return make_angle_sample(data, capabilities);
    }

    AngleSample angle_sample(const l30::AngleData &data, const DeviceCapabilities &capabilities) {
        return make_angle_sample(data, capabilities);
    }

    AngleSample angle_sample(const o30::AngleData &data, const DeviceCapabilities &capabilities) {
        return make_angle_sample(data, capabilities);
    }

    [[nodiscard]] HandSnapshot unified_snapshot(const l6::Snapshot &source, Model model, HandSide side,
                                                const DeviceCapabilities &capabilities) {
        auto result = snapshot_base(source, model, side, capabilities);
        if (source.position_kp) {
            result.position_kp =
                l6_control_sample<PositionKpSample>(*source.position_kp, capabilities, TelemetrySemantic::kPositionKp);
        }
        if (source.position_kd) {
            result.position_kd =
                l6_control_sample<PositionKdSample>(*source.position_kd, capabilities, TelemetrySemantic::kPositionKd);
        }
        if (source.torque_kd) {
            result.torque_kd =
                l6_control_sample<TorqueKdSample>(*source.torque_kd, capabilities, TelemetrySemantic::kTorqueKd);
        }
        if (source.limit_compensations) {
            result.limit_compensations = l6_control_sample<LimitCompensationSample>(
                *source.limit_compensations, capabilities, TelemetrySemantic::kLimitCompensations);
        }
        if (source.configured_speeds) {
            result.configured_speeds = l6_control_sample<ConfiguredSpeedSample>(*source.configured_speeds, capabilities,
                                                                                TelemetrySemantic::kConfiguredSpeeds);
        }
        if (source.current) {
            result.currents =
                vector_sample<CurrentSample>(source.current->current.to_array(), source.current->timestamp,
                                             capabilities, TelemetrySemantic::kCurrents);
        }
        if (source.current_limit) {
            result.current_limits = vector_sample<CurrentLimitSample>(source.current_limit->current_limit.to_array(),
                                                                      source.current_limit->timestamp, capabilities,
                                                                      TelemetrySemantic::kCurrentLimits);
        }
        if (source.torque) {
            result.torques = vector_sample<TorqueSample>(source.torque->torque.to_array(), source.torque->timestamp,
                                                         capabilities, TelemetrySemantic::kTorques);
        }
        if (source.temperature) {
            result.temperatures = vector_sample<TemperatureSample>(source.temperature->temperature.to_array(),
                                                                   source.temperature->timestamp, capabilities,
                                                                   TelemetrySemantic::kTemperatures);
        }
        if (source.fault) {
            result.faults = fault_sample(*source.fault, source.fault->fault.to_raw(), capabilities);
        }
        if (source.force_sensor) {
            result.force = force_sample(*source.force_sensor, capabilities);
        }
        if (source.stall_time) {
            result.stall_detection_times = vector_sample<StallDetectionTimeSample>(
                scaled_values(source.stall_time->time.to_array(), 0.1), source.stall_time->timestamp, capabilities,
                TelemetrySemantic::kStallDetectionTimes);
        }
        if (source.stall_threshold) {
            result.stall_thresholds = vector_sample<StallThresholdSample>(
                scaled_values(source.stall_threshold->limit.to_array(), 255.0 / 1400.0),
                source.stall_threshold->timestamp, capabilities, TelemetrySemantic::kStallThresholds);
        }
        if (source.stall_hold_current) {
            result.stall_hold_currents = vector_sample<StallHoldCurrentSample>(
                scaled_values(source.stall_hold_current->limit.to_array(), 255.0 / 1400.0),
                source.stall_hold_current->timestamp, capabilities, TelemetrySemantic::kStallHoldCurrents);
        }
        populate_joint_states(result, capabilities);
        return result;
    }

    [[nodiscard]] HandSnapshot unified_snapshot(const o6::Snapshot &source, Model model, HandSide side,
                                                const DeviceCapabilities &capabilities) {
        auto result = snapshot_base(source, model, side, capabilities);
        if (source.speed) {
            result.configured_speeds = vector_sample<ConfiguredSpeedSample>(
                scaled_values(source.speed->speed.to_array(), 255.0 / 100.0), source.speed->timestamp, capabilities,
                TelemetrySemantic::kConfiguredSpeeds);
        }
        if (source.acceleration) {
            result.configured_accelerations = vector_sample<ConfiguredAccelerationSample>(
                acceleration_raw(source.acceleration->acceleration.to_array()), source.acceleration->timestamp,
                capabilities, TelemetrySemantic::kConfiguredAccelerations);
        }
        if (source.current) {
            result.currents = vector_sample<CurrentSample>(source.current->currents, source.current->timestamp,
                                                           capabilities, TelemetrySemantic::kCurrents);
        }
        if (source.current_limit) {
            result.current_limits =
                vector_sample<CurrentLimitSample>(source.current_limit->current_limits, source.current_limit->timestamp,
                                                  capabilities, TelemetrySemantic::kCurrentLimits);
        }
        if (source.temperature) {
            result.temperatures = vector_sample<TemperatureSample>(source.temperature->temperature.to_array(),
                                                                   source.temperature->timestamp, capabilities,
                                                                   TelemetrySemantic::kTemperatures);
        }
        if (source.fault) {
            result.faults = fault_sample(*source.fault, source.fault->fault.to_raw(), capabilities);
        }
        if (source.force_sensor) {
            result.force = force_sample(*source.force_sensor, capabilities);
        }
        if (source.stall_time) {
            result.stall_detection_times = vector_sample<StallDetectionTimeSample>(
                scaled_values(source.stall_time->time.to_array(), 0.1), source.stall_time->timestamp, capabilities,
                TelemetrySemantic::kStallDetectionTimes);
        }
        if (source.stall_threshold) {
            result.stall_thresholds = vector_sample<StallThresholdSample>(
                scaled_values(source.stall_threshold->limit.to_array(), 255.0 / 1000.0),
                source.stall_threshold->timestamp, capabilities, TelemetrySemantic::kStallThresholds);
        }
        if (source.stall_hold_current) {
            result.stall_hold_currents = vector_sample<StallHoldCurrentSample>(
                scaled_values(source.stall_hold_current->limit.to_array(), 255.0 / 1000.0),
                source.stall_hold_current->timestamp, capabilities, TelemetrySemantic::kStallHoldCurrents);
        }
        populate_joint_states(result, capabilities);
        return result;
    }

    [[nodiscard]] HandSnapshot unified_snapshot(const l20lite::Snapshot &source, Model model, HandSide side,
                                                const DeviceCapabilities &capabilities) {
        auto result = snapshot_base(source, model, side, capabilities);
        if (source.speed) {
            result.measured_speeds = vector_sample<MeasuredSpeedSample>(
                scaled_values(source.speed->speed.to_array(), 255.0 / 100.0), source.speed->timestamp, capabilities,
                TelemetrySemantic::kMeasuredSpeeds);
        }
        if (source.configured_speeds) {
            result.configured_speeds = vector_sample<ConfiguredSpeedSample>(
                scaled_values(source.configured_speeds->speed.to_array(), 255.0 / 100.0),
                source.configured_speeds->timestamp, capabilities, TelemetrySemantic::kConfiguredSpeeds);
        }
        if (source.acceleration) {
            result.configured_accelerations = vector_sample<ConfiguredAccelerationSample>(
                scaled_values(source.acceleration->acceleration.to_array(), 254.0 / 100.0),
                source.acceleration->timestamp, capabilities, TelemetrySemantic::kConfiguredAccelerations);
        }
        if (source.current) {
            result.currents = vector_sample<CurrentSample>(source.current->currents, source.current->timestamp,
                                                           capabilities, TelemetrySemantic::kCurrents);
        }
        if (source.current_limit) {
            result.current_limits =
                vector_sample<CurrentLimitSample>(source.current_limit->current_limits, source.current_limit->timestamp,
                                                  capabilities, TelemetrySemantic::kCurrentLimits);
        }
        if (source.temperature) {
            result.temperatures = vector_sample<TemperatureSample>(source.temperature->temperature.to_array(),
                                                                   source.temperature->timestamp, capabilities,
                                                                   TelemetrySemantic::kTemperatures);
        }
        if (source.fault) {
            result.faults = fault_sample(*source.fault, source.fault->fault.to_raw(), capabilities);
        }
        if (source.force_sensor) {
            result.force = force_sample(*source.force_sensor, capabilities);
        }
        if (source.tip_force) {
            auto sample = tip_force_sample(
                *source.tip_force, make_sample_meta(*source.tip_force, capabilities, TelemetrySemantic::kForceFns),
                capabilities);
            result.normal_force_resultant = normal_force_resultant(sample);
            if (result.force) {
                merge_force_samples(*result.force, sample);
            } else {
                result.force = std::move(sample);
            }
        }
        if (source.stall_time) {
            result.stall_detection_times = vector_sample<StallDetectionTimeSample>(
                scaled_values(source.stall_time->time.to_array(), 0.1), source.stall_time->timestamp, capabilities,
                TelemetrySemantic::kStallDetectionTimes);
        }
        if (source.stall_threshold) {
            result.stall_thresholds = vector_sample<StallThresholdSample>(
                scaled_values(source.stall_threshold->limit.to_array(), 255.0 / 100.0),
                source.stall_threshold->timestamp, capabilities, TelemetrySemantic::kStallThresholds);
        }
        if (source.stall_hold_current) {
            result.stall_hold_currents = vector_sample<StallHoldCurrentSample>(
                scaled_values(source.stall_hold_current->limit.to_array(), 255.0 / 100.0),
                source.stall_hold_current->timestamp, capabilities, TelemetrySemantic::kStallHoldCurrents);
        }
        populate_joint_states(result, capabilities);
        return result;
    }

    [[nodiscard]] HandSnapshot unified_snapshot(const l20::Snapshot &source, Model model, HandSide side,
                                                const DeviceCapabilities &capabilities) {
        auto result = snapshot_base(source, model, side, capabilities);
        if (source.speed) {
            result.measured_speeds = vector_sample<MeasuredSpeedSample>(
                scaled_values(source.speed->speed.to_array(), 255.0 / 100.0), source.speed->timestamp, capabilities,
                TelemetrySemantic::kMeasuredSpeeds);
        }
        if (source.current) {
            result.currents = vector_sample<CurrentSample>(source.current->currents, source.current->timestamp,
                                                           capabilities, TelemetrySemantic::kCurrents);
        }
        if (source.temperature) {
            result.temperatures = vector_sample<TemperatureSample>(source.temperature->temperature.to_array(),
                                                                   source.temperature->timestamp, capabilities,
                                                                   TelemetrySemantic::kTemperatures);
        }
        if (source.fault) {
            result.faults = fault_sample(*source.fault, source.fault->fault.to_raw(), capabilities);
        }
        if (source.force_sensor) {
            result.force = force_sample(*source.force_sensor, capabilities);
        }
        populate_joint_states(result, capabilities);
        return result;
    }

    [[nodiscard]] HandSnapshot unified_snapshot(const l30::Snapshot &source, Model model, HandSide side,
                                                const DeviceCapabilities &capabilities) {
        auto result = snapshot_base(source, model, side, capabilities);
        if (source.configured_speeds) {
            result.configured_speeds = vector_sample<ConfiguredSpeedSample>(
                source.configured_speeds->speeds, source.configured_speeds->timestamp, capabilities,
                TelemetrySemantic::kConfiguredSpeeds);
        }
        if (source.speed) {
            result.measured_speeds = vector_sample<MeasuredSpeedSample>(
                source.speed->speeds, source.speed->timestamp, capabilities, TelemetrySemantic::kMeasuredSpeeds);
        }
        if (source.current) {
            result.currents = vector_sample<CurrentSample>(source.current->currents, source.current->timestamp,
                                                           capabilities, TelemetrySemantic::kCurrents);
        }
        if (source.current_limit) {
            result.current_limits =
                vector_sample<CurrentLimitSample>(source.current_limit->current_limits, source.current_limit->timestamp,
                                                  capabilities, TelemetrySemantic::kCurrentLimits);
        }
        if (source.temperature) {
            result.temperatures =
                vector_sample<TemperatureSample>(source.temperature->temperatures, source.temperature->timestamp,
                                                 capabilities, TelemetrySemantic::kTemperatures);
        }
        if (source.fault) {
            result.faults = fault_sample(*source.fault, source.fault->faults, capabilities);
        }
        if (source.force_sensor) {
            result.force = force_sample(*source.force_sensor, capabilities);
        }
        populate_joint_states(result, capabilities);
        return result;
    }

    [[nodiscard]] HandSnapshot unified_snapshot(const o30::Snapshot &source, Model model, HandSide side,
                                                const DeviceCapabilities &capabilities) {
        auto result = snapshot_base(source, model, side, capabilities);
        if (source.speed) {
            if (source.speed->kind == o30::ValueKind::kSetpoint) {
                result.configured_speeds = vector_sample<ConfiguredSpeedSample>(
                    source.speed->speeds, source.speed->timestamp, capabilities, TelemetrySemantic::kConfiguredSpeeds);
            } else {
                result.measured_speeds = vector_sample<MeasuredSpeedSample>(
                    source.speed->speeds, source.speed->timestamp, capabilities, TelemetrySemantic::kMeasuredSpeeds);
            }
        }
        if (source.acceleration) {
            result.configured_accelerations = vector_sample<ConfiguredAccelerationSample>(
                source.acceleration->accelerations, source.acceleration->timestamp, capabilities,
                TelemetrySemantic::kConfiguredAccelerations);
        }
        if (source.current) {
            result.currents = vector_sample<CurrentSample>(source.current->currents, source.current->timestamp,
                                                           capabilities, TelemetrySemantic::kCurrents);
        }
        if (source.current_limit) {
            result.current_limits =
                vector_sample<CurrentLimitSample>(source.current_limit->current_limits, source.current_limit->timestamp,
                                                  capabilities, TelemetrySemantic::kCurrentLimits);
        }
        if (source.torque) {
            result.torques = vector_sample<TorqueSample>(source.torque->torques, source.torque->timestamp, capabilities,
                                                         TelemetrySemantic::kTorques);
        }
        if (source.temperature) {
            result.temperatures =
                vector_sample<TemperatureSample>(source.temperature->temperatures, source.temperature->timestamp,
                                                 capabilities, TelemetrySemantic::kTemperatures);
        }
        if (source.voltage) {
            result.voltages = vector_sample<VoltageSample>(source.voltage->voltages, source.voltage->timestamp,
                                                           capabilities, TelemetrySemantic::kVoltages);
        }
        if (source.stall_time) {
            result.stall_detection_times =
                vector_sample<StallDetectionTimeSample>(source.stall_time->times, source.stall_time->timestamp,
                                                        capabilities, TelemetrySemantic::kStallDetectionTimes);
        }
        if (source.stall_threshold) {
            result.stall_thresholds = vector_sample<StallThresholdSample>(
                source.stall_threshold->thresholds, source.stall_threshold->timestamp, capabilities,
                TelemetrySemantic::kStallThresholds);
        }
        if (source.stall_hold_current) {
            result.stall_hold_currents = vector_sample<StallHoldCurrentSample>(
                source.stall_hold_current->currents, source.stall_hold_current->timestamp, capabilities,
                TelemetrySemantic::kStallHoldCurrents);
        }
        if (source.fault) {
            result.faults = fault_sample(*source.fault, source.fault->faults, capabilities);
        }
        if (source.force_sensor) {
            result.force = descriptor_force_sample(
                *source.force_sensor, make_sample_meta(*source.force_sensor, capabilities, TelemetrySemantic::kForce));
            if (const auto *field = result.force->field("fns");
                field && std::all_of(field->raw.begin(), field->raw.end(),
                                     [](const auto &finger) { return finger && finger->size() == 1; })) {
                result.normal_force_resultant = normal_force_resultant(*result.force);
            }
        }
        populate_joint_states(result, capabilities);
        return result;
    }
    TelemetrySample telemetry_sample(const HandSnapshot &snapshot, TelemetrySemantic semantic) {
#define LINKERBOT_RETURN_SNAPSHOT_SAMPLE(Semantic, Member)                                                             \
    case TelemetrySemantic::Semantic:                                                                                  \
        if (snapshot.Member) {                                                                                         \
            return *snapshot.Member;                                                                                   \
        }                                                                                                              \
        break

        switch (semantic) {
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kPositionKp, position_kp);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kPositionKd, position_kd);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kTorqueKd, torque_kd);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kTorqueLimits, torque_limits);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kMotionTimes, motion_times);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kLimitCompensations, limit_compensations);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kAngles, angles);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kMeasuredSpeeds, measured_speeds);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kConfiguredSpeeds, configured_speeds);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kConfiguredAccelerations, configured_accelerations);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kCurrents, currents);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kCurrentLimits, current_limits);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kTorques, torques);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kTemperatures, temperatures);
        case TelemetrySemantic::kForceFns:
        case TelemetrySemantic::kForceFts:
        case TelemetrySemantic::kForceFds:
        case TelemetrySemantic::kForcePx:
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kForce, force);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kFaults, faults);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kVoltages, voltages);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kStallDetectionTimes, stall_detection_times);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kStallThresholds, stall_thresholds);
            LINKERBOT_RETURN_SNAPSHOT_SAMPLE(kStallHoldCurrents, stall_hold_currents);
        default:
            break;
        }
#undef LINKERBOT_RETURN_SNAPSHOT_SAMPLE
        throw DeviceNotReadyError("telemetry cache has not been populated for this semantic");
    }
} // namespace linkerbot::hand::unified_detail
