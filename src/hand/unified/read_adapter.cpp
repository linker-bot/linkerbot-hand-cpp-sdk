#include "detail/force.hpp"
#include "detail/model_adapters.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <linkerbot/hand/errors.hpp>

namespace linkerbot::hand::unified_detail {
    namespace {
        void require_timeout(std::chrono::milliseconds timeout) {
            if (timeout.count() <= 0) {
                throw ValidationError("telemetry timeout must be positive");
            }
        }

        template <typename Device> [[nodiscard]] TelemetrySample projected(Device &device, TelemetrySemantic semantic) {
            return telemetry_sample(
                unified_snapshot(device.get_snapshot(), Model::kUnknown, HandSide::kUnknown, device.capabilities()),
                semantic);
        }

        [[nodiscard]] SampleMeta sample_meta(std::chrono::steady_clock::time_point timestamp,
                                             const DeviceCapabilities &capabilities, TelemetrySemantic semantic) {
            SampleMeta meta;
            meta.collected_at = timestamp;
            meta.age = std::chrono::steady_clock::duration::zero();
            meta.valid = true;
            meta.stale = false;
            if (const auto *channel = capabilities.telemetry_channel(semantic)) {
                if (channel->device_measured) {
                    meta.origin = TelemetryValueOrigin::kDeviceMeasured;
                } else if (channel->read_mode == ReadMode::kCommandCache ||
                           channel->read_mode == ReadMode::kCommandResponse) {
                    meta.origin = TelemetryValueOrigin::kCommandCache;
                } else {
                    meta.origin = TelemetryValueOrigin::kDeviceConfigured;
                }
                switch (channel->read_mode) {
                case ReadMode::kQuery:
                    meta.delivery = TelemetryDelivery::kQueryResponse;
                    break;
                case ReadMode::kReport:
                    meta.delivery = TelemetryDelivery::kPeriodicReport;
                    break;
                case ReadMode::kQueryAndReport:
                    meta.delivery = TelemetryDelivery::kQueryOrReport;
                    break;
                case ReadMode::kCommandResponse:
                    meta.delivery = TelemetryDelivery::kCommandResponse;
                    break;
                case ReadMode::kCommandCache:
                    meta.delivery = TelemetryDelivery::kCommandCache;
                    break;
                }
            }
            return meta;
        }

        [[nodiscard]] MeasuredSpeedSample l20lite_measured_speed(const l20lite::SpeedData &data,
                                                                 const DeviceCapabilities &capabilities) {
            MeasuredSpeedSample sample;
            for (const auto value : data.speed.to_array()) {
                sample.raw.push_back(std::round(value * 255.0 / 100.0));
            }
            sample.meta = sample_meta(data.timestamp, capabilities, TelemetrySemantic::kMeasuredSpeeds);
            return sample;
        }

        template <typename Sample, typename Value, size_t Size>
        [[nodiscard]] Sample raw_vector_sample(const std::array<Value, Size> &values,
                                               std::chrono::steady_clock::time_point timestamp,
                                               const DeviceCapabilities &capabilities, TelemetrySemantic semantic) {
            Sample sample;
            sample.raw.reserve(Size);
            for (const auto value : values) {
                sample.raw.push_back(static_cast<double>(value));
            }
            sample.meta = sample_meta(timestamp, capabilities, semantic);
            return sample;
        }

        [[nodiscard]] TelemetrySample l20lite_tip_force(const l20lite::TipForceData &data, TelemetrySemantic semantic,
                                                        const DeviceCapabilities &capabilities) {
            return tip_force_sample(data, sample_meta(data.timestamp, capabilities, semantic), capabilities);
        }
    } // namespace

    TelemetrySample read_telemetry(l6::L6 &device, TelemetrySemantic semantic, std::chrono::milliseconds timeout) {
        require_timeout(timeout);
        switch (semantic) {
        case TelemetrySemantic::kAngles:
            return angle_sample(device.angle.get_blocking(timeout), device.capabilities());
        case TelemetrySemantic::kConfiguredSpeeds:
            // L6 has no speed query. Return only a device write-response cache.
            if (!device.speed.get_snapshot()) {
                return ConfiguredSpeedSample{};
            }
            return projected(device, semantic);
        case TelemetrySemantic::kCurrentLimits:
            static_cast<void>(device.current_limit.get_blocking(timeout));
            break;
        case TelemetrySemantic::kTorques:
            static_cast<void>(device.torque.get_blocking(timeout));
            break;
        case TelemetrySemantic::kCurrents:
            static_cast<void>(device.current.get_blocking(timeout));
            break;
        case TelemetrySemantic::kTemperatures:
            static_cast<void>(device.temperature.get_blocking(timeout));
            break;
        case TelemetrySemantic::kFaults:
            static_cast<void>(device.fault.get_blocking(timeout));
            break;
        case TelemetrySemantic::kForce:
            static_cast<void>(device.force_sensor.get_blocking(timeout));
            break;
        case TelemetrySemantic::kStallDetectionTimes:
            static_cast<void>(device.stall.get_times_blocking(timeout));
            break;
        case TelemetrySemantic::kStallThresholds:
            static_cast<void>(device.stall.get_thresholds_blocking(timeout));
            break;
        case TelemetrySemantic::kStallHoldCurrents:
            static_cast<void>(device.stall.get_torques_blocking(timeout));
            break;
        default:
            throw UnsupportedOperationError("telemetry semantic is not implemented for L6");
        }
        return projected(device, semantic);
    }

    TelemetrySample read_telemetry(o6::O6 &device, TelemetrySemantic semantic, std::chrono::milliseconds timeout) {
        require_timeout(timeout);
        switch (semantic) {
        case TelemetrySemantic::kAngles:
            return angle_sample(device.angle.get_blocking(timeout), device.capabilities());
        case TelemetrySemantic::kConfiguredSpeeds:
            // A bare 0x05 is a zero-speed write on O6 V1.3.2. Use the
            // completed write response instead of sending a destructive query.
            return projected(device, semantic);
        case TelemetrySemantic::kConfiguredAccelerations:
            static_cast<void>(device.acceleration.get_blocking(timeout));
            break;
        case TelemetrySemantic::kCurrents:
            static_cast<void>(device.current.get_blocking(timeout));
            break;
        case TelemetrySemantic::kCurrentLimits:
            return projected(device, semantic);
        case TelemetrySemantic::kTemperatures:
            static_cast<void>(device.temperature.get_blocking(timeout));
            break;
        case TelemetrySemantic::kFaults:
            static_cast<void>(device.fault.get_blocking(timeout));
            break;
        case TelemetrySemantic::kForce:
            static_cast<void>(device.force_sensor.get_blocking(timeout));
            break;
        case TelemetrySemantic::kStallDetectionTimes:
            static_cast<void>(device.stall.get_times_blocking(timeout));
            break;
        case TelemetrySemantic::kStallThresholds:
            static_cast<void>(device.stall.get_thresholds_blocking(timeout));
            break;
        case TelemetrySemantic::kStallHoldCurrents:
            static_cast<void>(device.stall.get_torques_blocking(timeout));
            break;
        default:
            throw UnsupportedOperationError("telemetry semantic is not implemented for O6");
        }
        return projected(device, semantic);
    }

    TelemetrySample read_telemetry(l20lite::L20lite &device, TelemetrySemantic semantic,
                                   std::chrono::milliseconds timeout) {
        require_timeout(timeout);
        switch (semantic) {
        case TelemetrySemantic::kAngles:
            return angle_sample(device.angle.get_blocking(timeout), device.capabilities());
        case TelemetrySemantic::kConfiguredSpeeds:
        case TelemetrySemantic::kConfiguredAccelerations:
        case TelemetrySemantic::kCurrentLimits:
            return projected(device, semantic);
        case TelemetrySemantic::kMeasuredSpeeds:
            return l20lite_measured_speed(device.speed.get_blocking(timeout), device.capabilities());
        case TelemetrySemantic::kCurrents:
            static_cast<void>(device.current.get_blocking(timeout));
            break;
        case TelemetrySemantic::kTemperatures:
            static_cast<void>(device.temperature.get_blocking(timeout));
            break;
        case TelemetrySemantic::kFaults:
            static_cast<void>(device.fault.get_blocking(timeout));
            break;
        case TelemetrySemantic::kForce:
            static_cast<void>(device.force_sensor.get_blocking(timeout));
            break;
        case TelemetrySemantic::kForceFns:
        case TelemetrySemantic::kForceFts:
        case TelemetrySemantic::kForceFds:
        case TelemetrySemantic::kForcePx:
            return l20lite_tip_force(device.tip_force.get_blocking(timeout), semantic, device.capabilities());
        case TelemetrySemantic::kStallDetectionTimes:
            static_cast<void>(device.stall.get_times_blocking(timeout));
            break;
        case TelemetrySemantic::kStallThresholds:
            static_cast<void>(device.stall.get_thresholds_blocking(timeout));
            break;
        case TelemetrySemantic::kStallHoldCurrents:
            static_cast<void>(device.stall.get_torques_blocking(timeout));
            break;
        default:
            throw UnsupportedOperationError("telemetry semantic is not implemented for L20Lite");
        }
        return projected(device, semantic);
    }

    TelemetrySample read_telemetry(l20::L20 &device, TelemetrySemantic semantic, std::chrono::milliseconds timeout) {
        require_timeout(timeout);
        switch (semantic) {
        case TelemetrySemantic::kAngles:
            return angle_sample(device.angle.get_blocking(timeout), device.capabilities());
        case TelemetrySemantic::kMeasuredSpeeds:
            static_cast<void>(device.speed.get_blocking(timeout));
            break;
        case TelemetrySemantic::kCurrents:
            static_cast<void>(device.current.get_blocking(timeout));
            break;
        case TelemetrySemantic::kTemperatures:
            static_cast<void>(device.temperature.get_blocking(timeout));
            break;
        case TelemetrySemantic::kFaults:
            static_cast<void>(device.fault.get_blocking(timeout));
            break;
        case TelemetrySemantic::kForce:
            static_cast<void>(device.force_sensor.get_blocking(timeout));
            break;
        default:
            throw UnsupportedOperationError("telemetry semantic is not implemented for L20");
        }
        return projected(device, semantic);
    }

    TelemetrySample read_telemetry(l30::L30 &device, TelemetrySemantic semantic, std::chrono::milliseconds timeout) {
        require_timeout(timeout);
        switch (semantic) {
        case TelemetrySemantic::kAngles:
            return angle_sample(device.angle.get_blocking(timeout), device.capabilities());
        case TelemetrySemantic::kMeasuredSpeeds:
            static_cast<void>(device.speed.get_blocking(timeout));
            break;
        case TelemetrySemantic::kCurrents:
            static_cast<void>(device.current.get_blocking(timeout));
            break;
        case TelemetrySemantic::kCurrentLimits:
            return projected(device, semantic);
        case TelemetrySemantic::kConfiguredSpeeds:
            if (!device.speed.get_configured_snapshot()) {
                return ConfiguredSpeedSample{};
            }
            return projected(device, semantic);
        case TelemetrySemantic::kTemperatures:
            static_cast<void>(device.temperature.get_blocking(timeout));
            break;
        case TelemetrySemantic::kFaults:
            static_cast<void>(device.fault.get_blocking(timeout));
            break;
        case TelemetrySemantic::kForce:
            static_cast<void>(device.force_sensor.get_blocking(timeout));
            break;
        default:
            throw UnsupportedOperationError("telemetry semantic is not implemented for L30");
        }
        return projected(device, semantic);
    }

    TelemetrySample read_telemetry(o30::O30 &device, TelemetrySemantic semantic, std::chrono::milliseconds timeout) {
        require_timeout(timeout);
        switch (semantic) {
        case TelemetrySemantic::kAngles:
            return angle_sample(device.angle.get_blocking(timeout), device.capabilities());
        case TelemetrySemantic::kMeasuredSpeeds: {
            const auto data = device.speed.get_blocking(timeout, o30::ValueKind::kActual);
            return raw_vector_sample<MeasuredSpeedSample>(data.speeds, data.timestamp, device.capabilities(), semantic);
        }
        case TelemetrySemantic::kConfiguredSpeeds: {
            const auto data = device.speed.get_blocking(timeout, o30::ValueKind::kSetpoint);
            return raw_vector_sample<ConfiguredSpeedSample>(data.speeds, data.timestamp, device.capabilities(),
                                                            semantic);
        }
        case TelemetrySemantic::kConfiguredAccelerations: {
            const auto data = device.acceleration.get_blocking(timeout, o30::ValueKind::kSetpoint);
            return raw_vector_sample<ConfiguredAccelerationSample>(data.accelerations, data.timestamp,
                                                                   device.capabilities(), semantic);
        }
        case TelemetrySemantic::kCurrents: {
            const auto data = device.current.get_currents(timeout);
            return raw_vector_sample<CurrentSample>(data.currents, data.timestamp, device.capabilities(), semantic);
        }
        case TelemetrySemantic::kCurrentLimits: {
            const auto data = device.current.get_current_limits(timeout);
            return raw_vector_sample<CurrentLimitSample>(data.current_limits, data.timestamp, device.capabilities(),
                                                         semantic);
        }
        case TelemetrySemantic::kTorques: {
            const auto data = device.torque.get_torques(timeout);
            return raw_vector_sample<TorqueSample>(data.torques, data.timestamp, device.capabilities(), semantic);
        }
        case TelemetrySemantic::kTemperatures: {
            const auto data = device.temperature.get_blocking(timeout, o30::ValueKind::kActual);
            return raw_vector_sample<TemperatureSample>(data.temperatures, data.timestamp, device.capabilities(),
                                                        semantic);
        }
        case TelemetrySemantic::kVoltages: {
            const auto data = device.voltage.get_blocking(timeout, o30::ValueKind::kActual);
            return raw_vector_sample<VoltageSample>(data.voltages, data.timestamp, device.capabilities(), semantic);
        }
        case TelemetrySemantic::kFaults:
            static_cast<void>(device.fault.get_blocking(timeout, o30::ValueKind::kActual));
            break;
        case TelemetrySemantic::kForce: {
            const auto data = device.sensor.read_all_data(timeout);
            return descriptor_force_sample(data, sample_meta(data.timestamp, device.capabilities(), semantic));
        }
        case TelemetrySemantic::kStallDetectionTimes: {
            const auto data = device.stall.get_times(timeout, o30::ValueKind::kSetpoint);
            return raw_vector_sample<StallDetectionTimeSample>(data.times, data.timestamp, device.capabilities(),
                                                               semantic);
        }
        case TelemetrySemantic::kStallThresholds: {
            const auto data = device.stall.get_thresholds(timeout, o30::ValueKind::kSetpoint);
            return raw_vector_sample<StallThresholdSample>(data.thresholds, data.timestamp, device.capabilities(),
                                                           semantic);
        }
        case TelemetrySemantic::kStallHoldCurrents: {
            const auto data = device.stall.get_hold_currents(timeout, o30::ValueKind::kSetpoint);
            return raw_vector_sample<StallHoldCurrentSample>(data.currents, data.timestamp, device.capabilities(),
                                                             semantic);
        }
        default:
            throw UnsupportedOperationError("telemetry semantic is not implemented for O30");
        }
        return projected(device, semantic);
    }
} // namespace linkerbot::hand::unified_detail
