#include "detail/model_adapters.hpp"

#include <chrono>
#include <map>

#include <linkerbot/hand/errors.hpp>

namespace linkerbot::hand::unified_detail {
    namespace {
        void configure_l6_channel(l6::L6PollingPlan &target, const PollingChannel &channel) {
            const auto period = std::chrono::duration_cast<std::chrono::microseconds>(channel.interval);
            switch (channel.semantic) {
            case TelemetrySemantic::kAngles:
                target.angle_enabled = true;
                target.angle_period = period;
                return;
            case TelemetrySemantic::kCurrentLimits:
                target.current_limit_enabled = true;
                target.current_limit_period = period;
                return;
            case TelemetrySemantic::kTorques:
                target.torque_enabled = true;
                target.torque_period = period;
                return;
            case TelemetrySemantic::kTemperatures:
                target.temperature_enabled = true;
                target.temperature_period = period;
                return;
            case TelemetrySemantic::kCurrents:
                target.current_enabled = true;
                target.current_period = period;
                return;
            case TelemetrySemantic::kFaults:
                target.fault_enabled = true;
                target.fault_period = period;
                return;
            case TelemetrySemantic::kForce:
                target.tactile_enabled = true;
                target.tactile_period = period;
                return;
            default:
                break;
            }
            throw UnsupportedOperationError("telemetry is not pollable on L6");
        }
    } // namespace

    void start_device_polling(l6::L6 &device, const PollingPlan &plan) {
        l6::L6PollingPlan selected;
        for (const auto &channel : plan) {
            configure_l6_channel(selected, channel);
        }
        selected.request_timeout = std::chrono::milliseconds{100};
        selected.tactile_timeout = std::chrono::milliseconds{1000};
        device.start_polling(selected);
    }

    void start_device_polling(o6::O6 &device, const PollingPlan &plan) {
        std::map<o6::SensorSource, std::chrono::milliseconds> selected;
        for (const auto &channel : plan) {
            switch (channel.semantic) {
            case TelemetrySemantic::kAngles:
                selected[o6::SensorSource::kAngle] = channel.interval;
                break;
            case TelemetrySemantic::kCurrents:
                selected[o6::SensorSource::kCurrent] = channel.interval;
                break;
            case TelemetrySemantic::kTemperatures:
                selected[o6::SensorSource::kTemperature] = channel.interval;
                break;
            case TelemetrySemantic::kFaults:
                selected[o6::SensorSource::kFault] = channel.interval;
                break;
            case TelemetrySemantic::kForce:
                selected[o6::SensorSource::kForceSensor] = channel.interval;
                break;
            default:
                throw UnsupportedOperationError("telemetry is not pollable on O6");
            }
        }
        device.start_polling(selected);
    }

    void start_device_polling(l20lite::L20lite &device, const PollingPlan &plan) {
        std::map<l20lite::SensorSource, std::chrono::milliseconds> selected;
        for (const auto &channel : plan) {
            switch (channel.semantic) {
            case TelemetrySemantic::kMeasuredSpeeds:
                selected[l20lite::SensorSource::kSpeed] = channel.interval;
                break;
            case TelemetrySemantic::kAngles:
                selected[l20lite::SensorSource::kAngle] = channel.interval;
                break;
            case TelemetrySemantic::kCurrents:
                selected[l20lite::SensorSource::kCurrent] = channel.interval;
                break;
            case TelemetrySemantic::kTemperatures:
                selected[l20lite::SensorSource::kTemperature] = channel.interval;
                break;
            case TelemetrySemantic::kFaults:
                selected[l20lite::SensorSource::kFault] = channel.interval;
                break;
            case TelemetrySemantic::kForce:
                selected[l20lite::SensorSource::kForceSensor] = channel.interval;
                break;
            case TelemetrySemantic::kForceFns:
            case TelemetrySemantic::kForceFts:
            case TelemetrySemantic::kForceFds:
            case TelemetrySemantic::kForcePx: {
                const auto source = l20lite::SensorSource::kTipForce;
                const auto existing = selected.find(source);
                if (existing == selected.end() || channel.interval < existing->second) {
                    selected[source] = channel.interval;
                }
                break;
            }
            default:
                throw UnsupportedOperationError("telemetry is not pollable on L20Lite");
            }
        }
        device.start_polling(selected);
    }

    void start_device_polling(l20::L20 &device, const PollingPlan &plan) {
        std::map<l20::SensorSource, std::chrono::milliseconds> selected;
        for (const auto &channel : plan) {
            switch (channel.semantic) {
            case TelemetrySemantic::kMeasuredSpeeds:
                selected[l20::SensorSource::kSpeed] = channel.interval;
                break;
            case TelemetrySemantic::kAngles:
                selected[l20::SensorSource::kAngle] = channel.interval;
                break;
            case TelemetrySemantic::kCurrents:
                selected[l20::SensorSource::kCurrent] = channel.interval;
                break;
            case TelemetrySemantic::kTemperatures:
                selected[l20::SensorSource::kTemperature] = channel.interval;
                break;
            case TelemetrySemantic::kFaults:
                selected[l20::SensorSource::kFault] = channel.interval;
                break;
            case TelemetrySemantic::kForce:
                selected[l20::SensorSource::kForceSensor] = channel.interval;
                break;
            default:
                throw UnsupportedOperationError("telemetry is not pollable on L20");
            }
        }
        device.start_polling(selected);
    }

    void start_device_polling(l30::L30 &device, const PollingPlan &plan) {
        std::map<l30::SensorSource, std::chrono::milliseconds> selected;
        for (const auto &channel : plan) {
            switch (channel.semantic) {
            case TelemetrySemantic::kMeasuredSpeeds:
                selected[l30::SensorSource::kSpeed] = channel.interval;
                break;
            case TelemetrySemantic::kAngles:
                selected[l30::SensorSource::kAngle] = channel.interval;
                break;
            case TelemetrySemantic::kCurrents:
                selected[l30::SensorSource::kCurrent] = channel.interval;
                break;
            case TelemetrySemantic::kTemperatures:
                selected[l30::SensorSource::kTemperature] = channel.interval;
                break;
            case TelemetrySemantic::kFaults:
                selected[l30::SensorSource::kFault] = channel.interval;
                break;
            case TelemetrySemantic::kForce:
                selected[l30::SensorSource::kForceSensor] = channel.interval;
                break;
            default:
                throw UnsupportedOperationError("telemetry is not pollable on L30");
            }
        }
        device.start_polling(selected);
    }

    void start_device_polling(o30::O30 &device, const PollingPlan &plan) {
        std::map<o30::SensorSource, std::chrono::milliseconds> selected;
        for (const auto &channel : plan) {
            switch (channel.semantic) {
            case TelemetrySemantic::kMeasuredSpeeds:
                selected[o30::SensorSource::kSpeed] = channel.interval;
                break;
            case TelemetrySemantic::kVoltages:
                selected[o30::SensorSource::kVoltage] = channel.interval;
                break;
            case TelemetrySemantic::kAngles:
                selected[o30::SensorSource::kAngle] = channel.interval;
                break;
            case TelemetrySemantic::kCurrents:
                selected[o30::SensorSource::kCurrent] = channel.interval;
                break;
            case TelemetrySemantic::kCurrentLimits:
                selected[o30::SensorSource::kCurrentLimit] = channel.interval;
                break;
            case TelemetrySemantic::kTorques:
                selected[o30::SensorSource::kTorque] = channel.interval;
                break;
            case TelemetrySemantic::kTemperatures:
                selected[o30::SensorSource::kTemperature] = channel.interval;
                break;
            case TelemetrySemantic::kFaults:
                selected[o30::SensorSource::kFault] = channel.interval;
                break;
            case TelemetrySemantic::kForce:
                selected[o30::SensorSource::kAllForceSensors] = channel.interval;
                break;
            default:
                throw UnsupportedOperationError("telemetry is not pollable on O30");
            }
        }
        device.start_polling(selected);
    }
} // namespace linkerbot::hand::unified_detail
