#include "detail/freshness.hpp"
#include "detail/o30_v25.hpp"
#include <linkerbot/hand/conversion.hpp>
#include <linkerbot/hand/o30/v25.hpp>
namespace linkerbot::hand::unified_detail {
    namespace {
        using Semantic = TelemetrySemantic;
        SampleMeta metadata(const runtime::Sample &sample, const DeviceCapabilities &capabilities, Semantic semantic) {
            SampleMeta result;
            result.valid = sample.valid;
            result.sequence = sample.sequence;
            result.connection_generation = sample.device_generation;
            result.collected_at = sample.received_at;
            result.delivery = sample.origin == protocol::Origin::command_feedback ? TelemetryDelivery::kCommandResponse
                                                                                  : TelemetryDelivery::kQueryResponse;
            const auto *channel = capabilities.telemetry_channel(semantic);
            result.origin = channel && !channel->device_measured ? TelemetryValueOrigin::kDeviceConfigured
                                                                 : TelemetryValueOrigin::kDeviceMeasured;
            complete_freshness(result, channel, runtime::Clock::now());
            return result;
        }
        template <typename Sample>
        void project(const std::shared_ptr<runtime::DeviceRuntime> &core, const DeviceCapabilities &capabilities,
                     Semantic semantic, std::optional<Sample> &target) {
            const auto sample = core->snapshot(o30::v25::domain(o30::v25::object(semantic)));
            if (!sample || !sample->valid) {
                return;
            }
            Sample result;
            const auto values = o30::v25::decode(std::any_cast<const std::vector<uint8_t> &>(sample->value));
            result.meta = metadata(*sample, capabilities, semantic);
            if constexpr (std::is_same_v<Sample, FaultSample>) {
                for (const auto value : values) {
                    const auto raw = static_cast<uint8_t>(value);
                    result.raw.push_back(raw);
                    result.statuses.push_back(decode_fault_status(raw, 0xCF));
                }
            } else {
                result.raw = values;
                if constexpr (std::is_same_v<Sample, AngleSample>) {
                    result.percent = raw_angles_to_percent(values, capabilities);
                    result.physical_rad = raw_angles_to_physical_rad(values, capabilities);
                    result.meta.within_declared_range = raw_angles_within_declared_range(values, capabilities);
                }
            }
            target = std::move(result);
        }
    }
    CommandOutcome submit_v25(o30::O30 &device, const CommandRequest &request) {
        auto plan = o30::v25::transaction(device.request_id(), o30::v25::object(request.semantic),
                                          o30::v25::encode(request.raw_values));
        plan.publish_after_send = true;
        plan.preserve_last_success = true;
        if (request.semantic != CommandSemantic::kAngles) {
            // Actual-state ACKs do not confirm requested configuration values.
            auto consume = std::move(plan.consume);
            plan.consume = [consume = std::move(consume)](const can::Message &message) mutable {
                auto progress = consume(message);
                if (progress.kind == protocol::ProgressKind::complete) {
                    progress.value = true;
                }
                return progress;
            };
            plan.domain = "v25/write/" + std::to_string(static_cast<unsigned>(request.semantic));
        } else {
            auto consume = std::move(plan.consume);
            plan.consume = [consume = std::move(consume)](const can::Message &message) mutable {
                auto progress = consume(message);
                progress.origin = protocol::Origin::command_feedback;
                return progress;
            };
        }
        (void)device.runtime()->execute(std::move(plan), runtime::deadline_after(request.timeout));
        return {1, true};
    }
    void project_v25(const o30::O30 &device, HandSnapshot &snapshot, const DeviceCapabilities &capabilities) {
        auto core = device.runtime();
#define PROJECT(Member, Name) project(core, capabilities, Semantic::Name, snapshot.Member)
        PROJECT(angles, kAngles);
        PROJECT(measured_speeds, kMeasuredSpeeds);
        PROJECT(configured_speeds, kConfiguredSpeeds);
        PROJECT(configured_accelerations, kConfiguredAccelerations);
        PROJECT(currents, kCurrents);
        PROJECT(current_limits, kCurrentLimits);
        PROJECT(torques, kTorques);
        PROJECT(torque_limits, kTorqueLimits);
        PROJECT(motion_times, kMotionTimes);
        PROJECT(temperatures, kTemperatures);
        PROJECT(voltages, kVoltages);
        PROJECT(faults, kFaults);
        PROJECT(stall_detection_times, kStallDetectionTimes);
        PROJECT(stall_thresholds, kStallThresholds);
        PROJECT(stall_hold_currents, kStallHoldCurrents);
#undef PROJECT
        for (size_t i = 0; i < snapshot.joints.size(); ++i) {
            auto &joint = snapshot.joints[i];
#define JOINT(Member, Field)                                                                                           \
    if (snapshot.Member) {                                                                                             \
        joint.Field = snapshot.Member->raw.at(i);                                                                      \
    }
            JOINT(angles, angle_raw);
            if (snapshot.angles) {
                joint.angle_percent = snapshot.angles->percent.at(i);
                joint.physical_rad = snapshot.angles->physical_rad.at(i);
            }
            JOINT(measured_speeds, measured_speed_raw);
            JOINT(configured_speeds, configured_speed_raw);
            JOINT(configured_accelerations, configured_acceleration_raw);
            JOINT(currents, current_raw);
            JOINT(current_limits, current_limit_raw);
            JOINT(torques, torque_raw);
            JOINT(torque_limits, torque_limits_raw);
            JOINT(motion_times, motion_times_raw);
            JOINT(temperatures, temperature_raw);
            JOINT(voltages, voltage_raw);
            JOINT(faults, fault_raw);
            JOINT(stall_detection_times, stall_detection_time_raw);
            JOINT(stall_thresholds, stall_threshold_raw);
            JOINT(stall_hold_currents, stall_hold_current_raw);
#undef JOINT
        }
    }
    TelemetrySample read_v25(o30::O30 &device, Semantic semantic, std::chrono::milliseconds timeout,
                             const DeviceCapabilities &capabilities) {
        auto plan = o30::v25::transaction(device.request_id(), o30::v25::object(semantic));
        (void)device.runtime()->execute(std::move(plan), runtime::deadline_after(timeout));
        auto snapshot = unified_snapshot(device.get_snapshot(), Model::kO30, HandSide::kUnknown, capabilities);
        project_v25(device, snapshot, capabilities);
        return telemetry_sample(snapshot, semantic);
    }
}
