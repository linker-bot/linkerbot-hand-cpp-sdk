#pragma once

#include <linkerbot/hand/api/snapshot.hpp>
#include <type_traits>

namespace linkerbot::hand::unified_detail {
    template <class Visitor> void visit_snapshot_samples(HandSnapshot &snapshot, Visitor visit) {
        visit(snapshot.angles, TelemetrySemantic::kAngles);
        visit(snapshot.measured_speeds, TelemetrySemantic::kMeasuredSpeeds);
        visit(snapshot.configured_speeds, TelemetrySemantic::kConfiguredSpeeds);
        visit(snapshot.configured_accelerations, TelemetrySemantic::kConfiguredAccelerations);
        visit(snapshot.currents, TelemetrySemantic::kCurrents);
        visit(snapshot.current_limits, TelemetrySemantic::kCurrentLimits);
        visit(snapshot.torques, TelemetrySemantic::kTorques);
        visit(snapshot.temperatures, TelemetrySemantic::kTemperatures);
        visit(snapshot.force, TelemetrySemantic::kForce);
        visit(snapshot.normal_force_resultant, TelemetrySemantic::kForceFns);
        visit(snapshot.faults, TelemetrySemantic::kFaults);
        visit(snapshot.voltages, TelemetrySemantic::kVoltages);
        visit(snapshot.stall_detection_times, TelemetrySemantic::kStallDetectionTimes);
        visit(snapshot.stall_thresholds, TelemetrySemantic::kStallThresholds);
        visit(snapshot.stall_hold_currents, TelemetrySemantic::kStallHoldCurrents);
        visit(snapshot.position_kp, TelemetrySemantic::kPositionKp);
        visit(snapshot.position_kd, TelemetrySemantic::kPositionKd);
        visit(snapshot.torque_kd, TelemetrySemantic::kTorqueKd);
        visit(snapshot.torque_limits, TelemetrySemantic::kTorqueLimits);
        visit(snapshot.motion_times, TelemetrySemantic::kMotionTimes);
        visit(snapshot.limit_compensations, TelemetrySemantic::kLimitCompensations);
    }

    inline void shape_cached_samples(HandSnapshot &result, const DeviceCapabilities &capabilities) {
        visit_snapshot_samples(result, [&](auto &sample, TelemetrySemantic semantic) {
            const auto *channel = capabilities.telemetry_channel(semantic);
            if (!channel || sample) {
                return;
            }
            sample.emplace();
            using Sample = typename std::decay_t<decltype(sample)>::value_type;
            if constexpr (std::is_same_v<Sample, ForceSample>) {
                sample->shape = channel->shape;
            } else {
                sample->raw.resize(channel->channels.size());
                if constexpr (std::is_same_v<Sample, AngleSample>) {
                    sample->percent.resize(channel->channels.size());
                    sample->physical_rad.resize(channel->channels.size());
                }
            }
        });
    }

    inline HandSnapshot initial_cached_snapshot(const DeviceCapabilities &capabilities) {
        HandSnapshot result;
        result.model = capabilities.model;
        result.side = capabilities.side;
        for (const auto &joint : capabilities.joints) {
            JointState state;
            state.index = joint.index;
            state.name = joint.name;
            result.joints.push_back(state);
        }
        shape_cached_samples(result, capabilities);
        return result;
    }

    inline void set_cached_generation(HandSnapshot &snapshot, uint64_t generation) {
        snapshot.connection_generation = generation;
        visit_snapshot_samples(snapshot, [generation](auto &sample, TelemetrySemantic) {
            if (!sample) {
                return;
            }
            sample->meta.connection_generation = generation;
            using Sample = typename std::decay_t<decltype(sample)>::value_type;
            if constexpr (std::is_same_v<Sample, ForceSample>) {
                for (auto &field : sample->fields) {
                    field.meta.connection_generation = generation;
                }
            }
        });
    }

    inline void invalidate_cached_snapshot(HandSnapshot &snapshot) {
        visit_snapshot_samples(snapshot,
                               [generation = snapshot.connection_generation](auto &sample, TelemetrySemantic) {
                                   if (!sample) {
                                       return;
                                   }
                                   sample->meta = SampleMeta::unavailable(generation);
                                   using Sample = typename std::decay_t<decltype(sample)>::value_type;
                                   if constexpr (std::is_same_v<Sample, ForceSample>) {
                                       for (auto &field : sample->fields) {
                                           field.meta = SampleMeta::unavailable(generation);
                                       }
                                       for (auto &sensor : sample->sensors) {
                                           sensor.available = false;
                                       }
                                       for (auto &matrix : sample->tactile) {
                                           matrix.online.reset();
                                       }
                                   }
                               });
        for (auto &joint : snapshot.joints) {
            const auto index = joint.index;
            const auto name = joint.name;
            joint = JointState{};
            joint.index = index;
            joint.name = name;
        }
        // DeviceIdentity has no validity metadata, so stale identity is omitted.
        snapshot.device_info.reset();
    }
}
