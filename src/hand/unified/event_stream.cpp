#include "detail/event_stream.hpp"

namespace linkerbot::hand::unified_detail {
    namespace {
        template <typename Sample>
        void publish_if_new(const std::optional<Sample> &sample, TelemetrySemantic semantic,
                            const std::shared_ptr<EventQueue<HandEvent>> &queue,
                            std::map<TelemetrySemantic, std::chrono::steady_clock::time_point> &last_seen) {
            if (!sample) {
                return;
            }
            if (!sample->meta.collected_at) {
                return;
            }
            const auto timestamp = *sample->meta.collected_at;
            const auto previous = last_seen.find(semantic);
            if (previous != last_seen.end() && previous->second >= timestamp) {
                return;
            }
            last_seen[semantic] = timestamp;
            queue->push(HandEvent{semantic, *sample});
        }
    } // namespace

    void publish_snapshot_events(const HandSnapshot &snapshot, const std::shared_ptr<EventQueue<HandEvent>> &queue,
                                 std::map<TelemetrySemantic, std::chrono::steady_clock::time_point> &last_seen) {
        publish_if_new(snapshot.angles, TelemetrySemantic::kAngles, queue, last_seen);
        publish_if_new(snapshot.measured_speeds, TelemetrySemantic::kMeasuredSpeeds, queue, last_seen);
        publish_if_new(snapshot.configured_speeds, TelemetrySemantic::kConfiguredSpeeds, queue, last_seen);
        publish_if_new(snapshot.configured_accelerations, TelemetrySemantic::kConfiguredAccelerations, queue,
                       last_seen);
        publish_if_new(snapshot.currents, TelemetrySemantic::kCurrents, queue, last_seen);
        publish_if_new(snapshot.current_limits, TelemetrySemantic::kCurrentLimits, queue, last_seen);
        publish_if_new(snapshot.torques, TelemetrySemantic::kTorques, queue, last_seen);
        publish_if_new(snapshot.temperatures, TelemetrySemantic::kTemperatures, queue, last_seen);
        publish_if_new(snapshot.faults, TelemetrySemantic::kFaults, queue, last_seen);
        publish_if_new(snapshot.force, TelemetrySemantic::kForce, queue, last_seen);
        publish_if_new(snapshot.normal_force_resultant, TelemetrySemantic::kForceFns, queue, last_seen);
        publish_if_new(snapshot.voltages, TelemetrySemantic::kVoltages, queue, last_seen);
        publish_if_new(snapshot.stall_detection_times, TelemetrySemantic::kStallDetectionTimes, queue, last_seen);
        publish_if_new(snapshot.stall_thresholds, TelemetrySemantic::kStallThresholds, queue, last_seen);
        publish_if_new(snapshot.stall_hold_currents, TelemetrySemantic::kStallHoldCurrents, queue, last_seen);
    }
} // namespace linkerbot::hand::unified_detail
