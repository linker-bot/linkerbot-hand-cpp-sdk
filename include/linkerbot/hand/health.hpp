#pragma once

#include <linkerbot/can/health.hpp>
#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/transport/session.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace linkerbot::hand {
    // Increment when an existing health field, unit, or canonical name changes
    // incompatibly. Append-only metrics keep this version.
    inline constexpr uint16_t kHealthSchemaVersion = 1;

    // Health snapshots use a monotonic clock. Durations are never derived from
    // wall-clock time and remain valid across system clock adjustments.
    using HealthClock = std::chrono::steady_clock;
    using HealthTimePoint = HealthClock::time_point;
    using HealthDuration = HealthClock::duration;

    // Successful transaction latency retains only the latest samples. This
    // bound is part of the public metric contract, so long-running processes do
    // not accumulate unbounded diagnostic state.
    inline constexpr size_t kHealthLatencyWindowCapacity = 2048;

    enum class LifecycleState : uint8_t {
        kClosed,
        kOpening,
        kOpen,
        kClosing,
        kRecovering,
        kFaulted,
    };

    enum class ConnectionState : uint8_t {
        kClosed,
        kConnecting,
        kConnected,
        kReconnecting,
        kFaulted,
    };

    // Serializable error data retained by a health snapshot. The exception
    // cause remains available on the HandError at the throwing boundary; health
    // deliberately owns only stable diagnostic values.
    struct ErrorInfo {
        ErrorCode code = ErrorCode::kInternal;
        std::string message;
        std::string error_type;
        ErrorContext context;
    };

    struct DataAge {
        bool valid = false;
        bool stale = true;
        std::optional<HealthDuration> age;
        uint64_t sequence = 0;
        uint64_t generation = 0;
    };

    // A bounded latency distribution. All values use steady_clock::duration;
    // count is the number of samples currently retained by the rolling window,
    // never more than kHealthLatencyWindowCapacity.
    struct LatencyMetrics {
        uint64_t count = 0;
        std::optional<HealthDuration> p50;
        std::optional<HealthDuration> p95;
        std::optional<HealthDuration> p99;
    };

    // Depth values count currently retained jobs/items. accepted and dropped
    // are cumulative item counters, not CAN frame counters.
    struct QueueMetrics {
        size_t depth = 0;
        size_t high_water = 0;
        uint64_t accepted = 0;
        uint64_t dropped = 0;
    };

    // One canonical telemetry channel. Rates are samples per second; counters
    // count logical channel reads/samples rather than physical CAN frames.
    // requested_hz is configuration intent. achieved_hz is calculated from
    // complete feedback samples over the channel's active measurement window.
    struct ChannelMetrics {
        std::string channel;
        bool supported = true;
        bool polling_active = false;
        double requested_hz = 0.0;
        double achieved_hz = 0.0;
        uint64_t missed_deadlines = 0;
        uint64_t timeouts = 0;
        uint64_t completed_reads = 0;
        uint64_t feedback_samples = 0;
        DataAge data_age;
        LatencyMetrics latency;
        QueueMetrics queue;
        uint64_t command_feedback_samples = 0;
        std::optional<AngleFeedbackPolicy> feedback_policy;
    };

    // One independently scheduled command family. Counters count logical
    // commands; queue metrics describe the family's admission queue.
    struct CommandFamilyMetrics {
        std::string family;
        size_t pending = 0;
        uint64_t accepted = 0;
        uint64_t dispatched = 0;
        uint64_t acknowledged = 0;
        uint64_t failed = 0;
        uint64_t timed_out = 0;
        uint64_t cancelled = 0;
        uint64_t superseded = 0;
        QueueMetrics queue;
        // Bounded rolling distributions for unified setter queue residence
        // and execution (including arbitration), in steady-clock durations.
        LatencyMetrics queue_latency;
        LatencyMetrics execution_latency;
    };

    struct ForceFieldMetrics {
        std::string field;
        double requested_hz = 0.0;
        double query_hz = 0.0;
        double achieved_hz = 0.0;
        std::optional<TelemetrySemantic> query_channel;
        std::optional<bool> supported;
    };

    // Complete, zero-I/O snapshot for one hand connection generation. The
    // runtime returns this value by copy from mutex-protected cached state;
    // reading health never sends a frame or waits for a device response.
    //
    // Counter units:
    // - reconnect_count and consecutive_errors count lifecycle/error events;
    // - ChannelMetrics counters count logical reads or samples;
    // - CommandFamilyMetrics and QueueMetrics count jobs/items;
    // - nested SessionHealth/TransportHealth counters retain their documented
    //   physical frame and byte units.
    struct HandHealth {
        LifecycleState lifecycle_state = LifecycleState::kClosed;
        ConnectionState connection_state = ConnectionState::kClosed;
        bool identity_verified = false;
        bool polling_active = false;
        uint64_t connection_generation = 0;
        uint64_t reconnect_count = 0;
        uint64_t consecutive_errors = 0;
        std::optional<ErrorInfo> recent_error;
        std::optional<HealthTimePoint> last_successful_send;
        std::optional<HealthTimePoint> last_successful_receive;
        std::optional<HealthTimePoint> last_successful_transaction;
        std::vector<ChannelMetrics> channels;
        std::vector<CommandFamilyMetrics> command_families;
        LatencyMetrics transaction_latency;
        can::TransportHealth transport;
        transport::SessionHealth session;
        // Python policy: field intent and shared-query rates. Never sum these
        // rows to estimate physical traffic; channel counters count queries once.
        std::vector<ForceFieldMetrics> force_fields;

        [[nodiscard]] bool connected() const noexcept { return connection_state == ConnectionState::kConnected; }

        [[nodiscard]] size_t pending_commands() const noexcept {
            size_t total = 0;
            for (const auto &metrics : command_families) {
                total += metrics.pending;
            }
            return total;
        }

        [[nodiscard]] uint64_t superseded_commands() const noexcept {
            uint64_t total = 0;
            for (const auto &metrics : command_families) {
                total += metrics.superseded;
            }
            return total;
        }
    };
}
