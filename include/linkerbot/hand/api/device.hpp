#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <linkerbot/event_queue.hpp>
#include <linkerbot/hand/api/commands.hpp>
#include <linkerbot/hand/api/config.hpp>
#include <linkerbot/hand/api/events.hpp>
#include <linkerbot/hand/api/snapshot.hpp>
#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/health.hpp>
#include <linkerbot/hand/identity.hpp>

namespace linkerbot::can {
    class CANTransport;
}

namespace linkerbot::transport {
    class Session;
}

namespace linkerbot::hand {
    class DiscoveryCandidate;

    class HandDevice {
    public:
        using TransportFactory = std::function<std::unique_ptr<can::CANTransport>()>;

        explicit HandDevice(HandDeviceConfig config);
        HandDevice(HandDeviceConfig config, TransportFactory transport_factory);
        HandDevice(HandDeviceConfig config, std::shared_ptr<transport::Session> shared_session);
        ~HandDevice();

        HandDevice(const HandDevice &) = delete;
        HandDevice &operator=(const HandDevice &) = delete;
        HandDevice(HandDevice &&) = delete;
        HandDevice &operator=(HandDevice &&) = delete;

        // Opens the backend, verifies model/side identity and reads the
        // model-specific safety baseline. Commands are admitted only after
        // every initialization step succeeds.
        void open();
        // Stops admission and fails every unfinished setter with ClosedError.
        // Waits at most shutdown_timeout for cleanup, then throws TimeoutError
        // and remains non-Ready while retained resources finish cleanup.
        // Runtime callbacks must schedule close on an external thread instead.
        void close();
        // Local facade lifecycle only. Use get_health().connected() for the
        // current device connection state.
        [[nodiscard]] bool is_open() const noexcept;

        // Returns static declared capabilities before open(), then the
        // identity-verified profile for the active ready generation. This is
        // a cached query and never performs device I/O.
        [[nodiscard]] DeviceCapabilities get_capabilities() const;

        // Cached zero-I/O diagnostics. A closed facade returns the explicit
        // HandHealth closed state even when it was previously open.
        [[nodiscard]] HandHealth get_health() const;

        // open() populates the identity cache. This method explicitly reads
        // and validates device-reported identity again. A failed active
        // refresh invalidates readiness and requires close/reopen. Cached
        // queries below never send frames. The cache belongs to the current
        // open generation and is cleared by close().
        [[nodiscard]] DeviceIdentity verify_identity(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                                         1000});
        // Retained alias with verify_identity() lifecycle semantics.
        // Python-compatible read behavior uses refresh_device_info() below.
        [[nodiscard]] DeviceIdentity get_device_info(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                                         1000});
        // Active identity read in the current generation. Success does not
        // cancel commands or restart polling. A physical identity mismatch
        // faults this facade; timeout/protocol errors leave admission unchanged.
        // The timeout includes waiting for the endpoint execution gate.
        [[nodiscard]] DeviceIdentity refresh_device_info(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                                             1000});
        [[nodiscard]] std::optional<DeviceIdentity> get_identity() const;
        [[nodiscard]] std::optional<DeviceCapabilities> get_verified_capabilities() const;

        // Device operations throw DeviceNotReadyError until open() completes
        // identity and baseline initialization. Known unsupported operations
        // throw UnsupportedOperationError before any frame is sent.
        // All set_* copy and validate inputs, then return after bounded queue
        // admission. The device executor sends/waits in the background; runtime
        // failures are observed through handle.stage()/error(). Explicitly wait
        // and check the terminal result when protocol completion is required.
        // Angles (raw and percent), speeds and current limits
        // replace only pending targets of the same family. Other setters use
        // FIFO; a full pending FIFO throws QueueFullError before acceptance.
        // wait_for() timeout does not cancel or retry an accepted command.
        // Protocol-native integer positions. HandDevice validates each value
        // against the configured model's per-joint raw range.
        [[nodiscard]] CommandHandle set_angles(const std::vector<int32_t> &values);
        // Normalized positions in the inclusive 0..100 range. Conversion uses
        // the configured model's per-joint percent mapping before submission.
        [[nodiscard]] CommandHandle set_percent_angles(const std::vector<float> &values);
        [[nodiscard]] CommandHandle set_speeds(const std::vector<double> &values);
        [[nodiscard]] CommandHandle set_accelerations(const std::vector<double> &values);
        [[nodiscard]] CommandHandle set_current_limits(const std::vector<double> &values);
        [[nodiscard]] CommandHandle set_stall_detection_times(const std::vector<double> &values);
        [[nodiscard]] CommandHandle set_stall_thresholds(const std::vector<double> &values);
        [[nodiscard]] CommandHandle set_stall_hold_currents(const std::vector<double> &values);

        [[nodiscard]] CommandHandle set_position_kp(const std::vector<double> &values);
        [[nodiscard]] PositionKpSample get_position_kp(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                                           100});

        [[nodiscard]] CommandHandle set_position_kd(const std::vector<double> &values);
        [[nodiscard]] PositionKdSample get_position_kd(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                                           100});

        [[nodiscard]] CommandHandle set_torque_kd(const std::vector<double> &values);
        [[nodiscard]] TorqueKdSample get_torque_kd(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

        [[nodiscard]] CommandHandle set_torque_limits(const std::vector<double> &values);
        [[nodiscard]] TorqueLimitSample get_torque_limits(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                                              100});

        [[nodiscard]] CommandHandle set_motion_times(const std::vector<double> &values);
        [[nodiscard]] MotionTimeSample get_motion_times(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                                            100});

        [[nodiscard]] CommandHandle set_limit_compensations(const std::vector<double> &values);
        [[nodiscard]] LimitCompensationSample
        get_limit_compensations(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

        // Nonblocking operation admission through the same FIFO as setters.
        // Read-device-info is not an operation handle; use get_device_info().
        // Legacy operation methods below retain their existing synchronous behavior.
        [[nodiscard]] CommandHandle
        submit_operation(DeviceOperation operation, std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] CommandHandle clear_faults();
        [[nodiscard]] CommandHandle enable();
        [[nodiscard]] CommandHandle disable();
        [[nodiscard]] CommandHandle save_parameters();
        // Explicit ACK-gated persistence; legacy save_parameters() remains send-only.
        [[nodiscard]] CommandHandle
        save_parameters_confirmed(std::chrono::milliseconds timeout = std::chrono::milliseconds{200});
        // L30: disable, unlock and calibrate, each gated by its device ACK.
        [[nodiscard]] CommandHandle calibrate(std::chrono::milliseconds timeout = std::chrono::milliseconds{300});
        [[nodiscard]] CommandHandle set_torques(const std::vector<double> &values,
                                                std::chrono::milliseconds timeout = std::chrono::milliseconds{300});
        // Python spelling retained as an alias for torque limits.
        [[nodiscard]] CommandHandle set_torques_limits(const std::vector<double> &values);
        [[nodiscard]] TorqueLimitSample
        get_torques_limits(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

        [[nodiscard]] AngleSample get_angles(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] MeasuredSpeedSample get_speeds(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                                         100});
        [[nodiscard]] ConfiguredSpeedSample
        get_configured_speeds(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] ConfiguredAccelerationSample
        get_configured_accelerations(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] CurrentSample get_currents(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] CurrentLimitSample
        get_current_limits(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] TorqueSample get_torques(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] TemperatureSample get_temperatures(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                                             100});
        [[nodiscard]] ForceSample get_force(std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});
        [[nodiscard]] ForceSample get_force(const std::vector<std::string> &fields,
                                            std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});
        [[nodiscard]] NormalForceResultantSample
        get_normal_force_resultant(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] FaultSample get_faults(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] VoltageSample get_voltages(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] StallDetectionTimeSample
        get_stall_detection_times(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] StallThresholdSample
        get_stall_thresholds(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] StallHoldCurrentSample
        get_stall_hold_currents(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
        [[nodiscard]] HandSnapshot get_snapshot() const;
        // Zero-I/O cache available in every lifecycle state. Initial supported
        // vector samples are shaped but invalid. On generation loss raw evidence
        // is retained with invalid metadata; joint projections and identity are
        // cleared. Its generation counts successful opens/recoveries locally,
        // independently of the retained get_snapshot() native generation policy.
        [[nodiscard]] HandSnapshot get_cached_snapshot() const;
        // Waits for a cached snapshot revision. This method never performs
        // device I/O; timeout returns the latest snapshot, even if unchanged.
        [[nodiscard]] HandSnapshot wait_for_snapshot(const HandSnapshot &previous,
                                                     std::chrono::milliseconds timeout) const;

        void start_polling();
        void start_polling(const PollingPlan &plan);
        void stop_polling();

        // Ends the active generation, recovers the backend, verifies that the
        // same physical device is still attached, rereads the safety baseline
        // and restores only the previous polling plan. Targets and discrete
        // commands are never replayed. Failure leaves the facade Faulted until
        // close()/open(). Recovery also consumes the total shutdown_timeout
        // caller budget; expiry cannot later restore Ready.
        void recover();

        // A bounded stream of common cached telemetry. Model-specific streams
        // remain available on the concrete model classes.
        [[nodiscard]] std::shared_ptr<EventQueue<HandEvent>> stream_handle(size_t max_size = 100);
        void stop_stream();

    private:
        friend class DiscoveryCandidate;

        HandDevice(HandDeviceConfig config, TransportFactory transport_factory, DeviceIdentity expected_identity);

        class Impl;
        std::shared_ptr<Impl> impl_;
    };
} // namespace linkerbot::hand
