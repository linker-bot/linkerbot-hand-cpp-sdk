#pragma once

#include "command_delivery.hpp"
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <variant>
#include <vector>

#include <linkerbot/hand/api/device.hpp>
#include <linkerbot/hand/runtime/device_runtime.hpp>

namespace linkerbot::hand::unified_detail {
    struct CommandRequest {
        CommandSemantic semantic = CommandSemantic::kAngles;
        std::vector<double> raw_values;
        std::chrono::milliseconds timeout{100};
        AckExpectation ack_expectation = AckExpectation::kNone;
        std::function<bool()> generation_valid;
        std::optional<std::chrono::steady_clock::time_point> deadline;
        std::optional<CommandHandle> completion;
        std::function<void(uint64_t, std::chrono::steady_clock::time_point)> dispatched;
        std::function<void()> acknowledged;
        runtime::CancellationToken cancellation;
        std::optional<DeviceOperation> operation;
        std::shared_ptr<CommandDelivery> delivery;
        std::function<std::shared_ptr<void>(runtime::CancellationToken)> prepare_execution;
        std::shared_ptr<void> execution_lease;
    };

    using TelemetrySample =
        std::variant<AngleSample, MeasuredSpeedSample, ConfiguredSpeedSample, ConfiguredAccelerationSample,
                     CurrentSample, CurrentLimitSample, TorqueSample, TemperatureSample, ForceSample,
                     NormalForceResultantSample, FaultSample, VoltageSample, StallDetectionTimeSample,
                     StallThresholdSample, StallHoldCurrentSample, PositionKpSample, PositionKdSample, TorqueKdSample,
                     TorqueLimitSample, MotionTimeSample, LimitCompensationSample>;

    class DeviceBackend {
    public:
        virtual ~DeviceBackend() = default;
        virtual void stop_admission() {}
        virtual void resume_admission() {}

        [[nodiscard]] virtual CommandHandle submit(const CommandRequest &request) = 0;
        [[nodiscard]] virtual TelemetrySample read(TelemetrySemantic semantic, std::chrono::milliseconds timeout) = 0;
        [[nodiscard]] virtual CommandHandle operate(DeviceOperation operation, std::chrono::milliseconds timeout) = 0;
        [[nodiscard]] virtual HandSnapshot snapshot(Model model, HandSide side,
                                                    const DeviceCapabilities &capabilities) const = 0;
        [[nodiscard]] virtual HandHealth health() const = 0;
        [[nodiscard]] virtual DeviceIdentity verify_identity(HandSide side, std::chrono::milliseconds timeout) = 0;
        virtual TelemetrySample initialize_read(TelemetrySemantic semantic, std::chrono::milliseconds timeout) {
            return read(semantic, timeout);
        }
        virtual DeviceIdentity initialize_identity(HandSide side, std::chrono::milliseconds timeout) {
            return verify_identity(side, timeout);
        }
        virtual void start_polling(const PollingPlan &plan) = 0;
        virtual void stop_polling() = 0;
        virtual void recover() = 0;
        virtual void set_execution_deadline(runtime::Deadline deadline) = 0;
        virtual void drain_sends(runtime::Deadline deadline) = 0;
        virtual std::shared_ptr<void> prepare_command(runtime::CancellationToken cancellation,
                                                      const std::function<bool()> &valid) = 0;
        virtual void wait_quiet(std::chrono::milliseconds quiet, runtime::Deadline deadline,
                                const std::function<bool()> &cancelled) = 0;
        virtual void close() = 0;
        virtual void cancel_commands() = 0;
        virtual void assert_external_lifecycle() const = 0;
    };

    [[nodiscard]] std::unique_ptr<DeviceBackend>
    make_backend(const HandDeviceConfig &config, const HandDevice::TransportFactory &transport_factory,
                 const std::shared_ptr<transport::Session> &shared_session);
} // namespace linkerbot::hand::unified_detail
