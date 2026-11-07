#include "detail/backend.hpp"
#include "detail/o30_v25.hpp"

#include "detail/command_handle_access.hpp"
#include "detail/dispatch_guard.hpp"
#include "detail/execution_gate.hpp"
#include "detail/freshness.hpp"
#include "detail/model_adapters.hpp"
#include "detail/runtime_access.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>

#include <linkerbot/hand/conversion.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/transport/session.hpp>
#ifdef __linux__
#include <linkerbot/can/socketcan_transport.hpp>
#endif

namespace linkerbot::hand::unified_detail {
    namespace {
        [[nodiscard]] CommandFamily command_family(CommandSemantic semantic) {
            switch (semantic) {
            case CommandSemantic::kAngles:
                return CommandFamily::kAngleTarget;
            case CommandSemantic::kSpeeds:
                return CommandFamily::kSpeedSetting;
            case CommandSemantic::kAccelerations:
                return CommandFamily::kAccelerationSetting;
            case CommandSemantic::kCurrentLimits:
                return CommandFamily::kCurrentLimit;
            case CommandSemantic::kPositionKp:
            case CommandSemantic::kPositionKd:
            case CommandSemantic::kTorqueKd:
                return CommandFamily::kControlGain;
            case CommandSemantic::kTorqueLimits:
                return CommandFamily::kTorqueLimit;
            case CommandSemantic::kTorques:
                return CommandFamily::kTorqueTarget;
            case CommandSemantic::kMotionTimes:
                return CommandFamily::kMotionTime;
            case CommandSemantic::kLimitCompensations:
                return CommandFamily::kLimitCompensation;
            case CommandSemantic::kStallDetectionTimes:
            case CommandSemantic::kStallThresholds:
            case CommandSemantic::kStallHoldCurrents:
                return CommandFamily::kStallConfiguration;
            }
            throw UnsupportedOperationError("unknown command semantic");
        }

        [[nodiscard]] uint64_t next_command_id() {
            static std::atomic<uint64_t> next{1};
            return next.fetch_add(1, std::memory_order_relaxed);
        }

        template <typename Device> void recover_device(Device &device) { device.recover(); }

        void recover_device(l30::L30 &device) { device.runtime()->recover(); }

        template <typename Device, size_t JointCount> class ModelBackend;

        template <typename Device, size_t JointCount> class ModelBackend final : public DeviceBackend {
        public:
            explicit ModelBackend(std::unique_ptr<Device> device, std::shared_ptr<transport::Session> owned_session,
                                  const HandDeviceConfig &config)
                : config_(config), capabilities_(declared_capabilities(config)), device_(std::move(device)),
                  owned_session_(std::move(owned_session)), execution_gate_(std::make_shared<ExecutionGate>()),
                  dispatch_guard_(std::make_shared<DispatchGuard>()) {
                RuntimeAccess::set_polling_gate(*device_->runtime(), [gate = execution_gate_, guard = dispatch_guard_](
                                                                         runtime::CancellationToken token) {
                    auto lease = gate->acquire(false, runtime::Deadline::max(), token);
                    guard->wait(runtime::Deadline::max(), token);
                    return lease;
                });
            }

            [[nodiscard]] CommandHandle submit(const CommandRequest &request) override {
                const auto *capability = capabilities_.command(request.semantic);
                if (request.operation ? !capabilities_.supports(*request.operation) : !capability) {
                    throw UnsupportedOperationError("command or operation is not supported by this device");
                }
                if (request.timeout.count() <= 0) {
                    throw ValidationError("command timeout must be positive");
                }
                if (!request.operation) {
                    static_cast<void>(validate_command_values(request.raw_values, *capability));
                }

                auto handle =
                    request.completion
                        ? *request.completion
                        : CommandHandle{
                              next_command_id(),
                              request.operation ? CommandFamily::kDiscreteOperation : command_family(request.semantic),
                              request.operation
                                  ? (request.ack_expectation == AckExpectation::kRequired ? AckSupport::kRequired
                                                                                          : AckSupport::kUnsupported)
                                  : capability->ack_support,
                              request.ack_expectation};
                const auto deadline = request.deadline ? *request.deadline : runtime::deadline_after(request.timeout);
                auto lease = execution_gate_->acquire(true, deadline, request.cancellation, request.generation_valid);
                if (request.generation_valid && !request.generation_valid()) {
                    throw ClosedError("command generation ended before execution");
                }
                dispatch_guard_->wait(deadline, request.cancellation, request.generation_valid);
                std::chrono::milliseconds settle{0};
                if constexpr (std::is_same_v<Device, l6::L6>) {
                    if (!request.operation && request.semantic == CommandSemantic::kTorques) {
                        dispatch_guard_->reserve(runtime::Clock::now() + std::chrono::milliseconds{20});
                        dispatch_guard_->wait(deadline, request.cancellation, request.generation_valid);
                        settle = std::find(request.raw_values.begin(), request.raw_values.end(), 0.0) !=
                                         request.raw_values.end()
                                     ? std::chrono::milliseconds{150}
                                     : std::chrono::milliseconds{20};
                    }
                }
                struct Progress {
                    std::mutex mutex;
                    uint64_t sent = 0;
                    bool active = true;
                };
                auto progress = std::make_shared<Progress>();
                auto observe = [progress, handle, request, deadline, guard = dispatch_guard_,
                                settle](const transport::SendResult &sent, bool final_send) mutable {
                    const std::lock_guard<std::mutex> lock(progress->mutex);
                    command_detail::CommandHandleAccess::record_send_result(handle, sent);
                    if (settle.count() > 0) {
                        if (sent.outcome == transport::SendOutcome::dispatched && !sent.error && sent.completed_at) {
                            guard->finish_send(*sent.completed_at + settle);
                        } else {
                            guard->finish_send();
                        }
                    }
                    if (!progress->active || handle.done()) {
                        return;
                    }
                    progress->sent += sent.sent_frames;
                    command_detail::CommandHandleAccess::record_sent(handle, progress->sent);
                    if (!final_send || sent.outcome != transport::SendOutcome::dispatched || sent.error ||
                        !sent.completed_at || *sent.completed_at >= deadline || runtime::Clock::now() >= deadline ||
                        (request.generation_valid && !request.generation_valid())) {
                        return;
                    }
                    if (request.dispatched) {
                        request.dispatched(progress->sent, *sent.completed_at);
                    } else {
                        command_detail::CommandHandleAccess::mark_dispatched(handle, progress->sent,
                                                                             *sent.completed_at);
                    }
                };
                // O30 writes the three safe mounted ranges sequentially, each
                // with its own ACK. Dispatch means the last range was sent;
                // earlier receipts (including a revision query) still count.
                auto final_send = [this, operation = request.operation](const runtime::TransactionPlan &plan) {
                    if constexpr (std::is_same_v<Device, l30::L30>) {
                        if (operation == DeviceOperation::kCalibrate) {
                            return !plan.frames.empty() &&
                                   ((plan.frames.back().id >> 21U) & 0xFU) == l30::protocol::kParentConfig &&
                                   ((plan.frames.back().id >> 13U) & 0xFFU) == l30::protocol::kSubCalibrate;
                        }
                    }
                    if constexpr (std::is_same_v<Device, o30::O30>) {
                        if (config_.o30_layout == O30Layout::kPythonV25) {
                            return true;
                        }
                        return !plan.command_family.empty() && !plan.frames.empty() &&
                               plan.frames.back().data[1] == o30::kSafeWriteSegments.back().si;
                    }
                    return true;
                };
                auto deactivate = [&] {
                    const std::lock_guard<std::mutex> lock(progress->mutex);
                    progress->active = false;
                };
                auto bounded = request;
                bounded.timeout = runtime::remaining(deadline);
                CommandOutcome outcome;
                try {
                    RuntimeAccess::execute_command(
                        *device_->runtime(), deadline, request.ack_expectation == AckExpectation::kNone,
                        std::move(observe), std::move(final_send), request.cancellation,
                        [&] {
                            if (request.operation) {
                                outcome = operate_device(*device_, *request.operation, bounded.timeout);
                                return;
                            }
                            if constexpr (std::is_same_v<Device, o30::O30>) {
                                if (config_.o30_layout == O30Layout::kPythonV25) {
                                    outcome = submit_v25(*device_, bounded);
                                    return;
                                }
                            }
                            outcome = submit_command(*device_, bounded);
                        },
                        [handle]() mutable { command_detail::CommandHandleAccess::reserve_send_result(handle); },
                        [guard = dispatch_guard_, settle] {
                            if (settle.count() > 0) {
                                guard->begin_send();
                            }
                        });
                } catch (...) {
                    deactivate();
                    throw;
                }
                deactivate();
                if (request.ack_expectation == AckExpectation::kRequired) {
                    if (request.generation_valid && !request.generation_valid()) {
                        throw ClosedError("command generation ended before acknowledgement");
                    }
                    if (runtime::Clock::now() >= deadline) {
                        throw TimeoutError("command acknowledgement deadline expired");
                    }
                    if (!outcome.acknowledged || handle.stage() != CommandStage::kDispatched) {
                        throw LifecycleError("command completed without its send receipt or required ACK");
                    }
                    if (request.acknowledged) {
                        request.acknowledged();
                    } else {
                        command_detail::CommandHandleAccess::mark_acknowledged(handle);
                    }
                } else if (handle.stage() != CommandStage::kDispatched) {
                    throw TimeoutError("command send did not complete before deadline");
                }
                return handle;
            }

            [[nodiscard]] TelemetrySample read(TelemetrySemantic semantic, std::chrono::milliseconds timeout) override {
                if (!capabilities_.supports(semantic)) {
                    throw UnsupportedOperationError("telemetry semantic is not supported by this device");
                }
                const auto deadline = runtime::deadline_after(timeout);
                auto lease = execution_gate_->acquire(false, deadline);
                dispatch_guard_->wait(deadline, {});
                if constexpr (std::is_same_v<Device, o30::O30>) {
                    if (config_.o30_layout == O30Layout::kPythonV25 && semantic != TelemetrySemantic::kForce &&
                        semantic != TelemetrySemantic::kForceFns) {
                        return read_v25(*device_, semantic, runtime::remaining(deadline), capabilities_);
                    }
                }
                return read_telemetry(*device_, semantic, runtime::remaining(deadline));
            }

            [[nodiscard]] CommandHandle operate(DeviceOperation operation, std::chrono::milliseconds timeout) override {
                if (timeout.count() <= 0) {
                    throw ValidationError("operation timeout must be positive");
                }
                if (!capabilities_.supports(operation)) {
                    throw UnsupportedOperationError("device operation is not supported by this device");
                }
                if (operation == DeviceOperation::kReadDeviceInfo) {
                    throw UnsupportedOperationError("read-device-info returns identity; use verify_identity");
                }
                const auto acknowledged = capabilities_.operations.acknowledged(operation);
                CommandHandle handle{next_command_id(), CommandFamily::kDiscreteOperation,
                                     acknowledged ? AckSupport::kRequired : AckSupport::kUnsupported,
                                     acknowledged ? AckExpectation::kRequired : AckExpectation::kNone};
                const auto deadline = runtime::deadline_after(timeout);
                auto lease = execution_gate_->acquire(false, deadline);
                dispatch_guard_->wait(deadline, {});
                const auto outcome = operate_device(*device_, operation, runtime::remaining(deadline));
                command_detail::CommandHandleAccess::mark_dispatched(handle, outcome.sent_frames);
                if (acknowledged) {
                    if (!outcome.acknowledged) {
                        throw LifecycleError("operation completed without its required ACK");
                    }
                    command_detail::CommandHandleAccess::mark_acknowledged(handle);
                }
                return handle;
            }

            [[nodiscard]] HandSnapshot snapshot(Model model, HandSide side,
                                                const DeviceCapabilities &capabilities) const override {
                auto result = unified_snapshot(device_->get_snapshot(), model, side, capabilities);
                if constexpr (std::is_same_v<Device, o30::O30>) {
                    if (config_.o30_layout == O30Layout::kPythonV25) {
                        project_v25(*device_, result, capabilities);
                        return result;
                    }
                    const auto feedback = [&](auto &sample, const char *domain) {
                        if (!sample) {
                            return;
                        }
                        const auto native = device_->runtime()->snapshot(domain);
                        if (native && native->origin == protocol::Origin::command_feedback &&
                            native->received_at == sample->meta.collected_at) {
                            sample->meta.delivery = TelemetryDelivery::kCommandResponse;
                        }
                    };
                    feedback(result.angles, "angle/relay_");
                    feedback(result.configured_speeds, "speed/relay_");
                    feedback(result.configured_accelerations, "acceleration/relay_");
                    feedback(result.current_limits, "current_limit");
                    feedback(result.stall_detection_times, "stall/times_relay_");
                    feedback(result.stall_thresholds, "stall/thresholds_relay_");
                    feedback(result.stall_hold_currents, "stall/hold_relay_");
                }
                return result;
            }

            [[nodiscard]] HandHealth health() const override {
                auto health = device_->health();
                auto current = snapshot(Model::kUnknown, HandSide::kUnknown, capabilities_);
                const auto now = HealthClock::now();
                const auto &capabilities = capabilities_;
                const auto update = [&](auto &sample, TelemetrySemantic semantic, const char *name) {
                    if (!sample) {
                        return;
                    }
                    complete_freshness(sample->meta, capabilities.telemetry_channel(semantic), now);
                    const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                                      [name](const auto &metrics) { return metrics.channel == name; });
                    if (channel == health.channels.end()) {
                        return;
                    }
                    channel->data_age.valid = sample->meta.valid;
                    channel->data_age.stale = sample->meta.stale;
                    channel->data_age.generation = health.connection_generation;
                    channel->data_age.age = sample->meta.age;
                    channel->feedback_samples = std::max<uint64_t>(channel->feedback_samples, 1);
                };
                update(current.angles, TelemetrySemantic::kAngles, "angles");
                if constexpr (std::is_same_v<Device, o30::O30>) {
                    if (config_.o30_layout == O30Layout::kPythonV25) {
                        update(current.measured_speeds, TelemetrySemantic::kMeasuredSpeeds, "speeds");
                        update(current.voltages, TelemetrySemantic::kVoltages, "voltages");
                        const auto ensure = [&](const char *name) {
                            if (std::none_of(health.channels.begin(), health.channels.end(),
                                             [name](const auto &channel) { return channel.channel == name; })) {
                                ChannelMetrics channel;
                                channel.channel = name;
                                health.channels.push_back(std::move(channel));
                            }
                        };
                        if (current.torque_limits) {
                            ensure("torque_limits");
                            update(current.torque_limits, TelemetrySemantic::kTorqueLimits, "torque_limits");
                        }
                        if (current.motion_times) {
                            ensure("motion_times");
                            update(current.motion_times, TelemetrySemantic::kMotionTimes, "motion_times");
                        }
                    }
                }
                // These configuration channels also declare freshness bounds.
                update(current.configured_speeds, TelemetrySemantic::kConfiguredSpeeds, "speeds");
                return health;
            }

            [[nodiscard]] DeviceIdentity verify_identity(HandSide side, std::chrono::milliseconds timeout) override {
                const auto deadline = runtime::deadline_after(timeout);
                auto lease = execution_gate_->acquire(false, deadline);
                dispatch_guard_->wait(deadline, {});
                return read_identity(*device_, side, runtime::remaining(deadline));
            }

            void start_polling(const PollingPlan &plan) override {
                if constexpr (std::is_same_v<Device, o30::O30>) {
                    if (config_.o30_layout == O30Layout::kPythonV25) {
                        std::vector<runtime::DeviceRuntime::PollTask> tasks;
                        for (const auto &channel : plan) {
                            runtime::DeviceRuntime::PollTask task;
                            task.name = "v25/" + std::to_string(static_cast<unsigned>(channel.semantic));
                            task.period = channel.interval;
                            if (channel.semantic == TelemetrySemantic::kMeasuredSpeeds) {
                                task.name = "speeds";
                            }
                            if (channel.semantic == TelemetrySemantic::kVoltages) {
                                task.name = "voltages";
                            }
                            task.run = [this, semantic = channel.semantic](runtime::CancellationToken) {
                                if (semantic == TelemetrySemantic::kForce || semantic == TelemetrySemantic::kForceFns) {
                                    (void)read_telemetry(*device_, semantic, std::chrono::milliseconds{1000});
                                } else {
                                    (void)read_v25(*device_, semantic, std::chrono::milliseconds{100}, capabilities_);
                                }
                            };
                            tasks.push_back(std::move(task));
                        }
                        device_->runtime()->start_polling(std::move(tasks));
                        return;
                    }
                }
                start_device_polling(*device_, plan);
            }

            void assert_external_lifecycle() const override { device_->runtime()->assert_external_lifecycle(); }
            void cancel_commands() override { device_->runtime()->cancel_all(); }
            void stop_polling() override { device_->stop_polling(); }
            void recover() override { recover_device(*device_); }
            void close() override {
                std::exception_ptr error;
                try {
                    device_->stop();
                } catch (...) {
                    error = std::current_exception();
                }
                if (owned_session_) {
                    try {
                        owned_session_->close();
                    } catch (...) {
                        if (!error) {
                            error = std::current_exception();
                        }
                    }
                }
                if (error) {
                    std::rethrow_exception(error);
                }
            }

        private:
            HandDeviceConfig config_;
            DeviceCapabilities capabilities_;
            std::unique_ptr<Device> device_;
            std::shared_ptr<transport::Session> owned_session_;
            std::shared_ptr<ExecutionGate> execution_gate_;
            std::shared_ptr<DispatchGuard> dispatch_guard_;
        };

        template <typename Device, size_t JointCount>
        [[nodiscard]] std::unique_ptr<DeviceBackend> backend(std::unique_ptr<Device> device,
                                                             std::shared_ptr<transport::Session> owned_session,
                                                             const HandDeviceConfig &config) {
            return std::make_unique<ModelBackend<Device, JointCount>>(std::move(device), std::move(owned_session),
                                                                      config);
        }

        template <typename Side> [[nodiscard]] Side concrete_side(HandSide side) {
            return side == HandSide::kLeft ? Side::kLeft : Side::kRight;
        }
    } // namespace

    std::unique_ptr<DeviceBackend> make_backend(const HandDeviceConfig &config,
                                                const HandDevice::TransportFactory &transport_factory,
                                                const std::shared_ptr<transport::Session> &shared_session) {
        auto session = shared_session;
        std::shared_ptr<transport::Session> owned_session;
        if (transport_factory) {
            owned_session = transport::Session::create(transport_factory);
            session = owned_session;
        }
#ifdef __linux__
        else if (!session) {
            // The configuration-only facade also needs a recreatable transport.
            // Owning a single socket without a factory makes recover() fail.
            const auto interface_name = config.interface_name;
            const auto fd = declared_capabilities(config).transport.kind == TransportKind::kCanFd;
            owned_session = transport::Session::create(
                [interface_name, fd] { return std::make_unique<can::SocketCANTransport>(interface_name, fd); });
            session = owned_session;
        }
#endif
#if !defined(__linux__)
        if (!session) {
            throw UnsupportedOperationError(
                "HandDevice(config) has no default adapter Transport on this platform; "
                "pass make_pcan_transport_factory(), make_minicanfd_transport_factory() or a Session");
        }
#endif
        const bool injected = static_cast<bool>(session);
        switch (config.model) {
        case Model::kL6: {
            const auto side = concrete_side<l6::Side>(config.side);
            if (config.l6_protocol == L6Protocol::kHopV004) {
                auto hand =
                    std::make_unique<l6::HopL6>(side, session, config.address.value_or(l6::hop::kDefaultRequestId));
                return backend<l6::HopL6, l6::kJointCount>(std::move(hand), std::move(owned_session), config);
            }
            auto hand = injected
                            ? std::make_unique<l6::L6>(side, session, config.interface_name, config.angle_mapping_path)
                            : std::make_unique<l6::L6>(side, config.interface_name, config.angle_mapping_path);
            return backend<l6::L6, l6::kJointCount>(std::move(hand), std::move(owned_session), config);
        }
        case Model::kO6: {
            const auto side = concrete_side<o6::Side>(config.side);
            auto hand = injected
                            ? std::make_unique<o6::O6>(side, session, config.interface_name, config.angle_mapping_path)
                            : std::make_unique<o6::O6>(side, config.interface_name, config.angle_mapping_path);
            return backend<o6::O6, o6::kJointCount>(std::move(hand), std::move(owned_session), config);
        }
        case Model::kL20Lite: {
            const auto side = concrete_side<l20lite::Side>(config.side);
            auto hand =
                injected ? std::make_unique<l20lite::L20lite>(side, session, config.interface_name,
                                                              config.angle_mapping_path)
                         : std::make_unique<l20lite::L20lite>(side, config.interface_name, config.angle_mapping_path);
            return backend<l20lite::L20lite, l20lite::kJointCount>(std::move(hand), std::move(owned_session), config);
        }
        case Model::kL20: {
            const auto side = concrete_side<l20::Side>(config.side);
            auto hand =
                injected ? std::make_unique<l20::L20>(side, session, config.interface_name, config.angle_mapping_path)
                         : std::make_unique<l20::L20>(side, config.interface_name, config.angle_mapping_path);
            return backend<l20::L20, l20::kJointCount>(std::move(hand), std::move(owned_session), config);
        }
        case Model::kL30: {
            const auto node_id = static_cast<uint8_t>(config.address.value_or(l30::kDefaultNodeId));
            const auto host_id = static_cast<uint8_t>(config.host_address.value_or(l30::kDefaultHostId));
            auto hand = injected
                            ? std::make_unique<l30::L30>(session, node_id, host_id, config.auto_start_periodic_reports)
                            : std::make_unique<l30::L30>(config.interface_name, node_id, host_id,
                                                         config.auto_start_periodic_reports);
            return backend<l30::L30, l30::kJointCount>(std::move(hand), std::move(owned_session), config);
        }
        case Model::kO30: {
            const auto request_id = config.o30_layout == O30Layout::kPythonV25
                                        ? config.address.value_or(config.side == HandSide::kLeft ? 2U : 1U)
                                        : config.address.value_or(o30::protocol::kDefaultRequestId);
            auto hand = injected ? std::make_unique<o30::O30>(session, request_id)
                                 : std::make_unique<o30::O30>(config.interface_name, request_id);
            return backend<o30::O30, o30::kJointCount>(std::move(hand), std::move(owned_session), config);
        }
        case Model::kUnknown:
            break;
        }
        throw ValidationError("HandDevice model must not be Unknown");
    }
} // namespace linkerbot::hand::unified_detail
