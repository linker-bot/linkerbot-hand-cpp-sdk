#include "detail/backend.hpp"
#include "detail/cached_snapshot.hpp"
#include "detail/connection_initializer.hpp"
#include "detail/force.hpp"
#include "detail/o30_v25.hpp"
#include "detail/python_state.hpp"
#include "detail/python_transactions.hpp"
#include "detail/read_coordinator.hpp"

#include "detail/command_handle_access.hpp"
#include "detail/dispatch_guard.hpp"
#include "detail/execution_gate.hpp"
#include "detail/freshness.hpp"
#include "detail/model_adapters.hpp"
#include "detail/runtime_access.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <limits>
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
                if (config_.runtime_behavior == RuntimeBehavior::kPython) {
                    RuntimeAccess::resume_endpoint(*device_->runtime());
                    python_ = std::make_shared<PythonState>(config_);
                    RuntimeAccess::set_publication_handler(
                        *device_->runtime(),
                        [state = python_](const std::string &domain, const runtime::Sample &sample) {
                            state->publish(domain, sample);
                        });
                    reads_ = std::make_unique<ReadCoordinator<ReadValue>>(config_.read_waiter_capacity);
                    return;
                }
                RuntimeAccess::set_polling_gate(*device_->runtime(), [gate = execution_gate_, guard = dispatch_guard_](
                                                                         runtime::CancellationToken token) {
                    auto lease = gate->acquire(false, runtime::Deadline::max(), token);
                    guard->wait(runtime::Deadline::max(), token);
                    return lease;
                });
            }

            ~ModelBackend() override {
                if (python_) {
                    python_->stop();
                    RuntimeAccess::set_publication_handler(*device_->runtime(), {});
                }
                reads_closing_.store(true, std::memory_order_release);
                ++read_epoch_;
                if (reads_) {
                    reads_->invalidate();
                    device_->runtime()->cancel_all();
                    reads_.reset();
                }
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
                auto lease = request.execution_lease ? request.execution_lease
                                                     : execution_gate_->acquire(true, deadline, request.cancellation,
                                                                                request.generation_valid);
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
                        (request.generation_valid && !request.generation_valid() &&
                         !(request.delivery && request.delivery->preserved()))) {
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
                RuntimeAccess::AdmitSend admit_send;
                if (config_.runtime_behavior == RuntimeBehavior::kPython && request.delivery) {
                    admit_send = [delivery = request.delivery](const RuntimeAccess::Submission &submit, bool final) {
                        return delivery->admit(submit, final);
                    };
                }
                try {
                    with_windows([&] {
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
                            },
                            [delivery = request.delivery](const transport::SendReceipt &receipt, bool final) {
                                if (delivery) {
                                    delivery->accepted(receipt, final);
                                }
                            },
                            std::move(admit_send));
                        if (python_ && !request.operation && request.semantic == CommandSemantic::kAngles) {
                            RuntimeAccess::wait_command_feedback(*device_->runtime(), request.generation_valid);
                        }
                    });
                } catch (...) {
                    if (request.delivery && request.ack_expectation == AckExpectation::kNone) {
                        if (const auto receipt = request.delivery->retained()) {
                            receipt->wait();
                            RuntimeAccess::flush_sends(*device_->runtime());
                            if (const auto error = receipt->result().error) {
                                deactivate();
                                rethrow_as_hand_error(error, {}, ErrorCode::kTransport);
                            }
                        }
                    }
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
                return coordinated_read(semantic, timeout, false);
            }
            [[nodiscard]] TelemetrySample coordinated_read(TelemetrySemantic semantic,
                                                           std::chrono::milliseconds timeout, bool polling,
                                                           runtime::CancellationToken poll_cancel = {},
                                                           std::function<bool()> poll_demand = {}) {
                if (!capabilities_.supports(semantic)) {
                    throw UnsupportedOperationError("telemetry semantic is not supported by this device");
                }
                if (reads_) {
                    if (python_->disabled(semantic)) {
                        throw UnsupportedFirmwareError("optional telemetry is disabled for this generation");
                    }
                    const auto *channel = capabilities_.telemetry_channel(semantic);
                    if (channel && (channel->read_mode == ReadMode::kCommandCache ||
                                    channel->read_mode == ReadMode::kCommandResponse)) {
                        (void)read_telemetry(*device_, semantic, timeout);
                        try {
                            auto cached =
                                telemetry_sample(snapshot(config_.model, config_.side, capabilities_), semantic);
                            if (!std::visit([](const auto &value) { return value.meta.valid; }, cached)) {
                                throw UnsupportedOperationError(
                                    "no configuration response is cached for this generation");
                            }
                            return cached;
                        } catch (const DeviceNotReadyError &) {
                            throw UnsupportedOperationError("no configuration response is cached for this generation");
                        }
                    }
                    uint64_t baseline = 0;
                    try {
                        const auto previous =
                            telemetry_sample(snapshot(Model::kUnknown, HandSide::kUnknown, capabilities_), semantic);
                        baseline = std::visit(
                            [semantic](const auto &reading) {
                                if constexpr (std::is_same_v<std::decay_t<decltype(reading)>, ForceSample>) {
                                    for (const auto &field : reading.fields) {
                                        if (field.query_channel == semantic) {
                                            return field.meta.sequence;
                                        }
                                    }
                                    return uint64_t{0};
                                } else {
                                    return reading.meta.sequence;
                                }
                            },
                            previous);
                    } catch (const DeviceNotReadyError &) {
                        baseline = 0;
                    }
                    const auto deadline = runtime::deadline_after(timeout);
                    const auto valid = read_generation();
                    while (true) {
                        auto value = reads_->read_prepared(
                            static_cast<unsigned>(semantic), deadline,
                            [this, semantic, valid, polling](const std::shared_ptr<void> &lane) {
                                ReadValue result;
                                with_windows([&] { result = python_read(semantic, polling, valid, lane); }, valid);
                                return result;
                            },
                            [this, valid](const std::function<bool()> &demand) {
                                return execution_gate_->acquire(false, runtime::Deadline::max(), {},
                                                                [valid, demand] { return valid() && demand(); });
                            },
                            polling, poll_cancel, poll_demand);
                        auto sample = std::get<TelemetrySample>(std::move(value));
                        const bool fresh = std::visit(
                            [&](const auto &reading) { return reading.meta.valid && reading.meta.sequence > baseline; },
                            sample);
                        if (fresh) {
                            return sample;
                        }
                        // A caller joining after cache publication needs the next
                        // transaction. Frames buffered before delivery can still
                        // serve callers admitted before that publication.
                    }
                }
                return read_direct(semantic, timeout);
            }

            [[nodiscard]] TelemetrySample read_direct(TelemetrySemantic semantic, std::chrono::milliseconds timeout,
                                                      const std::function<bool()> &valid = {},
                                                      const std::shared_ptr<void> &prepared = {}) {
                if (!capabilities_.supports(semantic)) {
                    throw UnsupportedOperationError("telemetry semantic is not supported by this device");
                }
                const auto deadline = config_.runtime_behavior == RuntimeBehavior::kPython
                                          ? runtime::Deadline::max()
                                          : runtime::deadline_after(timeout);
                auto lease = prepared ? prepared : execution_gate_->acquire(false, deadline, {}, valid);
                dispatch_guard_->wait(deadline, {}, valid);
                if constexpr (std::is_same_v<Device, l20lite::L20lite>) {
                    if (python_ &&
                        (semantic == TelemetrySemantic::kForceFns || semantic == TelemetrySemantic::kForceFts ||
                         semantic == TelemetrySemantic::kForceFds || semantic == TelemetrySemantic::kForcePx)) {
                        read_python_scalar(*device_->runtime(), config_, semantic);
                        return telemetry_sample(snapshot(config_.model, config_.side, capabilities_), semantic);
                    }
                }
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
                const auto deadline = config_.runtime_behavior == RuntimeBehavior::kPython
                                          ? runtime::Deadline::max()
                                          : runtime::deadline_after(timeout);
                auto lease = execution_gate_->acquire(false, deadline);
                dispatch_guard_->wait(deadline, {});
                CommandOutcome outcome;
                with_windows([&] { outcome = operate_device(*device_, operation, runtime::remaining(deadline)); });
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
                if (python_) {
                    return filtered_snapshot(model, side, capabilities, true);
                }
                return native_snapshot(model, side, capabilities);
            }
            [[nodiscard]] HandSnapshot native_snapshot(Model model, HandSide side,
                                                       const DeviceCapabilities &capabilities) const {
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

            DeviceCapabilities observed_capabilities(DeviceCapabilities caps) const override {
                return python_ ? python_->capabilities(std::move(caps)) : caps;
            }
            HandSnapshot filtered_snapshot(Model model, HandSide side, const DeviceCapabilities &caps,
                                           bool include_feedback) const override {
                if (!python_) {
                    return native_snapshot(model, side, caps);
                }
                const auto view = python_->view(include_feedback);
                HandSnapshot result;
                RuntimeAccess::with_snapshot_view(*device_->runtime(), view.samples,
                                                  [&] { result = native_snapshot(model, side, caps); });
                python_->project(result, view, caps);
                return result;
            }
            void wait_publication(uint64_t sequence, runtime::Deadline deadline, bool include_feedback,
                                  const std::function<bool()> &generation_valid) const override {
                if (python_) {
                    python_->wait(sequence, deadline, include_feedback, generation_valid);
                }
            }
            void finish_initialization() override {
                if (python_) {
                    python_->initialized();
                }
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
                if (python_) {
                    python_->project_health(health);
                }
                return health;
            }

            [[nodiscard]] DeviceIdentity verify_identity(HandSide side, std::chrono::milliseconds timeout) override {
                if (reads_) {
                    const auto valid = read_generation();
                    auto value = reads_->read_prepared(
                        std::numeric_limits<unsigned>::max(), runtime::deadline_after(timeout),
                        [this, side, valid](const std::shared_ptr<void> &lane) {
                            ReadValue result;
                            with_windows([&] { result = identity_direct(side, config_.identity_timeout, valid, lane); },
                                         valid, true);
                            return result;
                        },
                        [this, valid](const std::function<bool()> &demand) {
                            return execution_gate_->acquire(false, runtime::Deadline::max(), {},
                                                            [valid, demand] { return valid() && demand(); });
                        });
                    return std::get<DeviceIdentity>(std::move(value));
                }
                return identity_direct(side, timeout);
            }

            [[nodiscard]] DeviceIdentity identity_direct(HandSide side, std::chrono::milliseconds timeout,
                                                         const std::function<bool()> &valid = {},
                                                         const std::shared_ptr<void> &prepared = {}) {
                const auto deadline = config_.runtime_behavior == RuntimeBehavior::kPython
                                          ? runtime::Deadline::max()
                                          : runtime::deadline_after(timeout);
                auto lease = prepared ? prepared : execution_gate_->acquire(false, deadline, {}, valid);
                dispatch_guard_->wait(deadline, {}, valid);
                auto identity = read_identity(*device_, side, runtime::remaining(deadline));
                if (python_) {
                    const auto view = python_->view();
                    const auto previous = view.samples.find("python/identity");
                    std::optional<DeviceIdentity> expected;
                    if (previous != view.samples.end()) {
                        expected = std::any_cast<DeviceIdentity>(previous->second.value);
                    }
                    validate_connection_identity(identity, config_, expected ? &*expected : nullptr,
                                                 "refresh_device_info");
                    auto provenance = device_->runtime()->response_provenance();
                    identity.meta.valid = true;
                    identity.meta.stale = false;
                    identity.meta.origin = TelemetryValueOrigin::kDeviceConfigured;
                    identity.meta.delivery = TelemetryDelivery::kQueryResponse;
                    identity.meta.collected_at = identity.timestamp;
                    identity.meta.age = runtime::Clock::now() - identity.timestamp;
                    identity.meta.connection_generation = provenance.device_generation;
                    identity.meta.sequence = 1;
                    provenance.received_at = identity.timestamp;
                    device_->runtime()->publish("python/identity", identity, provenance);
                    identity = std::any_cast<DeviceIdentity>(python_->view().samples.at("python/identity").value);
                }
                return identity;
            }

            TelemetrySample initialize_read(TelemetrySemantic semantic, std::chrono::milliseconds timeout) override {
                TelemetrySample result;
                with_windows([&] { result = read_direct(semantic, timeout); });
                return result;
            }
            DeviceIdentity initialize_identity(HandSide side, std::chrono::milliseconds timeout) override {
                DeviceIdentity result;
                with_windows([&] { result = identity_direct(side, timeout); }, {}, true);
                return result;
            }
            void start_polling(const PollingPlan &plan) override {
                if (reads_) {
                    if (!python_->replace(plan)) {
                        return;
                    }
                    runtime::DeviceRuntime::PollTask task;
                    task.name = "python-feedback";
                    task.period = std::chrono::milliseconds{1};
                    task.next = [state = python_] { return state->next_poll(); };
                    task.run = [this](runtime::CancellationToken token) {
                        const auto ticket = python_->take_poll();
                        if (!ticket || token.cancelled()) {
                            return;
                        }
                        try {
                            (void)coordinated_read(
                                ticket->semantic, config_.transaction_timeout, true, token,
                                [state = python_, ticket = *ticket] { return state->poll_needed(ticket); });
                        } catch (const TimeoutError &) {
                            return;
                        } catch (const ClosedError &) {
                            return;
                        } catch (const ProtocolError &) {
                            return;
                        } catch (const UnsupportedFirmwareError &) {
                            return;
                        }
                    };
                    device_->runtime()->start_polling({std::move(task)});
                    return;
                }
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
                            task.run = [this, semantic = channel.semantic](runtime::CancellationToken token) {
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
            void stop_admission() override {
                if (python_) {
                    python_->stop();
                }
                if (reads_) {
                    RuntimeAccess::stop_endpoint(*device_->runtime());
                }
            }
            void resume_admission() override {
                if (reads_) {
                    reads_->drain();
                    RuntimeAccess::resume_endpoint(*device_->runtime());
                    python_->resume();
                    reads_->activate();
                }
            }
            void cancel_commands() override {
                if (python_) {
                    python_->stop();
                }
                ++read_epoch_;
                if (reads_) {
                    reads_->invalidate();
                }
                device_->runtime()->cancel_all();
            }
            void stop_polling() override {
                if (python_) {
                    python_->stop_polling();
                }
                device_->stop_polling();
            }
            void wait_quiet(std::chrono::milliseconds quiet, runtime::Deadline deadline,
                            const std::function<bool()> &cancelled) override {
                RuntimeAccess::wait_endpoint_quiet(*device_->runtime(), quiet, deadline, cancelled);
            }
            std::shared_ptr<void> prepare_command(runtime::CancellationToken cancellation,
                                                  const std::function<bool()> &valid) override {
                return execution_gate_->acquire(true, runtime::Deadline::max(), cancellation, valid);
            }
            void drain_sends(runtime::Deadline deadline) override {
                RuntimeAccess::drain_sends(*device_->runtime(), deadline);
            }
            void set_execution_deadline(runtime::Deadline deadline) override {
                execution_deadline_.store(deadline.time_since_epoch().count(), std::memory_order_release);
            }
            void recover() override {
                ++read_epoch_;
                if (reads_) {
                    reads_->invalidate();
                    device_->runtime()->cancel_all();
                    reads_->drain();
                }
                recover_device(*device_);
                if (python_) {
                    python_->reset();
                }
                if (reads_) {
                    reads_->activate();
                }
            }
            void close() override {
                if (python_) {
                    python_->stop();
                }
                reads_closing_.store(true, std::memory_order_release);
                ++read_epoch_;
                if (reads_) {
                    reads_->invalidate();
                    device_->runtime()->cancel_all();
                    reads_->drain();
                }
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
            TelemetrySample python_read(TelemetrySemantic semantic, bool polling, const std::function<bool()> &valid,
                                        const std::shared_ptr<void> &lane) {
                if (python_->disabled(semantic)) {
                    throw UnsupportedFirmwareError("optional telemetry is disabled for this generation");
                }
                const auto sent = device_->runtime()->runtime_health().sent_frames;
                try {
                    (void)read_direct(semantic, config_.transaction_timeout, valid, lane);
                    auto result = telemetry_sample(snapshot(config_.model, config_.side, capabilities_), semantic);
                    if (auto *force = std::get_if<ForceSample>(&result)) {
                        force->fields.erase(
                            std::remove_if(force->fields.begin(), force->fields.end(),
                                           [semantic](const auto &field) { return field.query_channel != semantic; }),
                            force->fields.end());
                        if (semantic != TelemetrySemantic::kForce) {
                            force->raw.clear();
                            force->tactile.clear();
                            force->sensors.clear();
                            force->shape = {5};
                            for (const auto &field : force->fields) {
                                force->meta = field.meta;
                                for (const auto &finger : field.raw) {
                                    if (finger) {
                                        for (const auto value : *finger) {
                                            force->raw.push_back(static_cast<uint8_t>(value));
                                        }
                                    }
                                }
                            }
                        }
                    }
                    python_->read_succeeded(semantic, polling);
                    return result;
                } catch (const HandError &error) {
                    python_->read_failed(semantic, device_->runtime()->runtime_health().sent_frames > sent, error,
                                         polling);
                    throw;
                }
            }
            using ReadValue = std::variant<TelemetrySample, DeviceIdentity>;
            std::function<bool()> read_generation() {
                const auto epoch = read_epoch_.load(std::memory_order_acquire);
                return [this, epoch] {
                    return !reads_closing_.load(std::memory_order_acquire) &&
                           read_epoch_.load(std::memory_order_acquire) == epoch;
                };
            }
            void with_windows(const std::function<void()> &execute, const std::function<bool()> &valid = {},
                              bool identity = false) {
                if (config_.runtime_behavior != RuntimeBehavior::kPython) {
                    execute();
                    return;
                }
                // Keep policy-owned callbacks out of native exception unwinding.
                // The out-of-line scope owns those objects only on the Python path.
                execute_python_transactions(
                    *device_->runtime(), config_, execute,
                    runtime::Deadline{runtime::Clock::duration{execution_deadline_.load(std::memory_order_acquire)}},
                    valid, identity);
            }
            HandDeviceConfig config_;
            DeviceCapabilities capabilities_;
            std::unique_ptr<Device> device_;
            std::shared_ptr<transport::Session> owned_session_;
            std::shared_ptr<ExecutionGate> execution_gate_;
            std::shared_ptr<DispatchGuard> dispatch_guard_;
            std::unique_ptr<ReadCoordinator<ReadValue>> reads_;
            std::shared_ptr<PythonState> python_;
            std::atomic<uint64_t> read_epoch_{0};
            std::atomic<bool> reads_closing_{false};
            std::atomic<runtime::Clock::duration::rep> execution_deadline_{
                runtime::Deadline::max().time_since_epoch().count()};
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
        if (session && config.runtime_behavior == RuntimeBehavior::kPython) {
            RuntimeAccess::retire_closed_profiles(session);
        }
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
