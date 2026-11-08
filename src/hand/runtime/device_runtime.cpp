#include "../detail/checked_optional.hpp"
#include "../detail/failure_kind.hpp"
#include "../unified/detail/runtime_access.hpp"
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <linkerbot/can/dispatcher.hpp>
#include <linkerbot/can/errors.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/runtime/device_runtime.hpp>
#include <map>
#include <mutex>
#include <set>
#include <string_view>
#include <thread>
#include <utility>

namespace linkerbot::hand::runtime {
    Deadline deadline_after(std::chrono::milliseconds timeout) {
        if (timeout.count() <= 0) {
            throw ValidationError("timeout must be positive");
        }
        const auto now = Clock::now();
        const auto available = std::chrono::duration_cast<std::chrono::milliseconds>(Deadline::max() - now);
        return timeout >= available ? Deadline::max() : now + timeout;
    }
    std::chrono::milliseconds remaining(Deadline deadline) {
        const auto now = Clock::now();
        if (now >= deadline) {
            throw TimeoutError("operation deadline expired");
        }
        return std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
    }
    namespace {
        template <class Predicate>
        bool wait_for_state(std::condition_variable &changed, std::unique_lock<std::mutex> &lock, Deadline deadline,
                            Predicate ready) {
            // Older libstdc++ can overflow while converting max() between clocks.
            // An unbounded wait must still release the mutex for its producer.
            if (deadline == Deadline::max()) {
                changed.wait(lock, std::move(ready));
                return true;
            }
            return changed.wait_until(lock, deadline, std::move(ready));
        }
        LifecycleState lifecycle_state(DeviceState state) noexcept {
            switch (state) {
            case DeviceState::ready:
                return LifecycleState::kOpen;
            case DeviceState::closing:
                return LifecycleState::kClosing;
            case DeviceState::closed:
                return LifecycleState::kClosed;
            case DeviceState::recovering:
                return LifecycleState::kRecovering;
            case DeviceState::faulted:
                return LifecycleState::kFaulted;
            }
            return LifecycleState::kFaulted;
        }

        ConnectionState connection_state(DeviceState state) noexcept {
            switch (state) {
            case DeviceState::ready:
                return ConnectionState::kConnected;
            case DeviceState::recovering:
                return ConnectionState::kReconnecting;
            case DeviceState::faulted:
                return ConnectionState::kFaulted;
            case DeviceState::closing:
            case DeviceState::closed:
                return ConnectionState::kClosed;
            }
            return ConnectionState::kFaulted;
        }

        LatencyMetrics latency_metrics(std::vector<HealthDuration> values) {
            LatencyMetrics metrics;
            metrics.count = values.size();
            if (values.empty()) {
                return metrics;
            }
            std::sort(values.begin(), values.end());
            const auto percentile = [&](size_t percentage) {
                const auto index = ((values.size() - 1) * percentage + 50) / 100;
                return values[index];
            };
            metrics.p50 = percentile(50);
            metrics.p95 = percentile(95);
            metrics.p99 = percentile(99);
            return metrics;
        }

        bool has_prefix(std::string_view value, std::string_view prefix) noexcept {
            return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
        }

        struct SnapshotView {
            const DeviceRuntime *runtime;
            const std::map<std::string, Sample> *samples;
        };
        thread_local std::optional<SnapshotView> current_snapshot_view;
        thread_local std::optional<size_t> construction_inbox_capacity;

        std::string canonical_channel(std::string_view domain) {
            if (domain == "angle" || domain == "angles") {
                return "angles";
            }
            if (domain == "speed" || domain == "speeds") {
                return "speeds";
            }
            if (domain == "current" || domain == "currents" || domain == "current_limit") {
                return "currents";
            }
            if (domain == "force" || domain == "force_sensor" || domain == "force_sensor_all" || domain == "tactile" ||
                domain == "tip_force" || has_prefix(domain, "force_sensor_") || has_prefix(domain, "tactile/") ||
                has_prefix(domain, "tip_force_")) {
                return "force";
            }
            if (domain == "temperature" || domain == "temperatures") {
                return "temperatures";
            }
            if (domain == "fault" || domain == "faults") {
                return "faults";
            }
            if (domain == "version" || has_prefix(domain, "version_")) {
                return "version";
            }
            if (domain == "stall" || has_prefix(domain, "stall_")) {
                return "stall";
            }
            return std::string(domain);
        }

        double requested_hz(std::chrono::microseconds period) {
            return 1'000'000.0 / static_cast<double>(period.count());
        }

        std::string canonical_command_family(std::string_view family) {
            if (family == "angle" || family == "angles") {
                return "angle_targets";
            }
            if (family == "speed" || family == "speeds") {
                return "speed_targets";
            }
            if (family == "current" || family == "current_limit" || family == "currents") {
                return "current_limits";
            }
            if (family == "torque" || family == "torques") {
                return "torque_targets";
            }
            if (family == "acceleration") {
                return "acceleration_targets";
            }
            if (family == "fault" || family == "faults") {
                return "fault_reset";
            }
            if (has_prefix(family, "stall")) {
                return "stall";
            }
            if (has_prefix(family, "config")) {
                return "configuration";
            }
            return std::string(family);
        }

        std::string_view error_type_name(ErrorCode code) noexcept {
            switch (code) {
            case ErrorCode::kValidation:
                return "ValidationError";
            case ErrorCode::kUnsupportedOperation:
                return "UnsupportedOperationError";
            case ErrorCode::kUnsupportedFirmware:
                return "UnsupportedFirmwareError";
            case ErrorCode::kIdentityMismatch:
                return "IdentityMismatchError";
            case ErrorCode::kDeviceNotReady:
                return "DeviceNotReadyError";
            case ErrorCode::kTransport:
                return "TransportError";
            case ErrorCode::kProtocol:
                return "ProtocolError";
            case ErrorCode::kTimeout:
                return "TimeoutError";
            case ErrorCode::kDeviceFault:
                return "DeviceFaultError";
            case ErrorCode::kQueueFull:
                return "QueueFullError";
            case ErrorCode::kPollingPlan:
                return "PollingPlanError";
            case ErrorCode::kClosed:
                return "ClosedError";
            case ErrorCode::kLifecycle:
                return "LifecycleError";
            case ErrorCode::kInternal:
                return "InternalError";
            case ErrorCode::kDiscovery:
                return "DiscoveryError";
            case ErrorCode::kNoHandFound:
                return "NoHandFoundError";
            case ErrorCode::kAmbiguousHand:
                return "AmbiguousHandError";
            case ErrorCode::kIncompleteIdentity:
                return "IncompleteIdentityError";
            }
            return "HandError";
        }

        ErrorInfo error_info(const HandError &error) {
            ErrorInfo info;
            info.code = error.code();
            info.message = error.what();
            info.error_type = error_type_name(error.code());
            info.context = error.context();
            return info;
        }

        struct EventSlot {
            std::mutex mutex;
            std::condition_variable cv;
            bool active = true;
            bool paused = false;
            uint64_t physical_generation = 0;
            uint64_t device_generation = 0;
            size_t executing = 0;
            std::string error;
            DeviceRuntime::EventHandler handler;
            const void *owner = nullptr;
        };
        thread_local EventSlot *current_event_slot = nullptr;
        thread_local const void *current_poller = nullptr;
        thread_local const void *current_receiver = nullptr;
        thread_local CancellationToken current_poll_cancel;
        struct ResponseContext {
            std::weak_ptr<DeviceRuntime> runtime;
            Sample sample;
        };
        thread_local std::map<const void *, ResponseContext> last_response;
        // The callback executor has independent lifetime. Releasing a device's final
        // reference in its user callback can therefore join that device's workers.
        class Events {
        public:
            Events() : thread_([this] { run(); }) {}
            ~Events() {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    stopping_ = true;
                    cv_.notify_all();
                }
                thread_.join();
            }
            bool post(std::shared_ptr<EventSlot> slot, Sample sample, std::weak_ptr<DeviceRuntime> owner) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stopping_ || queue_.size() >= 4096) {
                    return false;
                }
                queue_.push_back({std::move(slot), std::move(sample), std::move(owner)});
                cv_.notify_one();
                return true;
            }
            static Events &instance() {
                static Events events;
                return events;
            }

        private:
            void run() noexcept {
                for (;;) {
                    Event event;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        cv_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
                        if (stopping_) {
                            return;
                        }
                        event = std::move(queue_.front());
                        queue_.pop_front();
                    }
                    auto &slot = *event.slot;
                    {
                        std::lock_guard<std::mutex> lock(slot.mutex);
                        if (!slot.active || slot.paused ||
                            slot.physical_generation != event.sample.physical_generation ||
                            slot.device_generation != event.sample.device_generation) {
                            continue;
                        }
                        ++slot.executing;
                    }
                    current_event_slot = &slot;
                    auto owner = event.owner.lock();
                    try {
                        if (owner) {
                            const auto health = owner->runtime_health();
                            if (health.state == DeviceState::ready &&
                                health.physical_generation == event.sample.physical_generation &&
                                health.device_generation == event.sample.device_generation) {
                                slot.handler(event.sample);
                            }
                        }
                    } catch (const std::exception &error) {
                        std::lock_guard<std::mutex> lock(slot.mutex);
                        slot.error = error.what();
                        slot.active = false;
                    } catch (...) {
                        std::lock_guard<std::mutex> lock(slot.mutex);
                        slot.error = "non-standard event callback exception";
                        slot.active = false;
                    }
                    // Cancellation must cover the executor's temporary ownership too:
                    // a borrowed Session must be gone before its dispatcher can die.
                    // Keep the self-cancellation context during final-owner teardown.
                    owner.reset();
                    current_event_slot = nullptr;
                    {
                        std::lock_guard<std::mutex> lock(slot.mutex);
                        --slot.executing;
                        slot.cv.notify_all();
                    }
                }
            }
            // Queued/cancelled events must not keep a Runtime's borrowed Session alive.
            // Only executing callbacks retain it, inside the cancellation barrier.
            struct Event {
                std::shared_ptr<EventSlot> slot;
                Sample sample;
                std::weak_ptr<DeviceRuntime> owner;
            };
            std::mutex mutex_;
            std::condition_variable cv_;
            std::deque<Event> queue_;
            bool stopping_ = false;
            std::thread thread_;
        };
        void deactivate(const std::shared_ptr<EventSlot> &slot) {
            std::unique_lock<std::mutex> lock(slot->mutex);
            slot->active = false;
            if (current_event_slot != slot.get()) {
                slot->cv.wait(lock, [&] { return !slot->executing; });
            }
        }
        std::mutex runtimes_mutex;
        std::map<std::pair<const void *, std::string>, std::weak_ptr<DeviceRuntime>> runtimes;
    }
    namespace {
        struct ResponsePolicy {
            unified_detail::RuntimeAccess::PlanTransform transform;
            Deadline deadline;
            std::function<bool()> generation_valid;
            unified_detail::RuntimeAccess::ErrorCapture error_capture;
            unified_detail::RuntimeAccess::TimingCapture timing;
        };
        thread_local std::map<const DeviceRuntime *, ResponsePolicy> response_policies;
    }
    struct Request::State {
        mutable std::mutex mutex;
        std::condition_variable cv;
        bool done = false;
        std::exception_ptr error;
        Sample result;
        std::optional<Sample> response;
        std::exception_ptr buffered_error;
        bool buffered_isolation = true;
        bool published = false;
        TransactionPlan plan;
        Deadline deadline;
        CancellationToken external;
        std::function<bool()> generation_valid;
        CancellationToken poll_cancel;
        CancellationSource abort;
        std::optional<transport::SendReceipt> receipt;
        std::optional<transport::SendResult> sent_result;
        std::function<void()> cancel;
        std::function<void()> expire;
        std::thread::id owner;
        uint64_t generation = 0;
        ErrorContext context;
        bool polling_request = false;
        bool command_dispatched = false;
        bool command_terminal = false;
        bool return_after_send = false;
        bool scoped_command = false;
        bool scoped_response_policy = false;
        unified_detail::RuntimeAccess::ErrorCapture error_capture;
        unified_detail::RuntimeAccess::TimingCapture timing;
        bool response_deadline_armed = false;
        bool response_window_expiry = false;
        std::optional<Deadline> receipt_response_deadline() const {
            if (!plan.response_window || !receipt) {
                return std::nullopt;
            }
            const auto sent = receipt->result();
            if (!sent.completed_at || sent.error) {
                return std::nullopt;
            }
            const auto available =
                std::chrono::duration_cast<std::chrono::microseconds>(Deadline::max() - *sent.completed_at);
            return *plan.response_window >= available ? Deadline::max() : *sent.completed_at + *plan.response_window;
        }
        std::exception_ptr expired_response(Deadline now) const {
            const auto response_deadline = receipt_response_deadline();
            if (!response_deadline || now < *response_deadline) {
                return {};
            }
            if (*response_deadline <= deadline) {
                return std::make_exception_ptr(ResponseTimeoutError(
                    "device response window expired", context,
                    ::linkerbot::hand::detail::checked_value(plan.response_window, "missing response window"),
                    plan.late_guard));
            }
            return std::make_exception_ptr(timeout_error("device execution deadline expired"));
        }
        std::exception_ptr timeout_exception(const std::string &message) const {
            if (response_window_expiry) {
                return std::make_exception_ptr(ResponseTimeoutError(
                    message, context, plan.response_window.value_or(std::chrono::microseconds{0}), plan.late_guard));
            }
            return std::make_exception_ptr(timeout_error(message));
        }
        std::vector<std::pair<std::string, Sample>> deferred_updates;

        TimeoutError timeout_error(std::string message) const {
            // Both the receive worker and caller waits can expire this request.
            // Preserve claimed-send uncertainty in the stored error on every path.
            if (receipt) {
                const auto sent = receipt->result();
                if (sent.claimed_at && sent.outcome == transport::SendOutcome::pending) {
                    message += "; claimed backend I/O may still complete";
                }
            }
            return TimeoutError(std::move(message), context);
        }
    };
    Request::Request(std::shared_ptr<State> state) : state_(std::move(state)) {}
    Sample Request::wait_until(Deadline deadline) const {
        if (!state_) {
            throw ValidationError("empty runtime request");
        }
        std::unique_lock<std::mutex> lock(state_->mutex);
        const auto wait_deadline =
            state_->scoped_response_policy ? Deadline::max() : std::min(deadline, state_->deadline);
        if (!wait_for_state(state_->cv, lock, wait_deadline,
                            [&] { return state_->done || (state_->return_after_send && state_->sent_result); })) {
            lock.unlock();
            if (state_->expire) {
                state_->expire();
            }
            std::rethrow_exception(state_->timeout_exception("device request deadline expired"));
        }
        if (state_->return_after_send && state_->sent_result &&
            ::linkerbot::hand::detail::checked_value(state_->sent_result, "missing runtime state: state_->sent_result")
                    .outcome == transport::SendOutcome::dispatched &&
            !::linkerbot::hand::detail::checked_value(state_->sent_result, "missing runtime state: state_->sent_result")
                 .error) {
            return {}; // Optional feedback remains owned by the native receiver.
        }
        if (state_->error) {
            rethrow_as_hand_error(state_->error, state_->context, ErrorCode::kProtocol);
        }
        if (state_->return_after_send && state_->sent_result &&
            ::linkerbot::hand::detail::checked_value(state_->sent_result, "missing runtime state: state_->sent_result")
                .error) {
            rethrow_as_hand_error(::linkerbot::hand::detail::checked_value(state_->sent_result,
                                                                           "missing runtime state: state_->sent_result")
                                      .error,
                                  state_->context, ErrorCode::kTransport);
        }
        return state_->result;
    }
    Sample Request::wait() const { return wait_until(Deadline::max()); }
    transport::SendResult Request::wait_sent_until(Deadline deadline) const {
        if (!state_) {
            throw ValidationError("empty runtime request");
        }
        std::unique_lock<std::mutex> lock(state_->mutex);
        const auto wait_deadline =
            state_->scoped_response_policy ? Deadline::max() : std::min(deadline, state_->deadline);
        if (!wait_for_state(state_->cv, lock, wait_deadline,
                            [&] { return state_->done || state_->sent_result.has_value(); })) {
            lock.unlock();
            if (state_->expire) {
                state_->expire();
            }
            throw state_->timeout_error("request send deadline expired");
        }
        if (state_->error) {
            rethrow_as_hand_error(state_->error, state_->context, ErrorCode::kProtocol);
        }
        if (!state_->sent_result) {
            throw InternalError("request completed without a send receipt", state_->context);
        }
        const auto result =
            ::linkerbot::hand::detail::checked_value(state_->sent_result, "missing runtime state: state_->sent_result");
        if (result.error) {
            rethrow_as_hand_error(result.error, state_->context);
        }
        return result;
    }
    bool Request::completed() const {
        if (!state_) {
            return true;
        }
        std::lock_guard<std::mutex> lock(state_->mutex);
        return state_->done;
    }
    std::optional<transport::SendResult> Request::send_result() const {
        if (!state_ || !state_->receipt) {
            return std::nullopt;
        }
        return ::linkerbot::hand::detail::checked_value(state_->receipt, "missing runtime state: state_->receipt")
            .result();
    }
    void Request::cancel() const {
        if (!state_) {
            return;
        }
        if (!state_->scoped_response_policy || !state_->receipt) {
            state_->abort.cancel();
        }
        if (state_->cancel) {
            state_->cancel();
        }
    }

    struct DeviceRuntime::Impl {
        struct Observer {
            std::string domain;
            Decoder decoder;
            DecoderFactory factory;
        };
        struct Subscriber {
            std::string domain;
            std::shared_ptr<EventSlot> slot;
        };
        struct Poller {
            PollTask task;
            std::thread thread;
            bool finished = false;
        };
        struct ChannelState {
            ChannelMetrics metrics;
            double registered_requested_hz = 0.0;
            std::map<std::string, double> polling_rates;
            std::optional<HealthTimePoint> last_sample;
            std::deque<HealthDuration> latencies;
            std::deque<HealthTimePoint> sample_times;
        };
        struct CommandFamilyState {
            CommandFamilyMetrics metrics;
        };
        std::shared_ptr<transport::Session> session;
        std::weak_ptr<DeviceRuntime> owner;
        std::string identity;
        std::vector<transport::RouteKey> routes;
        size_t inbox_capacity = 512;
        std::shared_ptr<transport::RouteInbox> inbox;
        mutable std::mutex mutex;
        std::condition_variable cv;
        std::mutex lifecycle;
        std::mutex polling_operation;
        unified_detail::RuntimeAccess::PollingGate polling_gate;
        unified_detail::RuntimeAccess::PublicationHandler publication_handler;
        std::thread rx;
        bool rx_stop = false;
        bool destruction_requested = false;
        bool python_endpoint_policy = false;
        RuntimeHealth health;
        std::map<std::string, Sample> samples;
        std::map<std::string, ChannelState> channel_states;
        std::map<std::string, CommandFamilyState> command_family_states;
        std::map<std::string, uint64_t> domain_sequences;
        std::deque<HealthDuration> transaction_latencies;
        uint64_t error_revision = 0;
        uint64_t last_success_error_revision = 0;
        std::optional<ErrorInfo> recent_error;
        uint64_t consecutive_errors = 0;
        uint64_t reconnect_count = 0;
        std::optional<HealthTimePoint> last_successful_send;
        std::optional<HealthTimePoint> last_successful_receive;
        Deadline last_endpoint_receive{};
        std::optional<HealthTimePoint> last_successful_transaction;
        size_t active_pollers = 0;
        uint64_t next_transaction = 1;
        std::vector<std::shared_ptr<Request::State>> pending;
        std::map<std::string, Deadline> quarantine;
        std::set<std::string> poisoned;
        std::map<HandlerId, Observer> observers;
        std::map<HandlerId, Subscriber> subscribers;
        std::map<HandlerId, std::shared_ptr<EventSlot>> cancelling_subscribers;
        HandlerId next_id = 1;
        std::thread::id operation_owner;
        unsigned operation_depth = 0;
        uint64_t operation_generation = 0;
        std::vector<Deadline> operation_deadlines;
        struct Group {
            std::thread::id owner;
            std::vector<Deadline> deadlines;
            uint64_t generation = 0;
        };
        std::map<std::string, Group> groups;
        CancellationSource polling_cancel;
        CancellationSource polling_request_cancel;
        uint64_t polling_wake = 0;
        struct CommandExecution {
            bool send_only;
            Deadline deadline;
            CancellationToken cancellation;
            unified_detail::RuntimeAccess::SendObserver observer;
            unified_detail::RuntimeAccess::FinalSend final_send;
            std::function<void()> prepare_send;
            std::function<void()> accepted_send;
            unified_detail::RuntimeAccess::ReceiptAccepted receipt_accepted;
            unified_detail::RuntimeAccess::AdmitSend admit_send;
        };
        std::map<std::thread::id, CommandExecution> command_executions;
        struct SendAudit {
            std::string domain;
            transport::SendReceipt receipt;
            unified_detail::RuntimeAccess::SendObserver observer;
            bool final_send = false;
        };
        std::vector<SendAudit> send_audit;
        std::map<std::string, uint64_t> sent_by_domain;
        void reap_sends_locked() {
            for (auto it = send_audit.begin(); it != send_audit.end();) {
                const auto result = it->receipt.result();
                if (result.outcome == transport::SendOutcome::pending) {
                    ++it;
                    continue;
                }
                health.sent_frames += result.sent_frames;
                sent_by_domain[it->domain] += result.sent_frames;
                if (!result.error && result.completed_at) {
                    last_successful_send = *result.completed_at;
                }
                auto observer = std::move(it->observer);
                const bool final_send = it->final_send;
                it = send_audit.erase(it);
                // Retire accounting before invoking an observer which may throw.
                if (observer) {
                    observer(result, final_send);
                }
            }
        }
        std::vector<std::unique_ptr<Poller>> pollers;

        ErrorContext error_context(std::string operation = {}, std::string channel = {}) const {
            ErrorContext context;
            context.operation = std::move(operation);
            context.model = identity;
            context.channel = std::move(channel);
            context.generation = health.device_generation;
            return context;
        }

        ChannelState &channel_state_locked(std::string_view channel) {
            return canonical_channel_state_locked(canonical_channel(channel));
        }
        ChannelState &canonical_channel_state_locked(const std::string &canonical) {
            auto [it, inserted] = channel_states.try_emplace(canonical);
            if (inserted) {
                it->second.metrics.channel = canonical;
            }
            return it->second;
        }

        CommandFamilyState &command_family_state_locked(const std::string &family) {
            return canonical_command_family_state_locked(canonical_command_family(family));
        }
        CommandFamilyState &canonical_command_family_state_locked(const std::string &canonical) {
            auto [it, inserted] = command_family_states.try_emplace(canonical);
            if (inserted) {
                it->second.metrics.family = canonical;
            }
            return it->second;
        }

        static void update_polling_metrics_locked(ChannelState &state) {
            state.metrics.requested_hz = state.registered_requested_hz;
            for (const auto &[source, rate] : state.polling_rates) {
                (void)source;
                state.metrics.requested_hz += rate;
            }
            state.metrics.polling_active = !state.polling_rates.empty();
        }

        void record_channel_success_locked(const std::string &channel, HealthTimePoint received_at,
                                           std::optional<HealthDuration> latency, uint64_t feedback_sample_count) {
            record_channel_success_locked(channel_state_locked(channel), received_at, latency, feedback_sample_count);
        }
        void record_channel_success_locked(ChannelState &state, HealthTimePoint received_at,
                                           std::optional<HealthDuration> latency, uint64_t feedback_sample_count) {
            ++state.metrics.completed_reads;
            state.metrics.feedback_samples += feedback_sample_count;
            state.metrics.data_age.valid = true;
            state.metrics.data_age.stale = false;
            ++state.metrics.data_age.sequence;
            state.metrics.data_age.generation = health.device_generation;
            state.last_sample = received_at;
            state.sample_times.push_back(received_at);
            if (state.sample_times.size() > kHealthLatencyWindowCapacity) {
                state.sample_times.pop_front();
            }
            last_successful_receive = received_at;
            if (!python_endpoint_policy) {
                consecutive_errors = 0;
            }
            if (latency) {
                state.latencies.push_back(*latency);
                if (state.latencies.size() > kHealthLatencyWindowCapacity) {
                    state.latencies.pop_front();
                }
            }
        }

        void update_command_depth_locked(CommandFamilyState &state) {
            state.metrics.queue.depth = state.metrics.pending;
            state.metrics.queue.high_water = std::max(state.metrics.queue.high_water, state.metrics.queue.depth);
        }

        void complete_command_locked(CommandFamilyState &state) {
            if (state.metrics.pending != 0) {
                --state.metrics.pending;
            }
            update_command_depth_locked(state);
        }

        void finish_command_locked(Request::State &request, std::exception_ptr error, bool superseded = false) {
            if (request.plan.command_family.empty() || request.command_terminal) {
                return;
            }
            auto &state = command_family_state_locked(request.plan.command_family);
            if (superseded) {
                ++state.metrics.superseded;
                ++state.metrics.queue.dropped;
            } else if (!error) {
                ++state.metrics.acknowledged;
            } else {
                ++state.metrics.failed;
                const auto kind = detail::failure_kind(error);
                if (kind == detail::FailureKind::timeout) {
                    ++state.metrics.timed_out;
                } else if (kind == detail::FailureKind::lifecycle || kind == detail::FailureKind::closed) {
                    ++state.metrics.cancelled;
                }
            }
            complete_command_locked(state);
            request.command_terminal = true;
        }

        void sync_command_receipt_locked(Request::State &request) {
            if (request.plan.command_family.empty() || !request.receipt || !request.receipt->completed()) {
                return;
            }
            const auto result = request.receipt->result();
            if (!request.command_dispatched && result.outcome == transport::SendOutcome::dispatched) {
                ++command_family_state_locked(request.plan.command_family).metrics.dispatched;
                request.command_dispatched = true;
            }
            if (result.outcome == transport::SendOutcome::superseded) {
                finish_command_locked(request, {}, true);
            }
        }

        std::exception_ptr record_error_locked(std::exception_ptr error, ErrorContext context = {},
                                               ErrorCode fallback = ErrorCode::kInternal) {
            auto classified = classify_exception(std::move(error), std::move(context), fallback);
            try {
                std::rethrow_exception(classified);
            } catch (const HandError &hand_error) {
                health.last_error = hand_error.what();
                recent_error = error_info(hand_error);
                ++error_revision;
                ++consecutive_errors;
            }
            return classified;
        }

        bool stop_profile_endpoint_locked() {
            try {
                // This call gates admission and cancels only queued sends. The
                // existing drain_sends() waits for claimed I/O outside mutex.
                (void)session->stop_endpoint(identity, Clock::now());
                return true;
            } catch (const can::TimeoutError &) {
                return false;
            }
        }
        void require_ready() const {
            if (health.state != DeviceState::ready || (inbox && inbox->health().closed)) {
                ErrorContext context;
                context.model = identity;
                context.generation = health.device_generation;
                throw DeviceNotReadyError("device runtime is not ready: " + health.last_error, std::move(context));
            }
        }
        void invalidate_locked() {
            for (auto &[domain, sample] : samples) {
                (void)domain;
                sample.valid = false;
            }
            for (auto &[channel, state] : channel_states) {
                (void)channel;
                state.metrics.data_age.valid = false;
                state.metrics.data_age.stale = true;
                state.metrics.achieved_hz = 0.0;
                state.sample_times.clear();
            }
        }
        static void reset_observer(Observer &observer) {
            auto decoder = observer.factory();
            if (!decoder) {
                throw InternalError("report decoder factory returned an empty decoder");
            }
            observer.decoder = std::move(decoder);
        }
        void reset_observers_locked() {
            for (auto &[id, observer] : observers) {
                (void)id;
                reset_observer(observer);
            }
        }
        void publish_locked(const std::string &domain, const Sample &sample) {
            if (domain.empty() || !sample.value.has_value()) {
                return;
            }
            std::optional<HealthDuration> latency;
            if (sample.started_at != Deadline{} && sample.received_at >= sample.started_at) {
                latency = sample.received_at - sample.started_at;
            }
            record_channel_success_locked(canonical_channel(domain), sample.received_at, latency, 1);
            if (sample.received_at != Deadline{}) {
                last_successful_receive = sample.received_at;
            }
            samples[domain] = sample;
            if (publication_handler) {
                publication_handler(domain, sample);
                ++polling_wake;
                cv.notify_all();
            }
            for (const auto &[id, subscriber] : subscribers) {
                (void)id;
                if (subscriber.domain == domain && !Events::instance().post(subscriber.slot, sample, owner)) {
                    ++health.event_drops;
                }
            }
        }
        void finish_locked(const std::shared_ptr<Request::State> &state, std::exception_ptr error = {},
                           bool isolate = false, bool superseded = false) {
            const auto it = std::find(pending.begin(), pending.end(), state);
            if (it == pending.end()) {
                return;
            }
            // Caller-side timeout/cancellation can beat the receive worker's next
            // reap. Account any already-completed physical receipt before freezing
            // the public command's terminal frame count.
            std::optional<transport::SendResult> observed_send;
            if (state->receipt &&
                ::linkerbot::hand::detail::checked_value(state->receipt, "missing runtime state: state->receipt")
                    .completed()) {
                reap_sends_locked();
                observed_send =
                    ::linkerbot::hand::detail::checked_value(state->receipt, "missing harvested receipt").result();
            }
            if (error) {
                error = record_error_locked(std::move(error), state->context, ErrorCode::kProtocol);
                if (state->timing) {
                    state->timing->failed.store(true, std::memory_order_release);
                }
                if (state->error_capture) {
                    state->error_capture->store(error_revision, std::memory_order_release);
                }
            }
            sync_command_receipt_locked(*state);
            if (isolate) {
                if (!state->scoped_response_policy || !state->receipt) {
                    state->abort.cancel();
                } else if (!::linkerbot::hand::detail::checked_value(state->receipt, "missing accepted receipt")
                                .completed()) {
                    (void)stop_profile_endpoint_locked();
                }
                bool may_have_sent = state->receipt && ::linkerbot::hand::detail::checked_value(
                                                           state->receipt, "missing runtime state: state->receipt")
                                                           .result()
                                                           .claimed_at.has_value();
                if (may_have_sent) {
                    for (const auto &key : state->plan.keys) {
                        if (state->plan.poison_on_timeout) {
                            poisoned.insert(key);
                        } else {
                            quarantine[key] = Clock::now() + state->plan.late_guard;
                        }
                    }
                }
                if (!state->plan.preserve_last_success && !state->plan.domain.empty()) {
                    samples[state->plan.domain].valid = false;
                    auto &channel = channel_state_locked(state->plan.domain);
                    channel.metrics.data_age.valid = false;
                    channel.metrics.data_age.stale = true;
                }
            }
            std::optional<HealthDuration> completed_latency;
            std::optional<HealthTimePoint> completed_at;
            {
                std::lock_guard<std::mutex> request_lock(state->mutex);
                if (state->done) {
                    return;
                }
                // A profile success may only use a receipt whose observer has
                // already run. If delivery finishes after the harvest above,
                // retain the logical error so the facade drains/flushes it before
                // publishing dispatch. Native default completion remains unchanged.
                if (state->scoped_response_policy) {
                    state->sent_result = observed_send;
                } else if (state->receipt && ::linkerbot::hand::detail::checked_value(
                                                 state->receipt, "missing runtime state: state->receipt")
                                                 .completed()) {
                    state->sent_result = ::linkerbot::hand::detail::checked_value(
                                             state->receipt, "missing runtime state: state->receipt")
                                             .result();
                }
                state->error = error;
                if (!error && state->response) {
                    state->result = ::linkerbot::hand::detail::checked_value(state->response,
                                                                             "missing runtime state: state->response");
                    if (state->result.started_at != Deadline{} &&
                        state->result.received_at >= state->result.started_at) {
                        completed_latency = state->result.received_at - state->result.started_at;
                    }
                    if (state->result.received_at != Deadline{}) {
                        completed_at = state->result.received_at;
                    }
                    if (!state->published) {
                        publish_locked(state->plan.domain, state->result);
                    }
                    for (auto &update : state->deferred_updates) {
                        publish_locked(update.first, std::move(update.second));
                    }
                }
                if (!error && state->receipt) {
                    const auto result = ::linkerbot::hand::detail::checked_value(
                                            state->receipt, "missing runtime state: state->receipt")
                                            .result();
                    if (!completed_at && result.completed_at) {
                        completed_at = result.completed_at;
                    }
                    if (!completed_latency && result.completed_at && *result.completed_at >= result.accepted_at) {
                        completed_latency = *result.completed_at - result.accepted_at;
                    }
                }
                state->done = true;
                state->cv.notify_all();
            }
            pending.erase(it);
            health.in_flight = pending.size();
            finish_command_locked(*state, error, superseded);
            if (error) {
                try {
                    std::rethrow_exception(error);
                } catch (const TimeoutError &) {
                    ++health.timeouts;
                    if (state->polling_request && !state->plan.domain.empty()) {
                        ++channel_state_locked(state->plan.domain).metrics.timeouts;
                    }
                } catch (const LifecycleError &) {
                    ++health.cancelled;
                } catch (const std::exception &) {
                    ++health.protocol_errors;
                } catch (...) {
                    ++health.protocol_errors;
                }
            } else {
                ++health.completed;
                if (!state->scoped_response_policy) {
                    consecutive_errors = 0;
                }
                if (completed_at) {
                    last_successful_transaction = completed_at;
                    if (state->scoped_response_policy) {
                        last_success_error_revision = error_revision;
                    }
                }
                if (completed_latency) {
                    transaction_latencies.push_back(*completed_latency);
                    if (transaction_latencies.size() > kHealthLatencyWindowCapacity) {
                        transaction_latencies.pop_front();
                    }
                }
            }
            cv.notify_all();
        }
        void reap_locked() {
            reap_sends_locked();
            const auto requests = pending;
            for (const auto &state : requests) {
                if (state->receipt &&
                    ::linkerbot::hand::detail::checked_value(state->receipt, "missing runtime state: state->receipt")
                        .completed()) {
                    // Completion may race the initial harvest above. Deliver
                    // the facade's send observer before waking native callers;
                    // otherwise they can return before their handle is Dispatched.
                    reap_sends_locked();
                    sync_command_receipt_locked(*state);
                    std::lock_guard<std::mutex> request_lock(state->mutex);
                    state->sent_result = ::linkerbot::hand::detail::checked_value(
                                             state->receipt, "missing runtime state: state->receipt")
                                             .result();
                    if (!state->response_deadline_armed) {
                        if (const auto response_deadline = state->receipt_response_deadline()) {
                            state->response_window_expiry = *response_deadline <= state->deadline;
                            state->deadline = std::min(state->deadline, *response_deadline);
                            state->response_deadline_armed = true;
                        }
                    }
                    state->cv.notify_all();
                }
                if (state->generation_valid && !state->generation_valid()) {
                    finish_locked(state, std::make_exception_ptr(ClosedError("read generation ended")), true);
                } else if (state->external.cancelled() || state->poll_cancel.cancelled() ||
                           state->abort.token().cancelled()) {
                    finish_locked(state, std::make_exception_ptr(CancelledError("device request cancelled")), true);
                } else if (Clock::now() >= state->deadline &&
                           !(state->plan.response_window && state->response_window_expiry &&
                             (state->response || state->buffered_error) && state->receipt &&
                             ::linkerbot::hand::detail::checked_value(state->receipt, "missing response receipt")
                                 .completed())) {
                    finish_locked(state, state->timeout_exception("device request deadline expired"), true);
                } else if (state->receipt && ::linkerbot::hand::detail::checked_value(
                                                 state->receipt, "missing runtime state: state->receipt")
                                                 .completed()) {
                    const auto result = ::linkerbot::hand::detail::checked_value(
                                            state->receipt, "missing runtime state: state->receipt")
                                            .result();
                    if (result.error) {
                        finish_locked(state, classify_exception(result.error, state->context, ErrorCode::kTransport),
                                      true);
                    } else if (state->buffered_error) {
                        finish_locked(state, state->buffered_error, state->buffered_isolation);
                    } else if (state->response) {
                        finish_locked(state);
                    } else if (!state->plan.consume) {
                        state->response = Sample{{},
                                                 true,
                                                 ::linkerbot::hand::detail::checked_value(
                                                     result.completed_at, "missing runtime state: result.completed_at"),
                                                 result.generation,
                                                 health.device_generation,
                                                 0,
                                                 protocol::Origin::unknown,
                                                 result.accepted_at,
                                                 state->generation,
                                                 0};
                        finish_locked(state);
                    } else if (state->plan.optional_response_window &&
                               Clock::now() >= ::linkerbot::hand::detail::checked_value(
                                                   result.completed_at, "missing runtime state: result.completed_at") +
                                                   ::linkerbot::hand::detail::checked_value(
                                                       state->plan.optional_response_window,
                                                       "missing runtime state: state->plan.optional_response_window")) {
                        if (state->timing && canonical_channel(state->plan.domain) == "angles") {
                            // Only angle feedback is a declared Python command response.
                            // Other native optional observations do not gate logical success.
                            state->timing->failed.store(true, std::memory_order_release);
                            (void)record_error_locked(
                                std::make_exception_ptr(ResponseTimeoutError(
                                    "command feedback window expired", state->context,
                                    state->plan.optional_response_window.value_or(std::chrono::microseconds{0}),
                                    state->plan.late_guard)),
                                state->context, ErrorCode::kTimeout);
                            if (state->error_capture) {
                                state->error_capture->store(error_revision, std::memory_order_release);
                            }
                        }
                        if (publication_handler && state->scoped_response_policy) {
                            publication_handler(state->plan.domain, Sample{});
                        }
                        finish_locked(state, {}, true);
                    }
                }
            }
        }
        bool keys_available(const TransactionPlan &plan) const {
            for (const auto &group : plan.exclusion_groups) {
                for (const auto &state : pending) {
                    if (std::find(state->plan.exclusion_groups.begin(), state->plan.exclusion_groups.end(), group) !=
                        state->plan.exclusion_groups.end()) {
                        return false;
                    }
                }
            }
            for (const auto &key : plan.keys) {
                if (poisoned.count(key) && !plan.supersede_optional_feedback) {
                    throw DeviceNotReadyError("ambiguous late response: recover required for key " + key);
                }
                auto q = quarantine.find(key);
                if (q != quarantine.end() && Clock::now() < q->second) {
                    return false;
                }
                for (const auto &state : pending) {
                    if (std::find(state->plan.keys.begin(), state->plan.keys.end(), key) != state->plan.keys.end()) {
                        return false;
                    }
                }
            }
            return true;
        }
        void consume_locked(const transport::RoutedMessage &input) {
            if (input.generation != health.physical_generation) {
                ++health.late_frames;
                return;
            }
            bool matched = false;
            const auto requests = pending;
            for (const auto &state : requests) {
                if (!state->plan.consume || input.received_at >= state->deadline ||
                    input.received_at < state->receipt->result().accepted_at) {
                    continue;
                }
                if (state->response || state->buffered_error) {
                    if (state->plan.defer_errors_until_send) {
                        // This request still owns its response domain while the
                        // send is pending. Matching frames are not reports.
                        try {
                            if (state->plan.consume(input.message).kind != protocol::ProgressKind::unmatched) {
                                matched = true;
                                break;
                            }
                        } catch (...) {
                            matched = true;
                            break;
                        }
                    }
                    continue;
                }
                try {
                    if (const auto error = state->expired_response(Clock::now())) {
                        matched = true;
                        finish_locked(state, error, true);
                        break;
                    }
                    auto progress = state->plan.consume(input.message);
                    if (const auto error = state->expired_response(Clock::now())) {
                        matched = true;
                        finish_locked(state, error, true);
                        break;
                    }
                    if (progress.kind == protocol::ProgressKind::unmatched) {
                        continue;
                    }
                    matched = true;
                    if (state->scoped_command && Clock::now() >= state->deadline) {
                        finish_locked(
                            state,
                            std::make_exception_ptr(state->timeout_error("command response decoded after deadline")),
                            true);
                        break;
                    }
                    health.protocol_errors += progress.protocol_errors;
                    health.duplicate_frames += progress.duplicates;
                    health.conflicting_duplicate_frames += progress.conflicting_duplicates;
                    for (auto &update : progress.updates) {
                        Sample sample{std::move(update.second), true,
                                      input.received_at,        input.generation,
                                      health.device_generation, ++domain_sequences[update.first],
                                      progress.origin,          state->receipt->result().accepted_at,
                                      state->generation,        input.sequence};
                        if (state->scoped_command ||
                            (state->plan.defer_errors_until_send && state->plan.publish_after_send)) {
                            state->deferred_updates.emplace_back(update.first, std::move(sample));
                        } else {
                            publish_locked(update.first, std::move(sample));
                        }
                    }
                    if (progress.kind == protocol::ProgressKind::failed) {
                        auto error = progress.error
                                         ? progress.error
                                         : std::make_exception_ptr(ProtocolError("protocol response failed"));
                        if (state->plan.defer_errors_until_send && state->receipt && !state->receipt->completed()) {
                            state->buffered_error = error;
                            state->buffered_isolation = !progress.terminal_error;
                        } else {
                            finish_locked(state, error, !progress.terminal_error);
                        }
                        break;
                    }
                    if (progress.kind == protocol::ProgressKind::complete) {
                        state->response = Sample{std::move(progress.value), true,
                                                 input.received_at,         input.generation,
                                                 health.device_generation,  ++domain_sequences[state->plan.domain],
                                                 progress.origin,           state->receipt->result().accepted_at,
                                                 state->generation,         input.sequence};
                        // A received sample can be observed before the backend's
                        // write returns. ACK completion still waits for its receipt.
                        if (!state->plan.publish_after_send) {
                            publish_locked(state->plan.domain, *state->response);
                            state->published = true;
                        }
                    }
                    break;
                } catch (...) {
                    matched = true;
                    if (state->plan.defer_errors_until_send && state->receipt && !state->receipt->completed()) {
                        state->buffered_error = std::current_exception();
                    } else {
                        finish_locked(state, std::current_exception(), true);
                    }
                    break;
                }
            }
            if (!matched) {
                for (auto &[id, observer] : observers) {
                    (void)id;
                    if (poisoned.count(observer.domain) ||
                        (quarantine.count(observer.domain) && Clock::now() < quarantine.at(observer.domain))) {
                        continue;
                    }
                    bool reset = false;
                    try {
                        auto progress = observer.decoder(input.message);
                        if (progress.kind != protocol::ProgressKind::unmatched) {
                            matched = true;
                        }
                        health.protocol_errors += progress.protocol_errors;
                        health.duplicate_frames += progress.duplicates;
                        health.conflicting_duplicate_frames += progress.conflicting_duplicates;
                        if (progress.kind == protocol::ProgressKind::failed) {
                            if (!progress.protocol_errors) {
                                ++health.protocol_errors;
                            }
                            samples[observer.domain].valid = false;
                            const auto error =
                                progress.error
                                    ? progress.error
                                    : std::make_exception_ptr(ProtocolError("report decoder rejected a frame"));
                            record_error_locked(error, error_context("runtime.observe", observer.domain),
                                                ErrorCode::kProtocol);
                            reset = true;
                        }
                        if (progress.kind == protocol::ProgressKind::complete) {
                            publish_locked(observer.domain, Sample{std::move(progress.value),
                                                                   true,
                                                                   input.received_at,
                                                                   input.generation,
                                                                   health.device_generation,
                                                                   ++domain_sequences[observer.domain],
                                                                   progress.origin,
                                                                   {},
                                                                   0,
                                                                   input.sequence});
                            reset = true;
                        }
                    } catch (const std::exception &) {
                        matched = true;
                        ++health.protocol_errors;
                        record_error_locked(std::current_exception(), error_context("runtime.observe", observer.domain),
                                            ErrorCode::kProtocol);
                        samples[observer.domain].valid = false;
                        reset = true;
                    } catch (...) {
                        matched = true;
                        ++health.protocol_errors;
                        record_error_locked(std::current_exception(), error_context("runtime.observe", observer.domain),
                                            ErrorCode::kProtocol);
                        samples[observer.domain].valid = false;
                        reset = true;
                    }
                    // A factory failure reaches the receive worker's fault boundary;
                    // continuing with stale assembly would silently mix reports.
                    if (reset) {
                        reset_observer(observer);
                    }
                }
            }
            if (!matched) {
                ++health.late_frames;
            }
        }
        void run() noexcept {
            current_receiver = this;
            struct ClearReceiver {
                ~ClearReceiver() { current_receiver = nullptr; }
            } clear_receiver;
            try {
                uint64_t overflow = 0;
                for (;;) {
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        if (rx_stop) {
                            break;
                        }
                        reap_locked();
                    }
                    auto input = inbox->receive_until(Clock::now() + std::chrono::milliseconds(1));
                    std::lock_guard<std::mutex> lock(mutex);
                    if (rx_stop) {
                        break;
                    }
                    const auto route_health = inbox->health();
                    if (route_health.overflow != overflow) {
                        overflow = route_health.overflow;
                        const auto requests = pending;
                        for (const auto &state : requests) {
                            finish_locked(state,
                                          std::make_exception_ptr(
                                              can::QueueFullError("device inbox overflow invalidated assembly")),
                                          true);
                        }
                        invalidate_locked();
                        reset_observers_locked();
                        // Drop already-buffered fragments; never combine after loss.
                        while (inbox->try_receive()) {
                        }
                        continue;
                    }
                    reap_locked();
                    if (input) {
                        last_endpoint_receive = Clock::now();
                        cv.notify_all();
                        consume_locked(*input);
                    }
                    reap_locked();
                }
            } catch (...) {
                std::lock_guard<std::mutex> lock(mutex);
                if (!rx_stop) {
                    health.state = DeviceState::faulted;
                    record_error_locked(std::current_exception(), error_context("runtime.receive"),
                                        ErrorCode::kTransport);
                    invalidate_locked();
                    const auto requests = pending;
                    for (const auto &state : requests) {
                        finish_locked(state, std::current_exception(), true);
                    }
                    polling_cancel.cancel();
                    polling_request_cancel.cancel();
                    cv.notify_all();
                }
            }
        }
        void drain_sends() {
            std::vector<transport::SendReceipt> receipts;
            {
                std::lock_guard<std::mutex> lock(mutex);
                for (const auto &entry : send_audit) {
                    receipts.push_back(entry.receipt);
                }
            }
            // Claimed I/O cannot be revoked. The backend contract requires bounded
            // I/O, and lifecycle completion waits for its actual result.
            for (const auto &receipt : receipts) {
                try {
                    receipt.wait();
                } catch (...) {
                    if (!receipt.completed()) {
                        throw;
                    }
                } // Terminal backend failures are audited below.
            }
            std::lock_guard<std::mutex> lock(mutex);
            reap_sends_locked();
        }
        void stop_pollers(bool cancel_requests = true) {
            // Caller owns polling_operation; never join holding the state mutex.
            {
                std::lock_guard<std::mutex> lock(mutex);
                polling_cancel.cancel();
                if (cancel_requests) {
                    polling_request_cancel.cancel();
                }
                cv.notify_all();
            }
            for (auto &poller : pollers) {
                if (poller->thread.joinable()) {
                    poller->thread.join();
                }
            }
            pollers.clear();
            {
                std::lock_guard<std::mutex> lock(mutex);
                active_pollers = 0;
                for (auto &[name, state] : channel_states) {
                    (void)name;
                    state.polling_rates.clear();
                    update_polling_metrics_locked(state);
                    state.metrics.achieved_hz = 0.0;
                    state.sample_times.clear();
                }
            }
        }
    };

    DeviceRuntime::DeviceRuntime(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
    void DeviceRuntime::start() {
        auto &p = *impl_;
        p.owner = shared_from_this();
        p.inbox = p.session->subscribe_routes(p.routes, p.inbox_capacity);
        p.health.physical_generation = p.inbox->health().generation;
        p.rx = std::thread([&p] { p.run(); });
    }
    std::shared_ptr<DeviceRuntime> DeviceRuntime::acquire(std::shared_ptr<transport::Session> session,
                                                          std::string identity, std::vector<transport::RouteKey> routes,
                                                          size_t capacity) {
        if (construction_inbox_capacity) {
            capacity = ::linkerbot::hand::detail::checked_value(construction_inbox_capacity, "missing inbox scope");
        }
        if (!session || identity.empty()) {
            throw ValidationError("runtime requires a session and identity");
        }
        std::sort(routes.begin(), routes.end());
        if (routes.empty() || !capacity || std::adjacent_find(routes.begin(), routes.end()) != routes.end()) {
            throw ValidationError("runtime requires distinct routes and positive capacity");
        }
        std::lock_guard<std::mutex> lock(runtimes_mutex);
        for (auto it = runtimes.begin(); it != runtimes.end();) {
            if (it->second.expired()) {
                it = runtimes.erase(it);
            } else {
                ++it;
            }
        }
        const auto key = std::make_pair(session.get(), identity);
        if (auto existing = runtimes[key].lock()) {
            std::lock_guard<std::mutex> state(existing->impl_->mutex);
            existing->impl_->require_ready();
            auto current_routes = existing->impl_->routes;
            std::sort(current_routes.begin(), current_routes.end());
            if (current_routes != routes || existing->impl_->inbox_capacity != capacity) {
                throw ValidationError("device identity already has a different route/capacity binding");
            }
            if (existing->impl_->health.state == DeviceState::ready) {
                return existing;
            }
            throw ClosedError("existing device runtime must be recovered or released");
        }
        auto p = std::make_unique<Impl>();
        p->session = std::move(session);
        p->identity = std::move(identity);
        p->routes = std::move(routes);
        p->inbox_capacity = capacity;
        auto runtime = std::shared_ptr<DeviceRuntime>(new DeviceRuntime(std::move(p)));
        try {
            runtime->start();
        } catch (...) {
            ErrorContext context;
            context.operation = "runtime.acquire";
            context.model = key.second;
            rethrow_as_hand_error(std::current_exception(), std::move(context), ErrorCode::kTransport);
        }
        runtimes[key] = runtime;
        return runtime;
    }
    std::shared_ptr<DeviceRuntime> DeviceRuntime::borrow(can::CANDispatcher &dispatcher, std::string identity,
                                                         std::vector<transport::RouteKey> routes, size_t capacity) {
        return acquire(transport::Session::borrow(dispatcher), std::move(identity), std::move(routes), capacity);
    }
    DeviceRuntime::~DeviceRuntime() {
        // A standalone manager may release its last runtime on the dispatcher's RX
        // thread. Runtime workers are independent of that thread; Session's deleter
        // handles the subsequent dispatcher subscription cleanup externally.
        try {
            close_impl();
            // Destruction is nonthrowing; explicit close/stop remains the error-reporting path.
            // NOLINTNEXTLINE(bugprone-empty-catch)
        } catch (...) {
        }
    }
    Request DeviceRuntime::submit(TransactionPlan plan, Deadline deadline, CancellationToken cancellation) {
        std::function<bool()> generation_valid;
        if (const auto policy = response_policies.find(this); policy != response_policies.end()) {
            generation_valid = policy->second.generation_valid;
            if (generation_valid && !generation_valid()) {
                throw ClosedError("read generation ended before submission");
            }
            plan = policy->second.transform(std::move(plan));
            deadline = policy->second.deadline;
        }
        if (plan.response_window && (plan.response_window->count() <= 0 || !plan.consume)) {
            throw ValidationError("required response window needs a decoder and positive duration");
        }
        if (plan.frames.empty()) {
            throw ValidationError("transaction plan must contain frames");
        }
        if (plan.consume && plan.keys.empty()) {
            throw ValidationError("response transaction requires a correlation key");
        }
        if (plan.supersede_optional_feedback && (!plan.optional_response_window || !plan.consume)) {
            throw ValidationError("feedback supersession requires an optional response plan");
        }
        auto state = std::make_shared<Request::State>();
        state->plan = std::move(plan);
        state->deadline = deadline;
        state->external = cancellation;
        state->generation_valid = std::move(generation_valid);
        state->scoped_response_policy = response_policies.count(this) != 0;
        if (const auto policy = response_policies.find(this); policy != response_policies.end()) {
            state->error_capture = policy->second.error_capture;
            state->timing = policy->second.timing;
        }
        state->owner = std::this_thread::get_id();
        auto &p = *impl_;
        {
            std::unique_lock<std::mutex> lock(p.mutex);
            state->context = p.error_context("runtime.submit", state->plan.domain);
            if (const auto execution = p.command_executions.find(state->owner);
                execution != p.command_executions.end()) {
                state->external = execution->second.cancellation;
            }
            if (!state->plan.frames.empty()) {
                state->context.arbitration_id = state->plan.frames.front().id;
            }
            if (!state->plan.domain.empty()) {
                p.channel_state_locked(state->plan.domain);
            }
            if (p.operation_owner == state->owner && !p.operation_deadlines.empty()) {
                if (p.operation_generation != p.health.device_generation) {
                    throw ClosedError("operation belongs to a previous device generation", state->context);
                }
                deadline = std::min(deadline, p.operation_deadlines.back());
                state->deadline = deadline;
            }
            for (const auto &[name, group] : p.groups) {
                (void)name;
                if (group.owner == state->owner && !group.deadlines.empty()) {
                    if (group.generation != p.health.device_generation) {
                        throw ClosedError("named operation belongs to a previous device generation", state->context);
                    }
                    deadline = std::min(deadline, group.deadlines.back());
                }
            }
            state->deadline = deadline;
            if (current_poller == &p) {
                state->poll_cancel = p.polling_request_cancel.token();
            }
            for (;;) {
                p.require_ready();
                if (state->generation_valid && !state->generation_valid()) {
                    throw ClosedError("read generation ended waiting for the transaction lane");
                }
                if (state->external.cancelled() || state->poll_cancel.cancelled()) {
                    throw LifecycleError("device request cancelled", state->context);
                }
                if (Clock::now() >= deadline) {
                    throw TimeoutError("deadline expired waiting for device transaction key", state->context);
                }
                p.reap_locked();
                if (state->plan.supersede_optional_feedback &&
                    (!p.operation_depth || p.operation_owner == state->owner)) {
                    const auto pending = p.pending;
                    for (const auto &previous : pending) {
                        if (!previous->plan.optional_response_window) {
                            continue;
                        }
                        const bool overlap =
                            std::any_of(state->plan.keys.begin(), state->plan.keys.end(), [&](const auto &key) {
                                return std::find(previous->plan.keys.begin(), previous->plan.keys.end(), key) !=
                                       previous->plan.keys.end();
                            });
                        // Never abort a control's physical send merely because another
                        // target arrived. Only its remaining optional feedback may end.
                        if (overlap && previous->receipt &&
                            ::linkerbot::hand::detail::checked_value(previous->receipt,
                                                                     "missing runtime state: previous->receipt")
                                .completed()) {
                            p.finish_locked(previous, {}, true, true);
                        }
                    }
                    if (std::any_of(state->plan.keys.begin(), state->plan.keys.end(),
                                    [&](const auto &key) { return p.poisoned.count(key); })) {
                        state->plan.consume = {};
                        state->plan.optional_response_window.reset();
                    }
                }
                if ((!p.operation_depth || p.operation_owner == state->owner) && p.keys_available(state->plan)) {
                    break;
                }
                p.cv.wait_until(lock, std::min(deadline, Clock::now() + std::chrono::milliseconds(1)));
            }
            if (const auto execution = p.command_executions.find(state->owner);
                execution != p.command_executions.end() && execution->second.prepare_send) {
                execution->second.prepare_send();
            }
            std::weak_ptr<DeviceRuntime> weak = shared_from_this();
            std::weak_ptr<Request::State> weak_state = state;
            state->cancel = [weak, weak_state] {
                if (auto runtime = weak.lock()) {
                    if (auto request = weak_state.lock()) {
                        std::lock_guard<std::mutex> guard(runtime->impl_->mutex);
                        runtime->impl_->finish_locked(
                            request, std::make_exception_ptr(CancelledError("request cancelled")), true);
                    }
                }
            };
            state->expire = [weak, weak_state] {
                if (auto runtime = weak.lock()) {
                    if (auto request = weak_state.lock()) {
                        std::lock_guard<std::mutex> guard(runtime->impl_->mutex);
                        runtime->impl_->finish_locked(
                            request, request->timeout_exception("device request wait deadline expired"), true);
                    }
                }
            };
            state->generation = p.next_transaction++;
            state->polling_request = current_poller == &p;
            if (!state->plan.command_family.empty()) {
                auto &command = p.command_family_state_locked(state->plan.command_family);
                ++command.metrics.accepted;
                ++command.metrics.pending;
                ++command.metrics.queue.accepted;
                p.update_command_depth_locked(command);
            }
            unified_detail::RuntimeAccess::SendObserver send_observer;
            bool final_send = false;
            if (const auto execution = p.command_executions.find(state->owner);
                execution != p.command_executions.end()) {
                send_observer = execution->second.observer;
                final_send = execution->second.final_send(state->plan);
                state->return_after_send = execution->second.send_only && bool(state->plan.consume);
                state->scoped_command = true;
                // A failed accepted write must retain the last successful sample,
                // including native managers which otherwise invalidate on timeout.
                state->plan.preserve_last_success = true;
                state->plan.publish_after_send = true;
            }
            // Reserve audit bookkeeping before admission can start backend I/O.
            // Once accepted, close/recover must retain and drain this receipt even
            // if another allocation later fails.
            std::string audit_domain = state->plan.domain;
            if (p.send_audit.size() == p.send_audit.capacity()) {
                p.send_audit.reserve(std::max<size_t>(8, p.send_audit.capacity() * 2));
            }
            p.sent_by_domain.try_emplace(audit_domain, 0);
            p.pending.push_back(state); // registered before physical send or receipt
            p.health.in_flight = p.pending.size();
            try {
                if (state->generation_valid && !state->generation_valid()) {
                    throw ClosedError("read generation ended before physical admission");
                }
                transport::SendOptions options;
                options.deadline = deadline;
                options.cancellation = state->abort.token();
                options.endpoint = p.identity;
                options.family = state->plan.command_family;
                // Admission does not perform I/O. Install the receipt while still
                // holding runtime state so cancel cannot lose claimed-send evidence.
                const auto submit = [&] {
                    if (state->timing && !state->timing->started) {
                        state->timing->started = Clock::now();
                    }
                    return p.session->submit({state->plan.frames, state->plan.gap, state->plan.initial_delay}, options);
                };
                const auto execution = p.command_executions.find(state->owner);
                state->receipt = execution != p.command_executions.end() && execution->second.admit_send
                                     ? execution->second.admit_send(submit, final_send)
                                     : submit();
                p.send_audit.push_back(
                    {std::move(audit_domain),
                     ::linkerbot::hand::detail::checked_value(state->receipt, "missing runtime state: state->receipt"),
                     std::move(send_observer), final_send});
                if (const auto execution = p.command_executions.find(state->owner);
                    execution != p.command_executions.end()) {
                    if (execution->second.receipt_accepted) {
                        execution->second.receipt_accepted(
                            ::linkerbot::hand::detail::checked_value(state->receipt, "missing accepted receipt"),
                            final_send);
                    }
                    if (execution->second.accepted_send) {
                        execution->second.accepted_send();
                    }
                }
                p.reap_locked();
            } catch (...) {
                p.finish_locked(state, std::current_exception(), true);
            }
        }
        return Request(state);
    }
    Sample DeviceRuntime::execute(TransactionPlan plan, Deadline deadline, CancellationToken cancellation) {
        for (auto it = last_response.begin(); it != last_response.end();) {
            if (it->second.runtime.expired() || it->first == this) {
                it = last_response.erase(it);
            } else {
                ++it;
            }
        }
        auto request = submit(std::move(plan), deadline, cancellation);
        auto sample = response_policies.count(this) ? request.wait() : request.wait_until(deadline);
        if (sample.origin != protocol::Origin::unknown) {
            last_response[this] = {weak_from_this(), sample};
        }
        return sample;
    }
    void DeviceRuntime::command(TransactionPlan plan, Deadline deadline, CancellationToken cancellation) {
        if (plan.command_family.empty()) {
            plan.command_family = canonical_command_family(plan.domain);
        }
        (void)execute(std::move(plan), deadline, cancellation);
    }
    std::optional<Sample> DeviceRuntime::snapshot(const std::string &domain) const {
        if (current_snapshot_view && current_snapshot_view->runtime == this) {
            const auto &samples = *current_snapshot_view->samples;
            const auto found = samples.find(domain);
            return found == samples.end() ? std::nullopt : std::optional<Sample>{found->second};
        }
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const auto it = impl_->samples.find(domain);
        if (it == impl_->samples.end()) {
            return std::nullopt;
        }
        auto sample = it->second;
        if (impl_->health.state != DeviceState::ready || impl_->inbox->health().closed) {
            sample.valid = false;
        }
        return sample;
    }
    DeviceRuntime::HandlerId DeviceRuntime::observe(std::string domain, Decoder decoder) {
        if (domain.empty() || !decoder) {
            throw ValidationError("observer needs domain and pure decoder");
        }
        return observe_factory(std::move(domain), [decoder = std::move(decoder)] { return decoder; });
    }
    DeviceRuntime::HandlerId DeviceRuntime::observe_factory(std::string domain, DecoderFactory factory) {
        if (domain.empty() || !factory) {
            throw ValidationError("observer needs domain and decoder factory");
        }
        auto decoder = factory();
        if (!decoder) {
            throw ValidationError("report decoder factory returned an empty decoder");
        }
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->require_ready();
        for (const auto &[id, observer] : impl_->observers) {
            (void)id;
            if (observer.domain == domain) {
                throw ValidationError("domain already observed");
            }
        }
        const auto id = impl_->next_id++;
        impl_->channel_state_locked(domain);
        impl_->observers.emplace(id, Impl::Observer{std::move(domain), std::move(decoder), std::move(factory)});
        return id;
    }
    void DeviceRuntime::unobserve(HandlerId id) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->observers.erase(id);
    }
    DeviceRuntime::HandlerId DeviceRuntime::subscribe(std::string domain, EventHandler handler) {
        if (!handler) {
            throw ValidationError("event handler is empty");
        }
        auto slot = std::make_shared<EventSlot>();
        slot->handler = std::move(handler);
        slot->owner = impl_.get();
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->require_ready();
        slot->physical_generation = impl_->health.physical_generation;
        slot->device_generation = impl_->health.device_generation;
        const auto id = impl_->next_id++;
        if (!domain.empty()) {
            impl_->channel_state_locked(domain);
        }
        impl_->subscribers.emplace(id, Impl::Subscriber{std::move(domain), std::move(slot)});
        return id;
    }
    bool DeviceRuntime::unsubscribe(HandlerId id) {
        std::shared_ptr<EventSlot> slot;
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            auto it = impl_->subscribers.find(id);
            if (it != impl_->subscribers.end()) {
                slot = it->second.slot;
                impl_->cancelling_subscribers.emplace(id, slot);
                impl_->subscribers.erase(it);
            } else {
                const auto old = impl_->cancelling_subscribers.find(id);
                if (old == impl_->cancelling_subscribers.end()) {
                    return true;
                }
                slot = old->second;
            }
        }
        deactivate(slot);
        // A self-unsubscribe must retain the barrier for another caller until this
        // callback exits. External cancellation can remove the completed tombstone.
        if (current_event_slot != slot.get()) {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->cancelling_subscribers.erase(id);
            return true;
        }
        return false;
    }
    void DeviceRuntime::publish(const std::string &domain, std::any value) {
        (void)publish(domain, std::move(value), response_provenance());
    }
    Sample DeviceRuntime::response_provenance() const {
        const auto it = last_response.find(this);
        if (it == last_response.end() || it->second.runtime.lock().get() != this) {
            throw LifecycleError("publication requires a completed runtime response");
        }
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->require_ready();
        const auto &sample = it->second.sample;
        if (!sample.valid || sample.device_generation != impl_->health.device_generation ||
            sample.physical_generation != impl_->health.physical_generation) {
            throw ClosedError("response belongs to an invalid device generation");
        }
        return sample;
    }
    Sample DeviceRuntime::publish(const std::string &domain, std::any value, Sample sample) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->require_ready();
        if (const auto execution = impl_->command_executions.find(std::this_thread::get_id());
            execution != impl_->command_executions.end() && execution->second.cancellation.cancelled()) {
            throw ClosedError("command cache publication cancelled", impl_->error_context("runtime.publish", domain));
        }
        if (const auto execution = impl_->command_executions.find(std::this_thread::get_id());
            execution != impl_->command_executions.end() && Clock::now() >= execution->second.deadline) {
            throw TimeoutError("command cache publication deadline expired",
                               impl_->error_context("runtime.publish", domain));
        }
        if (!sample.valid || sample.device_generation != impl_->health.device_generation ||
            sample.physical_generation != impl_->health.physical_generation) {
            throw ClosedError("publication belongs to an invalid device generation");
        }
        // Typed publication may run after a queued report callback. Never replace a
        // newer sample with an older frame from the same physical/device generation.
        const auto existing = impl_->samples.find(domain);
        if (existing != impl_->samples.end() && existing->second.valid &&
            existing->second.received_at > sample.received_at) {
            return existing->second;
        }
        sample.value = std::move(value);
        sample.sequence = ++impl_->domain_sequences[domain];
        impl_->publish_locked(domain, sample);
        return sample;
    }
    void DeviceRuntime::invalidate(const std::string &domain) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto &channel = impl_->channel_state_locked(domain);
        channel.metrics.data_age.valid = false;
        channel.metrics.data_age.stale = true;
        impl_->samples[domain].valid = false;
    }
    DeviceRuntime::Operation::Operation(std::shared_ptr<DeviceRuntime> runtime, std::string group, Deadline deadline)
        : runtime_(std::move(runtime)), group_(std::move(group)), deadline_(deadline) {}
    DeviceRuntime::Operation::Operation(Operation &&other) noexcept
        : runtime_(std::move(other.runtime_)), group_(std::move(other.group_)), deadline_(other.deadline_),
          call_(std::move(other.call_)) {}
    DeviceRuntime::Operation &DeviceRuntime::Operation::operator=(Operation &&other) noexcept {
        if (this != &other) {
            if (runtime_) {
                runtime_->release_operation(group_);
            }
            runtime_ = std::move(other.runtime_);
            group_ = std::move(other.group_);
            deadline_ = other.deadline_;
            call_ = std::move(other.call_);
        }
        return *this;
    }
    DeviceRuntime::Operation::~Operation() {
        if (runtime_) {
            runtime_->release_operation(group_);
        }
    }
    DeviceRuntime::Operation DeviceRuntime::operation(Deadline deadline, CancellationToken cancellation) {
        if (const auto policy = response_policies.find(this); policy != response_policies.end()) {
            deadline = policy->second.deadline;
        }
        auto &p = *impl_;
        std::unique_lock<std::mutex> lock(p.mutex);
        const auto owner = std::this_thread::get_id();
        if (const auto execution = p.command_executions.find(owner); execution != p.command_executions.end()) {
            cancellation = execution->second.cancellation;
        }
        if (p.operation_owner == owner && !p.operation_deadlines.empty()) {
            deadline = std::min(deadline, p.operation_deadlines.back());
        }
        for (const auto &[group_name, group] : p.groups) {
            (void)group_name;
            if (group.owner == owner && !group.deadlines.empty()) {
                deadline = std::min(deadline, group.deadlines.back());
            }
        }
        for (;;) {
            p.require_ready();
            if (const auto policy = response_policies.find(this); policy != response_policies.end() &&
                                                                  policy->second.generation_valid &&
                                                                  !policy->second.generation_valid()) {
                throw ClosedError("read generation ended waiting for an operation");
            }
            if (cancellation.cancelled() || (current_poller == &p && current_poll_cancel.cancelled())) {
                throw LifecycleError("device operation cancelled", p.error_context("runtime.operation"));
            }
            if (Clock::now() >= deadline) {
                throw TimeoutError("device operation deadline expired", p.error_context("runtime.operation"));
            }
            const bool other_requests =
                std::any_of(p.pending.begin(), p.pending.end(), [owner](const auto &r) { return r->owner != owner; });
            const bool other_groups = std::any_of(p.groups.begin(), p.groups.end(),
                                                  [owner](const auto &g) { return g.second.owner != owner; });
            const bool owns_group = std::any_of(p.groups.begin(), p.groups.end(),
                                                [owner](const auto &g) { return g.second.owner == owner; });
            if (owns_group && other_groups) {
                throw LifecycleError("acquire a global operation before competing named composites",
                                     p.error_context("runtime.operation"));
            }
            if ((!p.operation_depth && !other_requests && !other_groups) || p.operation_owner == owner) {
                break;
            }
            p.cv.wait_until(lock, std::min(deadline, Clock::now() + std::chrono::milliseconds(1)));
        }
        if (p.operation_depth) {
            if (p.operation_generation != p.health.device_generation) {
                throw ClosedError("operation belongs to a previous device generation",
                                  p.error_context("runtime.operation"));
            }
            deadline = std::min(deadline, p.operation_deadlines.back());
        } else {
            p.operation_generation = p.health.device_generation;
        }
        p.operation_owner = owner;
        p.operation_deadlines.push_back(deadline);
        ++p.operation_depth;
        return Operation(shared_from_this(), {}, deadline);
    }
    DeviceRuntime::Operation DeviceRuntime::operation(std::string name, Deadline deadline,
                                                      CancellationToken cancellation) {
        if (const auto policy = response_policies.find(this); policy != response_policies.end()) {
            deadline = policy->second.deadline;
        }
        if (name.empty()) {
            return operation(deadline, cancellation);
        }
        auto &p = *impl_;
        std::unique_lock<std::mutex> lock(p.mutex);
        const auto owner = std::this_thread::get_id();
        if (const auto execution = p.command_executions.find(owner); execution != p.command_executions.end()) {
            cancellation = execution->second.cancellation;
        }
        if (p.operation_owner == owner && !p.operation_deadlines.empty()) {
            deadline = std::min(deadline, p.operation_deadlines.back());
        }
        for (const auto &[group_name, group] : p.groups) {
            (void)group_name;
            if (group.owner == owner && !group.deadlines.empty()) {
                deadline = std::min(deadline, group.deadlines.back());
            }
        }
        for (;;) {
            p.require_ready();
            if (const auto policy = response_policies.find(this); policy != response_policies.end() &&
                                                                  policy->second.generation_valid &&
                                                                  !policy->second.generation_valid()) {
                throw ClosedError("read generation ended waiting for an operation");
            }
            if (cancellation.cancelled() || (current_poller == &p && current_poll_cancel.cancelled())) {
                throw LifecycleError("named device operation cancelled", p.error_context("runtime.operation", name));
            }
            if (Clock::now() >= deadline) {
                throw TimeoutError("named device operation deadline expired",
                                   p.error_context("runtime.operation", name));
            }
            auto it = p.groups.find(name);
            if (it != p.groups.end() && it->second.owner != owner &&
                std::any_of(p.groups.begin(), p.groups.end(),
                            [owner](const auto &g) { return g.second.owner == owner; })) {
                throw LifecycleError("competing named composite acquisition would invert ownership",
                                     p.error_context("runtime.operation", name));
            }
            if ((!p.operation_depth || p.operation_owner == owner) &&
                (it == p.groups.end() || it->second.owner == owner)) {
                break;
            }
            p.cv.wait_until(lock, std::min(deadline, Clock::now() + std::chrono::milliseconds(1)));
        }
        auto &group = p.groups[name];
        if (!group.deadlines.empty()) {
            if (group.generation != p.health.device_generation) {
                throw ClosedError("named operation belongs to a previous device generation",
                                  p.error_context("runtime.operation", name));
            }
            deadline = std::min(deadline, group.deadlines.back());
        } else {
            group.generation = p.health.device_generation;
        }
        group.deadlines.push_back(deadline);
        group.owner = owner;
        return Operation(shared_from_this(), std::move(name), deadline);
    }
    void DeviceRuntime::release_operation(const std::string &group) noexcept {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!group.empty()) {
            auto it = impl_->groups.find(group);
            if (it != impl_->groups.end()) {
                it->second.deadlines.pop_back();
                if (it->second.deadlines.empty()) {
                    impl_->groups.erase(it);
                }
            }
            impl_->cv.notify_all();
            return;
        }
        if (impl_->operation_depth) {
            impl_->operation_deadlines.pop_back();
            if (!--impl_->operation_depth) {
                impl_->operation_owner = {};
            }
        }
        impl_->cv.notify_all();
    }
    std::unique_lock<std::timed_mutex> DeviceRuntime::lock_external(std::timed_mutex &gate, Deadline deadline,
                                                                    CancellationToken cancellation) {
        std::unique_lock<std::timed_mutex> lock(gate, std::defer_lock);
        for (;;) {
            {
                std::lock_guard<std::mutex> state_lock(impl_->mutex);
                impl_->require_ready();
            }
            if (cancellation.cancelled() || (current_poller == impl_.get() && current_poll_cancel.cancelled())) {
                throw LifecycleError("external transaction gate cancelled");
            }
            if (Clock::now() >= deadline) {
                throw TimeoutError("external transaction gate deadline expired");
            }
            if (lock.try_lock_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(1)))) {
                return lock;
            }
        }
    }
    void DeviceRuntime::start_polling(std::vector<PollTask> tasks) {
        std::set<std::string> names;
        for (const auto &task : tasks) {
            if (!task.run || task.name.empty() || task.period.count() <= 0 || !names.insert(task.name).second) {
                throw PollingPlanError("polling source must be unique and interval must be positive: " + task.name);
            }
            for (const auto &channel : task.channels) {
                if (channel.name.empty() || channel.period.count() <= 0) {
                    throw PollingPlanError("polling channel name and interval must be valid: " + channel.name);
                }
            }
        }
        auto &p = *impl_;
        assert_external_lifecycle();
        std::lock_guard<std::mutex> operation(p.polling_operation);
        // Replacing a plan stops its scheduling loops but lets already-admitted,
        // bounded transactions finish. Cancelling those transactions here can
        // poison multi-frame correlation keys and make the replacement unusable.
        p.stop_pollers(false);
        {
            std::lock_guard<std::mutex> lock(p.mutex);
            p.require_ready();
            p.polling_cancel = CancellationSource{};
            p.polling_request_cancel = CancellationSource{};
            p.health.polling_faulted = false;
            p.health.polling_error.clear();
            for (const auto &task : tasks) {
                const auto channels =
                    task.channels.empty() ? std::vector<PollTask::Channel>{{task.name, task.period}} : task.channels;
                for (const auto &channel : channels) {
                    auto &state = p.channel_state_locked(channel.name);
                    state.metrics.supported = true;
                    state.polling_rates[task.name] += requested_hz(channel.period);
                    Impl::update_polling_metrics_locked(state);
                }
            }
        }
        try {
            for (auto &task : tasks) {
                auto poller = std::make_unique<Impl::Poller>();
                poller->task = std::move(task);
                auto *ptr = poller.get();
                p.pollers.push_back(std::move(poller));
                auto token = p.polling_cancel.token();
                {
                    std::lock_guard<std::mutex> lock(p.mutex);
                    ++p.active_pollers;
                }
                ptr->thread = std::thread([&p, ptr, token] {
                    struct MarkFinished {
                        Impl &runtime;
                        Impl::Poller &poller;
                        ~MarkFinished() {
                            std::lock_guard<std::mutex> lock(runtime.mutex);
                            poller.finished = true;
                            const auto channels =
                                poller.task.channels.empty()
                                    ? std::vector<PollTask::Channel>{{poller.task.name, poller.task.period}}
                                    : poller.task.channels;
                            for (const auto &channel : channels) {
                                auto &state = runtime.channel_state_locked(channel.name);
                                state.polling_rates.erase(poller.task.name);
                                Impl::update_polling_metrics_locked(state);
                            }
                            if (runtime.active_pollers != 0) {
                                --runtime.active_pollers;
                            }
                            runtime.cv.notify_all();
                        }
                    } finished{p, *ptr};
                    struct PollContext {
                        explicit PollContext(const void *owner) { current_poller = owner; }
                        ~PollContext() {
                            current_poller = nullptr;
                            current_poll_cancel = {};
                        }
                    } context(&p);
                    current_poll_cancel = token;
                    try {
                        auto next = Clock::now();
                        while (!token.cancelled()) {
                            if (ptr->task.next) {
                                uint64_t wake;
                                {
                                    std::lock_guard<std::mutex> lock(p.mutex);
                                    wake = p.polling_wake;
                                }
                                const auto due = ptr->task.next();
                                if (!due) {
                                    break;
                                }
                                std::unique_lock<std::mutex> lock(p.mutex);
                                p.cv.wait_until(lock, *due, [&] {
                                    return token.cancelled() || p.health.state != DeviceState::ready ||
                                           p.polling_wake != wake;
                                });
                                if (token.cancelled() || p.health.state != DeviceState::ready) {
                                    break;
                                }
                                if (p.polling_wake != wake) {
                                    continue;
                                }
                            }
                            unified_detail::RuntimeAccess::PollingGate gate;
                            {
                                std::lock_guard<std::mutex> gate_lock(p.mutex);
                                gate = p.polling_gate;
                            }
                            auto execution_lease = gate ? gate(token) : std::shared_ptr<void>{};
                            if (token.cancelled()) {
                                break;
                            }
                            ptr->task.run(token);
                            execution_lease.reset();
                            if (ptr->task.next) {
                                continue;
                            }
                            next += ptr->task.period;
                            const auto now = Clock::now();
                            if (next < now) {
                                const auto missed = static_cast<uint64_t>((now - next) / ptr->task.period) + 1;
                                {
                                    std::lock_guard<std::mutex> metrics_lock(p.mutex);
                                    p.channel_state_locked(ptr->task.name).metrics.missed_deadlines += missed;
                                }
                                next += missed * ptr->task.period;
                            }
                            std::unique_lock<std::mutex> lock(p.mutex);
                            p.cv.wait_until(lock, next,
                                            [&] { return token.cancelled() || p.health.state != DeviceState::ready; });
                            if (p.health.state != DeviceState::ready) {
                                break;
                            }
                        }
                    } catch (const CancelledError &) {
                        if (!token.cancelled()) {
                            std::lock_guard<std::mutex> lock(p.mutex);
                            p.health.polling_faulted = true;
                            p.health.polling_error = "polling request cancelled";
                            p.record_error_locked(std::current_exception(),
                                                  p.error_context("runtime.poll", ptr->task.name),
                                                  ErrorCode::kLifecycle);
                            if (ptr->task.isolate_failure) {
                                p.cv.notify_all();
                                return;
                            }
                            p.polling_cancel.cancel();
                            p.polling_request_cancel.cancel();
                            p.cv.notify_all();
                        }
                    } catch (const std::exception &error) {
                        std::lock_guard<std::mutex> lock(p.mutex);
                        if (token.cancelled() || p.health.state == DeviceState::closing ||
                            p.health.state == DeviceState::closed || p.health.state == DeviceState::recovering) {
                            return;
                        }
                        p.health.polling_faulted = true;
                        p.health.polling_error = error.what();
                        p.record_error_locked(std::current_exception(), p.error_context("runtime.poll", ptr->task.name),
                                              ErrorCode::kInternal);
                        if (ptr->task.isolate_failure) {
                            p.cv.notify_all();
                            return;
                        }
                        p.health.state = DeviceState::faulted;
                        p.polling_cancel.cancel();
                        p.polling_request_cancel.cancel();
                        p.invalidate_locked();
                        const auto pending = p.pending;
                        for (const auto &state : pending) {
                            p.finish_locked(state, std::current_exception(), true);
                        }
                        p.cv.notify_all();
                    } catch (...) {
                        std::lock_guard<std::mutex> lock(p.mutex);
                        if (token.cancelled() || p.health.state == DeviceState::closing ||
                            p.health.state == DeviceState::closed || p.health.state == DeviceState::recovering) {
                            return;
                        }
                        p.health.polling_faulted = true;
                        p.health.polling_error = "non-standard polling failure";
                        p.record_error_locked(std::current_exception(), p.error_context("runtime.poll", ptr->task.name),
                                              ErrorCode::kInternal);
                        if (ptr->task.isolate_failure) {
                            p.cv.notify_all();
                            return;
                        }
                        p.health.state = DeviceState::faulted;
                        p.polling_cancel.cancel();
                        p.polling_request_cancel.cancel();
                        p.invalidate_locked();
                        const auto pending = p.pending;
                        for (const auto &state : pending) {
                            p.finish_locked(state, std::current_exception(), true);
                        }
                        p.cv.notify_all();
                    }
                });
            }
        } catch (...) {
            const auto error = std::current_exception();
            p.stop_pollers();
            ErrorContext context;
            {
                std::lock_guard<std::mutex> lock(p.mutex);
                context = p.error_context("runtime.start_polling");
            }
            rethrow_as_hand_error(error, std::move(context));
        }
    }
    void DeviceRuntime::stop_polling() {
        auto &p = *impl_;
        assert_external_lifecycle();
        std::lock_guard<std::mutex> operation(p.polling_operation);
        p.stop_pollers();
    }
    bool DeviceRuntime::wait_polling_stopped(Deadline deadline) {
        auto &p = *impl_;
        assert_external_lifecycle();
        std::lock_guard<std::mutex> operation(p.polling_operation);
        std::unique_lock<std::mutex> lock(p.mutex);
        const auto all_finished = [&] {
            return std::all_of(p.pollers.begin(), p.pollers.end(), [](const auto &poller) { return poller->finished; });
        };
        if (!wait_for_state(p.cv, lock, deadline, all_finished)) {
            return false;
        }
        lock.unlock();
        for (auto &poller : p.pollers) {
            if (poller->thread.joinable()) {
                poller->thread.join();
            }
        }
        p.pollers.clear();
        return true;
    }
    void DeviceRuntime::wake_polling() {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        ++impl_->polling_wake;
        impl_->cv.notify_all();
    }
    void DeviceRuntime::wait_until(Deadline deadline, CancellationToken cancellation) const {
        auto &p = *impl_;
        std::unique_lock<std::mutex> lock(p.mutex);
        for (;;) {
            p.require_ready();
            if (const auto policy = response_policies.find(this); policy != response_policies.end() &&
                                                                  policy->second.generation_valid &&
                                                                  !policy->second.generation_valid()) {
                throw ClosedError("read generation ended waiting for an operation");
            }
            if (cancellation.cancelled() || (current_poller == &p && current_poll_cancel.cancelled())) {
                throw LifecycleError("runtime wait cancelled", p.error_context("runtime.wait"));
            }
            if (Clock::now() >= deadline) {
                return;
            }
            p.cv.wait_until(lock, std::min(deadline, Clock::now() + std::chrono::milliseconds(1)));
        }
    }
    uint64_t DeviceRuntime::sent_frames(const std::string &domain) const {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->reap_sends_locked();
        const auto it = impl_->sent_by_domain.find(domain);
        return it == impl_->sent_by_domain.end() ? 0 : it->second;
    }
    size_t DeviceRuntime::pending_count(const std::string &domain) const {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        return static_cast<size_t>(std::count_if(impl_->pending.begin(), impl_->pending.end(),
                                                 [&](const auto &request) { return request->plan.domain == domain; }));
    }
    bool DeviceRuntime::wait_for_fault(Deadline deadline) const {
        std::unique_lock<std::mutex> lock(impl_->mutex);
        wait_for_state(impl_->cv, lock, deadline, [&] {
            return impl_->health.polling_faulted || impl_->health.state == DeviceState::faulted ||
                   impl_->health.state == DeviceState::closed;
        });
        return impl_->health.polling_faulted || impl_->health.state == DeviceState::faulted;
    }
    RuntimeHealth DeviceRuntime::runtime_health() const {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->reap_sends_locked();
        auto h = impl_->health;
        for (const auto &[id, subscriber] : impl_->subscribers) {
            (void)id;
            std::lock_guard<std::mutex> slot_lock(subscriber.slot->mutex);
            if (!subscriber.slot->error.empty()) {
                h.last_error = subscriber.slot->error;
            }
        }
        return h;
    }
    HandHealth DeviceRuntime::health() const {
        struct ChannelSnapshotInput {
            ChannelMetrics metrics;
            std::optional<HealthTimePoint> last_sample;
            std::vector<HealthDuration> latencies;
            std::vector<HealthTimePoint> sample_times;
        };
        RuntimeHealth runtime;
        bool polling_active = false;
        std::vector<HealthDuration> latencies;
        std::vector<ChannelSnapshotInput> channel_inputs;
        std::vector<CommandFamilyMetrics> command_families;
        std::optional<ErrorInfo> recent_error;
        uint64_t consecutive_errors = 0;
        uint64_t reconnect_count = 0;
        std::optional<HealthTimePoint> last_successful_send;
        std::optional<HealthTimePoint> last_successful_receive;
        Deadline last_endpoint_receive{};
        std::optional<HealthTimePoint> last_successful_transaction;
        const auto captured_at = HealthClock::now();
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->reap_sends_locked();
            runtime = impl_->health;
            polling_active = impl_->active_pollers != 0;
            recent_error = impl_->recent_error;
            consecutive_errors = impl_->consecutive_errors;
            reconnect_count = impl_->reconnect_count;
            last_successful_send = impl_->last_successful_send;
            last_successful_receive = impl_->last_successful_receive;
            last_successful_transaction = impl_->last_successful_transaction;
            latencies.assign(impl_->transaction_latencies.begin(), impl_->transaction_latencies.end());
            channel_inputs.reserve(impl_->channel_states.size());
            for (const auto &[domain, state] : impl_->channel_states) {
                ChannelSnapshotInput input;
                input.metrics = state.metrics;
                input.metrics.data_age.age.reset();
                input.last_sample = state.last_sample;
                input.latencies.assign(state.latencies.begin(), state.latencies.end());
                input.sample_times.assign(state.sample_times.begin(), state.sample_times.end());
                const auto sample = impl_->samples.find(domain);
                if (sample != impl_->samples.end() &&
                    (sample->second.sequence >= input.metrics.data_age.sequence || !sample->second.valid)) {
                    input.metrics.data_age.valid = sample->second.valid;
                    input.metrics.data_age.stale = !sample->second.valid;
                    input.metrics.data_age.sequence = sample->second.sequence;
                    input.metrics.data_age.generation = sample->second.device_generation;
                    input.metrics.feedback_samples = std::max(input.metrics.feedback_samples, sample->second.sequence);
                    if (sample->second.received_at != Deadline{}) {
                        input.last_sample = sample->second.received_at;
                    }
                }
                channel_inputs.push_back(std::move(input));
            }
            command_families.reserve(impl_->command_family_states.size());
            for (const auto &[family, state] : impl_->command_family_states) {
                (void)family;
                command_families.push_back(state.metrics);
            }
            for (const auto &[id, subscriber] : impl_->subscribers) {
                (void)id;
                std::lock_guard<std::mutex> slot_lock(subscriber.slot->mutex);
                if (!subscriber.slot->error.empty()) {
                    runtime.last_error = subscriber.slot->error;
                }
            }
        }

        std::vector<ChannelMetrics> channels;
        channels.reserve(channel_inputs.size());
        for (auto &input : channel_inputs) {
            if (input.metrics.data_age.valid && input.last_sample && captured_at >= *input.last_sample) {
                input.metrics.data_age.age = captured_at - *input.last_sample;
            }
            input.metrics.latency = latency_metrics(std::move(input.latencies));
            if (input.sample_times.size() >= 2 && input.sample_times.back() > input.sample_times.front()) {
                const auto seconds =
                    std::chrono::duration<double>(input.sample_times.back() - input.sample_times.front()).count();
                input.metrics.achieved_hz = static_cast<double>(input.sample_times.size() - 1) / seconds;
            } else {
                input.metrics.achieved_hz = 0.0;
            }
            channels.push_back(std::move(input.metrics));
        }

        HandHealth snapshot;
        snapshot.lifecycle_state = lifecycle_state(runtime.state);
        snapshot.connection_state = connection_state(runtime.state);
        snapshot.identity_verified = runtime.state == DeviceState::ready;
        snapshot.polling_active = polling_active;
        snapshot.connection_generation = runtime.device_generation;
        snapshot.reconnect_count = reconnect_count;
        snapshot.consecutive_errors = consecutive_errors;
        snapshot.recent_error = std::move(recent_error);
        snapshot.last_successful_send = last_successful_send;
        snapshot.last_successful_receive = last_successful_receive;
        snapshot.last_successful_transaction = last_successful_transaction;
        snapshot.channels = std::move(channels);
        snapshot.command_families = std::move(command_families);
        snapshot.transaction_latency = latency_metrics(std::move(latencies));
        snapshot.transport = impl_->session->transport_health();
        snapshot.session = impl_->session->health();
        return snapshot;
    }
    void DeviceRuntime::register_channel(std::string channel, double requested_hz, bool supported) {
        if (channel.empty()) {
            throw ValidationError("health channel name is empty");
        }
        if (!std::isfinite(requested_hz) || requested_hz < 0.0) {
            throw ValidationError("health channel requested Hz must be finite and non-negative");
        }
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto &state = impl_->channel_state_locked(channel);
        state.registered_requested_hz = requested_hz;
        state.metrics.supported = supported;
        Impl::update_polling_metrics_locked(state);
    }
    void DeviceRuntime::register_command_family(std::string family) {
        if (family.empty()) {
            throw ValidationError("command family name is empty");
        }
        family = canonical_command_family(family);
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->canonical_command_family_state_locked(family);
    }
    void DeviceRuntime::record_channel_success(std::string channel, HealthTimePoint received_at,
                                               std::optional<HealthDuration> latency, uint64_t feedback_samples) {
        if (channel.empty()) {
            throw ValidationError("health channel name is empty");
        }
        if (latency && *latency < HealthDuration::zero()) {
            throw ValidationError("health channel latency cannot be negative");
        }
        channel = canonical_channel(channel);
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->record_channel_success_locked(impl_->canonical_channel_state_locked(channel), received_at, latency,
                                             feedback_samples);
    }
    void DeviceRuntime::record_channel_timeout(std::string channel) {
        if (channel.empty()) {
            throw ValidationError("health channel name is empty");
        }
        channel = canonical_channel(channel);
        std::lock_guard<std::mutex> lock(impl_->mutex);
        ++impl_->canonical_channel_state_locked(channel).metrics.timeouts;
    }
    void DeviceRuntime::record_deadline_miss(std::string channel, uint64_t count) {
        if (channel.empty()) {
            throw ValidationError("health channel name is empty");
        }
        channel = canonical_channel(channel);
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->canonical_channel_state_locked(channel).metrics.missed_deadlines += count;
    }
    void DeviceRuntime::record_command_accepted(std::string family) {
        if (family.empty()) {
            throw ValidationError("command family name is empty");
        }
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto &state = impl_->command_family_state_locked(family);
        ++state.metrics.accepted;
        ++state.metrics.pending;
        ++state.metrics.queue.accepted;
        impl_->update_command_depth_locked(state);
    }
    void DeviceRuntime::record_command_dispatched(std::string family) {
        if (family.empty()) {
            throw ValidationError("command family name is empty");
        }
        family = canonical_command_family(family);
        std::lock_guard<std::mutex> lock(impl_->mutex);
        ++impl_->canonical_command_family_state_locked(family).metrics.dispatched;
    }
    void DeviceRuntime::record_command_acknowledged(std::string family) {
        if (family.empty()) {
            throw ValidationError("command family name is empty");
        }
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto &state = impl_->command_family_state_locked(family);
        ++state.metrics.acknowledged;
        impl_->complete_command_locked(state);
    }
    void DeviceRuntime::record_command_failed(std::string family, std::optional<ErrorCode> reason) {
        if (family.empty()) {
            throw ValidationError("command family name is empty");
        }
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto &state = impl_->command_family_state_locked(family);
        ++state.metrics.failed;
        if (reason == ErrorCode::kTimeout) {
            ++state.metrics.timed_out;
        }
        if (reason == ErrorCode::kLifecycle || reason == ErrorCode::kClosed) {
            ++state.metrics.cancelled;
        }
        impl_->complete_command_locked(state);
    }
    void DeviceRuntime::record_command_superseded(std::string family) {
        if (family.empty()) {
            throw ValidationError("command family name is empty");
        }
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto &state = impl_->command_family_state_locked(family);
        ++state.metrics.superseded;
        ++state.metrics.queue.dropped;
        impl_->complete_command_locked(state);
    }
    void DeviceRuntime::record_error(std::exception_ptr error, ErrorContext context, ErrorCode fallback) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (context.operation.empty()) {
            context.operation = "runtime.record_error";
        }
        if (context.model.empty()) {
            context.model = impl_->identity;
        }
        if (context.generation == 0) {
            context.generation = impl_->health.device_generation;
        }
        (void)impl_->record_error_locked(std::move(error), std::move(context), fallback);
    }
    std::shared_ptr<transport::Session> DeviceRuntime::session() const { return impl_->session; }
    void DeviceRuntime::set_routes(std::vector<transport::RouteKey> keys) {
        auto &p = *impl_;
        std::lock_guard<std::mutex> lock(p.mutex);
        p.require_ready();
        try {
            p.session->update_routes(p.inbox, keys);
        } catch (...) {
            std::rethrow_exception(p.record_error_locked(std::current_exception(),
                                                         p.error_context("runtime.set_routes"), ErrorCode::kTransport));
        }
        p.routes = std::move(keys);
    }
    void DeviceRuntime::cancel_domain(const std::string &domain, bool preserve_snapshot) {
        auto &p = *impl_;
        std::lock_guard<std::mutex> lock(p.mutex);
        const auto pending = p.pending;
        for (const auto &state : pending) {
            if (state->plan.domain == domain) {
                p.finish_locked(state, std::make_exception_ptr(CancelledError("domain cancelled")), true);
            }
        }
        if (!preserve_snapshot) {
            auto &channel = p.channel_state_locked(domain);
            channel.metrics.data_age.valid = false;
            channel.metrics.data_age.stale = true;
            p.samples[domain].valid = false;
        }
        p.cv.notify_all();
    }
    void DeviceRuntime::cancel_all() {
        auto &p = *impl_;
        std::lock_guard<std::mutex> lock(p.mutex);
        if (p.python_endpoint_policy && p.health.state == DeviceState::closed) {
            return;
        }
        const auto pending = p.pending;
        for (const auto &state : pending) {
            p.finish_locked(state, std::make_exception_ptr(CancelledError("device requests cancelled")), true);
        }
        if (p.python_endpoint_policy) {
            (void)p.stop_profile_endpoint_locked();
        } else {
            p.session->cancel_endpoint(p.identity);
        }
        p.cv.notify_all();
    }
    bool DeviceRuntime::is_event_thread() const {
        return current_event_slot && current_event_slot->owner == impl_.get();
    }
    void DeviceRuntime::assert_external_lifecycle() const {
        const auto *p = impl_.get();
        if (can::CANDispatcher::is_any_receive_thread() || current_poller == p || current_receiver == p ||
            (current_event_slot && current_event_slot->owner == p)) {
            throw LifecycleError("runtime lifecycle operation requires an external caller");
        }
    }
    void DeviceRuntime::quiesce_for_destruction() {
        auto &p = *impl_;
        if (current_poller == &p || current_receiver == &p) {
            throw LifecycleError("facade destruction cannot run on its polling/decoder worker");
        }
        {
            std::lock_guard<std::mutex> lock(p.mutex);
            if (p.health.state != DeviceState::closed) {
                p.health.state = DeviceState::closing;
            }
            p.destruction_requested = true;
            p.polling_cancel.cancel();
            p.rx_stop = true;
            if (p.inbox) {
                p.inbox->close();
            }
            p.invalidate_locked();
            for (const auto &[id, subscriber] : p.subscribers) {
                (void)id;
                std::lock_guard<std::mutex> gate(subscriber.slot->mutex);
                subscriber.slot->active = false;
            }
            p.cv.notify_all();
        }
        cancel_all();
        {
            std::lock_guard<std::mutex> gate(p.polling_operation);
            p.stop_pollers();
        }
        p.drain_sends();
    }
    void DeviceRuntime::close() {
        assert_external_lifecycle();
        close_impl();
    }
    void DeviceRuntime::close_impl() {
        auto &p = *impl_;
        std::lock_guard<std::mutex> lifecycle(p.lifecycle);
        {
            std::lock_guard<std::mutex> lock(p.mutex);
            if (p.health.state == DeviceState::closed) {
                return;
            }
            p.health.state = DeviceState::closing;
            p.polling_cancel.cancel();
            p.cv.notify_all();
        }
        cancel_all();
        {
            std::lock_guard<std::mutex> gate(p.polling_operation);
            p.stop_pollers();
        }
        p.drain_sends();
        {
            std::lock_guard<std::mutex> lock(p.mutex);
            p.rx_stop = true;
            if (p.inbox) {
                p.inbox->close();
            }
        }
        if (p.rx.joinable()) {
            p.rx.join();
        }
        std::vector<std::shared_ptr<EventSlot>> slots;
        {
            std::lock_guard<std::mutex> lock(p.mutex);
            for (auto &[id, subscriber] : p.subscribers) {
                p.cancelling_subscribers.emplace(id, subscriber.slot);
            }
            for (auto &[id, slot] : p.cancelling_subscribers) {
                (void)id;
                slots.push_back(slot);
            }
            p.subscribers.clear();
            p.observers.clear();
            p.invalidate_locked();
            p.health.state = DeviceState::closed;
            p.cv.notify_all();
        }
        for (const auto &slot : slots) {
            deactivate(slot);
        }
        {
            std::lock_guard<std::mutex> lock(p.mutex);
            p.cancelling_subscribers.clear();
        }
    }
    void DeviceRuntime::recover() {
        auto &p = *impl_;
        assert_external_lifecycle();
        std::lock_guard<std::mutex> lifecycle(p.lifecycle);
        uint64_t generation;
        std::vector<std::shared_ptr<EventSlot>> slots;
        {
            std::lock_guard<std::mutex> lock(p.mutex);
            if (p.health.state == DeviceState::closed || p.health.state == DeviceState::closing) {
                throw ClosedError("closed device cannot recover", p.error_context("runtime.recover"));
            }
            for (const auto &[id, subscriber] : p.subscribers) {
                (void)id;
                slots.push_back(subscriber.slot);
            }
            for (const auto &[id, slot] : p.cancelling_subscribers) {
                (void)id;
                slots.push_back(slot);
            }
            p.health.state = DeviceState::recovering;
            for (const auto &slot : slots) {
                std::lock_guard<std::mutex> gate(slot->mutex);
                slot->paused = true;
            }
            generation = p.health.physical_generation;
            p.polling_cancel.cancel();
            p.cv.notify_all();
        }
        cancel_all();
        stop_polling();
        p.drain_sends();
        {
            std::lock_guard<std::mutex> lock(p.mutex);
            p.rx_stop = true;
            p.inbox->close();
            p.invalidate_locked();
        }
        if (p.rx.joinable()) {
            p.rx.join();
        }
        for (const auto &slot : slots) {
            std::unique_lock<std::mutex> gate(slot->mutex);
            slot->cv.wait(gate, [&] { return !slot->executing; });
        }
        try {
            const auto next = p.session->recover(generation);
            auto inbox = p.session->subscribe_routes(p.routes, p.inbox_capacity);
            {
                std::lock_guard<std::mutex> lock(p.mutex);
                if (p.destruction_requested) {
                    throw ClosedError("facade destruction interrupted recovery", p.error_context("runtime.recover"));
                }
                p.inbox = std::move(inbox);
                p.health.physical_generation = next;
                ++p.health.device_generation;
                ++p.reconnect_count;
                p.reset_observers_locked();
                for (const auto &slot : slots) {
                    std::lock_guard<std::mutex> gate(slot->mutex);
                    slot->physical_generation = next;
                    slot->device_generation = p.health.device_generation;
                    slot->paused = false;
                }
                p.poisoned.clear();
                p.quarantine.clear();
                if (p.python_endpoint_policy) {
                    p.session->resume_endpoint(p.identity);
                }
                p.rx_stop = false;
                p.health.state = DeviceState::ready;
            }
            p.rx = std::thread([&p] { p.run(); });
        } catch (...) {
            const auto error = std::current_exception();
            std::exception_ptr classified;
            {
                std::lock_guard<std::mutex> lock(p.mutex);
                p.health.state = DeviceState::faulted;
                classified = p.record_error_locked(error, p.error_context("runtime.recover"), ErrorCode::kTransport);
            }
            std::rethrow_exception(classified);
        }
    }
} // namespace linkerbot::hand::runtime

void linkerbot::hand::unified_detail::RuntimeAccess::set_polling_gate(runtime::DeviceRuntime &runtime,
                                                                      PollingGate gate) {
    std::lock_guard<std::mutex> lock(runtime.impl_->mutex);
    runtime.impl_->polling_gate = std::move(gate);
}

void linkerbot::hand::unified_detail::RuntimeAccess::execute_command(
    runtime::DeviceRuntime &runtime, runtime::Deadline deadline, bool send_only, SendObserver observer,
    FinalSend final_send, runtime::CancellationToken cancellation, const std::function<void()> &execute,
    std::function<void()> prepare_send, std::function<void()> accepted_send, ReceiptAccepted receipt_accepted,
    AdmitSend admit_send) {
    struct SendCallbacks {
        SendObserver &observer;
        FinalSend &final_send;
        const std::function<void()> &execute;
        std::function<void()> &prepare_send;
        std::function<void()> &accepted_send;
    };
    const auto callbacks = SendCallbacks{observer, final_send, execute, prepare_send, accepted_send};
    const auto owner = std::this_thread::get_id();
    auto &p = *runtime.impl_;
    std::optional<runtime::DeviceRuntime::Impl::CommandExecution> previous;
    {
        const std::lock_guard<std::mutex> lock(p.mutex);
        if (const auto it = p.command_executions.find(owner); it != p.command_executions.end()) {
            previous = it->second;
        }
        p.command_executions.insert_or_assign(
            owner, runtime::DeviceRuntime::Impl::CommandExecution{
                       send_only, deadline, cancellation, std::move(callbacks.observer),
                       std::move(callbacks.final_send), std::move(callbacks.prepare_send),
                       std::move(callbacks.accepted_send), std::move(receipt_accepted), std::move(admit_send)});
    }
    const auto restore = [&] {
        const std::lock_guard<std::mutex> lock(p.mutex);
        if (previous) {
            p.command_executions.insert_or_assign(owner, std::move(*previous));
        } else {
            p.command_executions.erase(owner);
        }
    };
    try {
        auto operation = runtime.operation(deadline, cancellation);
        callbacks.execute();
    } catch (...) {
        restore();
        throw;
    }
    restore();
}

void linkerbot::hand::unified_detail::RuntimeAccess::with_response_windows(
    runtime::DeviceRuntime &runtime, PlanTransform transform, const std::function<void()> &execute,
    runtime::Deadline execution_deadline, std::function<bool()> generation_valid, ErrorCapture error_capture,
    TimingCapture timing) {
    {
        const std::lock_guard<std::mutex> lock(runtime.impl_->mutex);
        runtime.impl_->python_endpoint_policy = true;
    }
    auto previous = runtime::response_policies.extract(&runtime);
    try {
        runtime::response_policies.emplace(
            &runtime, runtime::ResponsePolicy{std::move(transform), execution_deadline, std::move(generation_valid),
                                              std::move(error_capture), std::move(timing)});
    } catch (...) {
        if (!previous.empty()) {
            runtime::response_policies.insert(std::move(previous));
        }
        throw;
    }
    const auto restore = [&] {
        runtime::response_policies.erase(&runtime);
        if (!previous.empty()) {
            runtime::response_policies.insert(std::move(previous));
        }
    };
    try {
        execute();
    } catch (...) {
        restore();
        throw;
    }
    restore();
}

void linkerbot::hand::unified_detail::RuntimeAccess::wait_endpoint_quiet(runtime::DeviceRuntime &runtime,
                                                                         std::chrono::milliseconds quiet,
                                                                         runtime::Deadline deadline,
                                                                         const std::function<bool()> &cancelled) {
    auto &p = *runtime.impl_;
    std::unique_lock<std::mutex> lock(p.mutex);
    auto last = runtime::Clock::now();
    while (true) {
        last = std::max(last, p.last_endpoint_receive);
        if (cancelled()) {
            throw ClosedError("recovery ended during quiet period");
        }
        if (runtime::Clock::now() >= deadline) {
            throw TimeoutError("recovery execution deadline expired during quiet period");
        }
        if (runtime::Clock::now() - last >= quiet) {
            return;
        }
        p.cv.wait_until(lock, std::min({deadline, last + quiet, runtime::Clock::now() + std::chrono::milliseconds{1}}));
    }
}

void linkerbot::hand::unified_detail::RuntimeAccess::flush_sends(runtime::DeviceRuntime &runtime) {
    const std::lock_guard<std::mutex> lock(runtime.impl_->mutex);
    runtime.impl_->reap_sends_locked();
}

void linkerbot::hand::unified_detail::RuntimeAccess::drain_sends(runtime::DeviceRuntime &runtime,
                                                                 runtime::Deadline deadline) {
    auto &p = *runtime.impl_;
    std::unique_lock<std::mutex> lock(p.mutex);
    while (true) {
        p.reap_sends_locked();
        if (p.send_audit.empty()) {
            return;
        }
        if (runtime::Clock::now() >= deadline) {
            throw TimeoutError("recovery preparation did not drain sends before its deadline");
        }
        p.cv.wait_until(lock, std::min(deadline, runtime::Clock::now() + std::chrono::milliseconds{1}));
    }
}

bool linkerbot::hand::unified_detail::RuntimeAccess::uses_response_windows(const runtime::DeviceRuntime &runtime) {
    return runtime::response_policies.count(&runtime) != 0;
}

void linkerbot::hand::unified_detail::RuntimeAccess::resume_endpoint(runtime::DeviceRuntime &runtime) {
    const std::lock_guard<std::mutex> lock(runtime.impl_->mutex);
    runtime.impl_->session->resume_endpoint(runtime.impl_->identity);
    runtime.impl_->python_endpoint_policy = true;
}

void linkerbot::hand::unified_detail::RuntimeAccess::stop_endpoint(runtime::DeviceRuntime &runtime) {
    const std::lock_guard<std::mutex> lock(runtime.impl_->mutex);
    (void)runtime.impl_->stop_profile_endpoint_locked();
}

void linkerbot::hand::unified_detail::RuntimeAccess::retire_closed_profiles(
    const std::shared_ptr<transport::Session> &session) {
    const std::lock_guard<std::mutex> registry(runtime::runtimes_mutex);
    for (auto it = runtime::runtimes.begin(); it != runtime::runtimes.end();) {
        bool closed_profile = false;
        if (it->first.first == session.get()) {
            if (auto existing = it->second.lock()) {
                const std::lock_guard<std::mutex> lock(existing->impl_->mutex);
                closed_profile = existing->impl_->python_endpoint_policy &&
                                 existing->impl_->health.state == runtime::DeviceState::closed;
            }
        }
        if (closed_profile) {
            it = runtime::runtimes.erase(it);
        } else {
            ++it;
        }
    }
}

void linkerbot::hand::unified_detail::RuntimeAccess::set_publication_handler(runtime::DeviceRuntime &device,
                                                                             PublicationHandler handler) {
    std::lock_guard<std::mutex> lock(device.impl_->mutex);
    device.impl_->publication_handler = std::move(handler);
}
void linkerbot::hand::unified_detail::RuntimeAccess::with_snapshot_view(
    const runtime::DeviceRuntime &device, const std::map<std::string, runtime::Sample> &samples,
    const std::function<void()> &read) {
    struct Restore {
        std::optional<runtime::SnapshotView> previous;
        ~Restore() { runtime::current_snapshot_view = previous; }
    } restore{runtime::current_snapshot_view};
    runtime::current_snapshot_view = runtime::SnapshotView{&device, &samples};
    read();
}

void linkerbot::hand::unified_detail::RuntimeAccess::wait_command_feedback(
    runtime::DeviceRuntime &device, const std::function<bool()> &generation_valid) {
    auto &p = *device.impl_;
    std::unique_lock<std::mutex> lock(p.mutex);
    p.cv.wait(lock, [&] {
        return p.health.state != runtime::DeviceState::ready || (generation_valid && !generation_valid()) ||
               std::none_of(p.pending.begin(), p.pending.end(),
                            [](const auto &request) { return request->plan.optional_response_window.has_value(); });
    });
}

linkerbot::hand::unified_detail::RuntimeAccess::InboxScope::InboxScope(size_t capacity)
    : previous_(runtime::construction_inbox_capacity) {
    if (capacity == 0) {
        throw ValidationError("inbox capacity must be positive");
    }
    runtime::construction_inbox_capacity = capacity;
}
linkerbot::hand::unified_detail::RuntimeAccess::InboxScope::~InboxScope() {
    runtime::construction_inbox_capacity = previous_;
}
size_t linkerbot::hand::unified_detail::RuntimeAccess::inbox_capacity(const runtime::DeviceRuntime &device) {
    const std::lock_guard<std::mutex> lock(device.impl_->mutex);
    return device.impl_->inbox_capacity;
}
uint64_t linkerbot::hand::unified_detail::RuntimeAccess::error_revision(const runtime::DeviceRuntime &device) {
    const std::lock_guard<std::mutex> lock(device.impl_->mutex);
    return device.impl_->error_revision;
}
void linkerbot::hand::unified_detail::RuntimeAccess::promote_error(runtime::DeviceRuntime &device, uint64_t revision,
                                                                   const HandError &error,
                                                                   std::optional<uint64_t> unrecorded_revision) {
    const std::lock_guard<std::mutex> lock(device.impl_->mutex);
    // A later observer/transport error must not be replaced by an older query's downgrade.
    if (revision != 0 && revision == device.impl_->error_revision) {
        auto info = runtime::error_info(error);
        device.impl_->recent_error = std::move(info);
        device.impl_->health.last_error = error.what();
    } else if (revision == 0 && unrecorded_revision &&
               unrecorded_revision.value_or(0) == device.impl_->error_revision) {
        // Some protocol adapters reject a decoded value after a successful native
        // read. That logical failure has not yet been counted by the runtime.
        (void)device.impl_->record_error_locked(
            std::make_exception_ptr(UnsupportedFirmwareError(error.what(), error.context(), error.cause())),
            device.impl_->error_context("runtime.read", error.context().channel));
    }
}

void linkerbot::hand::unified_detail::RuntimeAccess::initialized_health(runtime::DeviceRuntime &device) {
    const std::lock_guard<std::mutex> lock(device.impl_->mutex);
    if (device.impl_->python_endpoint_policy &&
        device.impl_->last_success_error_revision == device.impl_->error_revision) {
        device.impl_->recent_error.reset();
        device.impl_->health.last_error.clear();
        device.impl_->consecutive_errors = 0;
        ++device.impl_->error_revision;
        device.impl_->last_success_error_revision = device.impl_->error_revision;
    }
}

void linkerbot::hand::unified_detail::RuntimeAccess::logical_response_completed(runtime::DeviceRuntime &device) {
    const auto policy = runtime::response_policies.find(&device);
    if (policy != runtime::response_policies.end() && policy->second.timing) {
        policy->second.timing->completed = runtime::Clock::now();
    }
}
void linkerbot::hand::unified_detail::RuntimeAccess::logical_transaction_succeeded(runtime::DeviceRuntime &device) {
    const std::lock_guard<std::mutex> lock(device.impl_->mutex);
    if (device.impl_->last_success_error_revision == device.impl_->error_revision) {
        device.impl_->consecutive_errors = 0;
    }
}
