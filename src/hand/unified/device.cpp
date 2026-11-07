#include "../detail/checked_optional.hpp"
#include <linkerbot/hand/api/device.hpp>

#include "detail/backend.hpp"
#include "detail/cached_snapshot.hpp"
#include "detail/command_scheduler.hpp"
#include "detail/common.hpp"
#include "detail/connection_initializer.hpp"
#include "detail/event_stream.hpp"
#include "detail/force.hpp"
#include "detail/freshness.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <future>
#include <map>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>
#include <variant>

#include <linkerbot/hand/conversion.hpp>
#include <linkerbot/hand/errors.hpp>

namespace linkerbot::hand {
    namespace {
        // Deferred destruction owns every resource until uninterruptible driver
        // calls return. No thread is detached and no worker borrows a dead facade.
        // Process teardown also drains this queue; it requires bounded drivers.
        class CleanupQueue {
        public:
            CleanupQueue() : worker_([this] { run(); }) {}
            ~CleanupQueue() {
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    changed_.wait(lock, [this] { return lifecycles_ == 0; });
                    stopping_ = true;
                }
                changed_.notify_all();
                worker_.join();
            }
            void add(std::function<void()> cleanup) {
                {
                    const std::lock_guard<std::mutex> lock(mutex_);
                    jobs_.push_back(std::move(cleanup));
                }
                changed_.notify_one();
            }
            void retain_lifecycle() {
                const std::lock_guard<std::mutex> lock(mutex_);
                ++lifecycles_;
            }
            void release_lifecycle() {
                {
                    const std::lock_guard<std::mutex> lock(mutex_);
                    --lifecycles_;
                }
                changed_.notify_all();
            }
            static CleanupQueue &instance() {
                static CleanupQueue queue;
                return queue;
            }

        private:
            void run() {
                std::unique_lock<std::mutex> lock(mutex_);
                for (;;) {
                    changed_.wait(lock, [this] { return stopping_ || !jobs_.empty(); });
                    if (jobs_.empty()) {
                        return;
                    }
                    auto job = std::move(jobs_.front());
                    jobs_.pop_front();
                    lock.unlock();
                    job();
                    lock.lock();
                }
            }
            std::mutex mutex_;
            std::condition_variable changed_;
            std::deque<std::function<void()>> jobs_;
            bool stopping_ = false;
            size_t lifecycles_ = 0;
            std::thread worker_;
        };
    }

    class HandDevice::Impl {
    public:
        enum class State : uint8_t {
            kClosed,
            kOpening,
            kReady,
            kClosing,
            kRecovering,
            kFaulted,
        };

        Impl(HandDeviceConfig config, TransportFactory transport_factory,
             std::shared_ptr<transport::Session> shared_session = {},
             std::optional<DeviceIdentity> expected_identity = {})
            : config_(std::move(config)), transport_factory_(std::move(transport_factory)),
              shared_session_(std::move(shared_session)), expected_identity_(std::move(expected_identity)) {
            unified_detail::validate_config(config_);
            declared_capabilities_ = declared_capabilities(config_);
            cached_snapshot_ = unified_detail::initial_cached_snapshot(declared_capabilities_);
            scheduler_ = std::make_unique<unified_detail::CommandScheduler>(config_.command_rate_hz,
                                                                            config_.discrete_queue_capacity);
        }

        template <typename... Args> static std::shared_ptr<Impl> create(Args &&...args) {
            auto &cleanup = CleanupQueue::instance();
            return std::shared_ptr<Impl>(new Impl(std::forward<Args>(args)...), [&cleanup](Impl *owned) {
                bool defer = owned->state_.load(std::memory_order_acquire) != State::kClosed ||
                             (owned->lifecycle_worker_.joinable() &&
                              owned->lifecycle_worker_.get_id() == std::this_thread::get_id());
                try {
                    owned->assert_external_lifecycle();
                } catch (...) {
                    defer = true;
                }
                if (defer) {
                    cleanup.add([owned] { delete owned; });
                } else {
                    delete owned;
                }
            });
        }

        ~Impl() {
            // Always called by the external cleanup owner, including when the
            // last reference was released by a lifecycle worker or callback.
            if (lifecycle_worker_.joinable()) {
                lifecycle_worker_.join();
            }
            invalidate_commands("destruction");
            stop_stream();
            if (backend_) {
                try {
                    backend_->cancel_commands();
                    backend_->close();
                } catch (...) {
                    scheduler_.reset();
                    return;
                }
            }
            scheduler_.reset();
        }

        void close_backend();
        void recover_backend();
        [[nodiscard]] HandSnapshot snapshot();

        void retain_snapshot() noexcept {
            try {
                if (open_.load(std::memory_order_acquire)) {
                    (void)snapshot();
                }
            } catch (...) {
                // A concurrent transition or allocation failure must not prevent
                // admission shutdown. Existing evidence is still invalidated.
            }
        }

        void invalidate_cache_locked() {
            if (cache_valid_) {
                unified_detail::invalidate_cached_snapshot(cached_snapshot_);
                cached_snapshot_.captured_at = std::chrono::steady_clock::now();
                cached_snapshot_.snapshot_sequence = ++cache_revision_;
                cache_valid_ = false;
            }
        }

        void request_close(bool capture_cache = true) {
            if (capture_cache) {
                retain_snapshot();
            }
            {
                const std::lock_guard<std::mutex> lock(mutex_);
                if (state_ == State::kClosed && !backend_) {
                    return;
                }
                invalidate_commands("close");
                ++generation_;
                state_.store(State::kClosing, std::memory_order_release);
                open_.store(false, std::memory_order_release);
            }
            snapshot_cv_.notify_all();
        }

        [[noreturn]] void shutdown_expired(const char *operation) {
            ErrorContext context;
            {
                const std::lock_guard<std::mutex> lock(mutex_);
                context = {operation, unified_detail::model_name(config_.model), {}, generation_};
                if (state_ != State::kClosed) {
                    invalidate_commands(operation);
                    ++generation_;
                    state_.store(State::kFaulted, std::memory_order_release);
                    open_.store(false, std::memory_order_release);
                }
            }
            snapshot_cv_.notify_all();
            throw TimeoutError("lifecycle shutdown budget expired; resources retained until driver/callback returns",
                               context);
        }

        void lifecycle(const std::shared_ptr<Impl> &owner, const char *operation, const std::function<void()> &action) {
            const auto deadline = std::chrono::steady_clock::now() + config_.shutdown_timeout;
            std::unique_lock<std::timed_mutex> lock(lifecycle_mutex_, std::defer_lock);
            if (!lock.try_lock_until(deadline)) {
                shutdown_expired(operation);
            }
            if (lifecycle_result_.valid()) {
                if (lifecycle_result_.wait_until(deadline) != std::future_status::ready) {
                    shutdown_expired(operation);
                }
                lifecycle_worker_.join();
                // A subsequent close cleans up even when the previous recovery
                // failed. The original caller already received that failure.
                lifecycle_result_ = {};
            }
            if (std::string_view(operation) == "close") {
                const std::lock_guard<std::mutex> state(mutex_);
                if (state_ == State::kClosed && !backend_) {
                    return;
                }
            }
            auto result = std::make_shared<std::promise<void>>();
            lifecycle_result_ = result->get_future().share();
            auto &cleanup = CleanupQueue::instance();
            cleanup.retain_lifecycle();
            try {
                lifecycle_worker_ = std::thread([retained = owner, work = action, result, &cleanup]() mutable {
                    try {
                        work();
                        result->set_value();
                    } catch (...) {
                        result->set_exception(std::current_exception());
                    }
                    // Queue any last-owner deletion before allowing process
                    // teardown to stop the cleanup executor.
                    work = {};
                    retained.reset();
                    cleanup.release_lifecycle();
                });
            } catch (...) {
                cleanup.release_lifecycle();
                lifecycle_result_ = {};
                throw;
            }
            if (lifecycle_result_.wait_until(deadline) != std::future_status::ready) {
                shutdown_expired(operation);
            }
            lifecycle_worker_.join();
            auto completed = std::move(lifecycle_result_);
            lifecycle_result_ = {};
            completed.get();
        }

        [[nodiscard]] std::unique_ptr<unified_detail::DeviceBackend> create_backend() {
            return unified_detail::make_backend(config_, transport_factory_, shared_session_);
        }

        void ensure_ready(const char *operation) const {
            if (!open_.load(std::memory_order_acquire)) {
                throw DeviceNotReadyError(std::string("HandDevice must be ready before ") + operation,
                                          {operation, unified_detail::model_name(config_.model)});
            }
        }

        [[nodiscard]] unified_detail::DeviceBackend &require_ready(const char *operation) const {
            ensure_ready(operation);
            if (!backend_ || state_.load(std::memory_order_acquire) != State::kReady || !identity_ ||
                !verified_capabilities_) {
                throw DeviceNotReadyError(std::string("HandDevice must be ready before ") + operation,
                                          {operation, unified_detail::model_name(config_.model)});
            }
            return *backend_;
        }

        struct ExecutionContext {
            std::shared_ptr<unified_detail::DeviceBackend> backend;
            DeviceCapabilities capabilities;
            uint64_t generation;
        };

        [[nodiscard]] ExecutionContext capture(const char *operation) const {
            const std::lock_guard<std::mutex> lock(mutex_);
            static_cast<void>(require_ready(operation));
            return {backend_, capabilities_unlocked(), generation_};
        }

        // Call with mutex_ held. A context owns its backend even after close
        // releases it; the epoch also rejects contexts from close/reopen.
        void check_context(const ExecutionContext &context, const char *operation) const {
            if (context.generation != generation_ || context.backend != backend_ ||
                state_.load(std::memory_order_acquire) != State::kReady) {
                throw DeviceNotReadyError("HandDevice execution context is no longer ready",
                                          {operation, unified_detail::model_name(config_.model)});
            }
        }

        [[nodiscard]] std::shared_lock<std::shared_mutex> begin_io(const ExecutionContext &context,
                                                                   const char *operation) {
            std::shared_lock<std::shared_mutex> lease(io_mutex_);
            const std::lock_guard<std::mutex> lock(mutex_);
            check_context(context, operation);
            return lease;
        }

        void finish_io(const ExecutionContext &context, const char *operation) const {
            const std::lock_guard<std::mutex> lock(mutex_);
            check_context(context, operation);
        }

        template <typename Action>
        [[nodiscard]] decltype(std::declval<Action &>()()) complete_io(const ExecutionContext &context,
                                                                       const char *operation, Action action) const {
            auto result = [&] {
                try {
                    return action();
                } catch (...) {
                    // Native cancellation may complete exceptionally. Still
                    // fence completion against the current facade generation.
                    finish_io(context, operation);
                    throw;
                }
            }();
            finish_io(context, operation);
            return result;
        }

        // Reject runtime callback/poller lifecycle reentry before waiting for
        // facade lifecycle or I/O leases (those callers may be joined below).
        void assert_external_lifecycle() const {
            std::shared_ptr<unified_detail::DeviceBackend> selected;
            {
                const std::lock_guard<std::mutex> lock(mutex_);
                selected = backend_;
            }
            if (selected) {
                selected->assert_external_lifecycle();
            }
        }

        [[nodiscard]] const DeviceCapabilities &capabilities_unlocked() const {
            if (state_.load(std::memory_order_acquire) == State::kReady && verified_capabilities_) {
                return *verified_capabilities_;
            }
            return declared_capabilities_;
        }

        void observe_force_capabilities(const ForceSample &sample) {
            if (!verified_capabilities_ || sample.sensors.size() != 5) {
                return;
            }
            auto fields = declared_capabilities_.force_fields();
            const auto *channel = verified_capabilities_->telemetry_channel(TelemetrySemantic::kForce);
            if (!channel) {
                return;
            }
            std::vector<std::string_view> shared;
            for (const auto &field : fields) {
                if (sample.field(field.name)) {
                    shared.push_back(field.name);
                }
            }
            for (auto &field : fields) {
                const auto *reading = sample.field(field.name);
                field.supported = reading != nullptr;
                field.shared_fields = shared;
                field.sensor_names.clear();
                if (reading) {
                    for (size_t i = 0; i < reading->raw.size(); ++i) {
                        if (reading->raw[i]) {
                            field.sensor_names.push_back(channel->channels.at(i));
                        }
                    }
                }
            }
            verified_capabilities_->observed_force_fields = std::move(fields);
        }

        static void project_health(HandHealth &health, State state) {
            switch (state) {
            case State::kClosed:
                health.lifecycle_state = LifecycleState::kClosed;
                health.connection_state = ConnectionState::kClosed;
                health.identity_verified = false;
                health.polling_active = false;
                break;
            case State::kOpening:
                health.lifecycle_state = LifecycleState::kOpening;
                health.connection_state = ConnectionState::kConnecting;
                health.identity_verified = false;
                health.polling_active = false;
                break;
            case State::kReady:
                health.lifecycle_state = LifecycleState::kOpen;
                health.connection_state = ConnectionState::kConnected;
                health.identity_verified = true;
                break;
            case State::kClosing:
                health.lifecycle_state = LifecycleState::kClosing;
                health.connection_state = ConnectionState::kClosed;
                health.identity_verified = false;
                health.polling_active = false;
                break;
            case State::kRecovering:
                health.lifecycle_state = LifecycleState::kRecovering;
                health.connection_state = ConnectionState::kReconnecting;
                health.identity_verified = false;
                break;
            case State::kFaulted:
                health.lifecycle_state = LifecycleState::kFaulted;
                health.connection_state = ConnectionState::kFaulted;
                health.identity_verified = false;
                health.polling_active = false;
                break;
            }
        }

        [[nodiscard]] CommandHandle submit_raw(CommandSemantic semantic, const std::vector<double> &values,
                                               const char *operation,
                                               std::optional<std::chrono::milliseconds> timeout = {}) {
            const auto context = capture(operation);
            const auto &capabilities = context.capabilities;
            const auto *command = capabilities.command(semantic);
            if (!command) {
                throw UnsupportedOperationError(std::string("configured hand model does not support ") + operation,
                                                {operation, unified_detail::model_name(config_.model)});
            }
            const auto expectation =
                command->ack_support == AckSupport::kRequired ? AckExpectation::kRequired : AckExpectation::kNone;
            auto raw = validate_command_values(values, *command);
            const auto budget = timeout.value_or(config_.command_timeout);
            if (budget.count() <= 0) {
                throw ValidationError("command timeout must be positive");
            }
            return enqueue(context, {semantic, std::move(raw), budget, expectation}, operation);
        }

        [[nodiscard]] CommandHandle enqueue(const ExecutionContext &context, unified_detail::CommandRequest request,
                                            const char *operation) {
            const auto *capability = context.capabilities.command(request.semantic);
            const auto family = request.operation ? CommandFamily::kDiscreteOperation
                                                  : unified_detail::command_family(request.semantic);
            const auto support = request.operation
                                     ? (request.ack_expectation == AckExpectation::kRequired ? AckSupport::kRequired
                                                                                             : AckSupport::kUnsupported)
                                     : capability->ack_support;
            request.generation_valid = [this, context] {
                return context.generation == generation_.load(std::memory_order_acquire) &&
                       state_.load(std::memory_order_acquire) == State::kReady;
            };
            auto execute = [this, context, operation](const unified_detail::CommandRequest &accepted) {
                try {
                    auto lease = begin_io(context, operation);
                    return complete_io(context, operation, [&] { return context.backend->submit(accepted); });
                } catch (...) {
                    const std::lock_guard<std::mutex> lock(mutex_);
                    if (context.generation != generation_ || context.backend != backend_) {
                        throw ClosedError("accepted command generation ended",
                                          {operation, unified_detail::model_name(config_.model),
                                           unified_detail::command_family_name(
                                               accepted.operation ? CommandFamily::kDiscreteOperation
                                                                  : unified_detail::command_family(accepted.semantic)),
                                           context.generation});
                    }
                    throw;
                }
            };
            const std::lock_guard<std::mutex> lock(mutex_);
            check_context(context, operation);
            return scheduler_->accept(std::move(request), support,
                                      {operation, unified_detail::model_name(config_.model),
                                       unified_detail::command_family_name(family), context.generation,
                                       context.capabilities.transport.request_id},
                                      std::move(execute));
        }

        // Called under mutex_ at the same point that closes admission and
        // invalidates a generation. Executing work is fenced by its I/O context.
        void invalidate_commands(const char *operation) {
            invalidate_cache_locked();
            scheduler_->invalidate(std::make_exception_ptr(
                ClosedError("pending command generation ended",
                            {operation, unified_detail::model_name(config_.model), {}, generation_})));
        }

        [[nodiscard]] unified_detail::TelemetrySample
        read_raw(TelemetrySemantic semantic, std::chrono::milliseconds timeout, const char *operation) {
            const auto context = capture(operation);
            const auto &capabilities = context.capabilities;
            if (!capabilities.supports(semantic)) {
                throw UnsupportedOperationError(std::string("configured hand model does not support ") + operation,
                                                {operation, unified_detail::model_name(config_.model)});
            }
            if (timeout.count() <= 0) {
                throw ValidationError(std::string(operation) + " timeout must be positive",
                                      {operation, unified_detail::model_name(config_.model)});
            }
            auto lease = begin_io(context, operation);
            auto sample = complete_io(context, operation, [&] { return context.backend->read(semantic, timeout); });
            const auto generation = context.backend->health().connection_generation;
            finish_io(context, operation);
            std::visit([generation](auto &value) { value.meta.connection_generation = generation; }, sample);
            return sample;
        }

        [[nodiscard]] CommandHandle enqueue_operation(DeviceOperation operation, std::chrono::milliseconds timeout,
                                                      const char *name) {
            const auto context = capture(name);
            if (operation == DeviceOperation::kReadDeviceInfo || !context.capabilities.supports(operation)) {
                throw UnsupportedOperationError("operation is unsupported; use get_device_info for identity reads");
            }
            if (timeout.count() <= 0) {
                throw ValidationError("operation timeout must be positive");
            }
            unified_detail::CommandRequest request;
            request.operation = operation;
            request.timeout = timeout;
            request.ack_expectation = context.capabilities.operations.acknowledged(operation)
                                          ? AckExpectation::kRequired
                                          : AckExpectation::kNone;
            return enqueue(context, std::move(request), name);
        }

        [[nodiscard]] CommandHandle operate(DeviceOperation operation, const char *name) {
            const auto context = capture(name);
            const auto &capabilities = context.capabilities;
            if (!capabilities.supports(operation)) {
                throw UnsupportedOperationError(std::string("configured hand model does not support ") + name,
                                                {name, unified_detail::model_name(config_.model)});
            }
            auto lease = begin_io(context, name);
            return complete_io(context, name,
                               [&] { return context.backend->operate(operation, config_.command_timeout); });
        }

        void run_stream(const std::shared_ptr<EventQueue<HandEvent>> &queue) {
            std::map<TelemetrySemantic, std::chrono::steady_clock::time_point> last_seen;
            while (!stream_stop_.load(std::memory_order_acquire)) {
                try {
                    // Streams and explicit snapshots must expose the same
                    // generation, sequence and freshness metadata for all fields.
                    auto snapshot = this->snapshot();
                    unified_detail::publish_snapshot_events(snapshot, queue, last_seen);
                } catch (...) {
                    break;
                }
                std::unique_lock<std::mutex> lock(stream_mutex_);
                stream_cv_.wait_for(lock, std::chrono::milliseconds{2},
                                    [this] { return stream_stop_.load(std::memory_order_acquire); });
            }
            queue->close();
        }

        void stop_stream() {
            std::thread stopping;
            std::shared_ptr<EventQueue<HandEvent>> queue;
            {
                const std::lock_guard<std::mutex> lock(stream_mutex_);
                stream_stop_.store(true, std::memory_order_release);
                queue = event_queue_;
                if (stream_thread_.joinable()) {
                    stopping = std::move(stream_thread_);
                }
            }
            if (queue) {
                queue->close();
            }
            stream_cv_.notify_all();
            if (stopping.joinable()) {
                stopping.join();
            }
        }

        HandDeviceConfig config_;
        DeviceCapabilities declared_capabilities_;
        TransportFactory transport_factory_;
        std::shared_ptr<transport::Session> shared_session_;
        mutable std::mutex mutex_;
        // Lock order: lifecycle_mutex_ -> io_mutex_ -> mutex_. Never wait for
        // lifecycle/I/O/stream joins while holding mutex_. Runtime calls that
        // send/wait/join and backend cache reads run outside mutex_. Facade
        // cache metadata is published under it after epoch validation. Event
        // publication and consumer callbacks run without it.
        std::timed_mutex lifecycle_mutex_;
        std::thread lifecycle_worker_;
        std::shared_future<void> lifecycle_result_;
        std::shared_mutex io_mutex_;
        std::shared_ptr<unified_detail::DeviceBackend> backend_;
        std::atomic<uint64_t> generation_{0};
        // Object-level constraint. Unlike identity_, this survives close(),
        // failed open() and failed recovery attempts.
        std::optional<DeviceIdentity> expected_identity_;
        std::optional<DeviceIdentity> identity_;
        std::optional<ErrorInfo> refresh_error_;
        std::optional<DeviceCapabilities> verified_capabilities_;
        std::optional<PollingPlan> polling_plan_;
        std::atomic<State> state_{State::kClosed};
        HandSnapshot cached_snapshot_;
        uint64_t cache_generation_ = 0;
        uint64_t cache_revision_ = 0;
        bool cache_valid_ = false;
        std::optional<uint64_t> cached_source_sequence_;
        uint64_t snapshot_sequence_ = 0;
        std::optional<std::chrono::steady_clock::time_point> last_snapshot_time_;
        std::atomic<bool> open_{false};
        mutable std::mutex snapshot_wait_mutex_;
        mutable std::condition_variable snapshot_cv_;
        std::mutex stream_mutex_;
        std::condition_variable stream_cv_;
        std::shared_ptr<EventQueue<HandEvent>> event_queue_;
        std::thread stream_thread_;
        std::atomic<bool> stream_stop_{true};
        // Executor borrows this Impl; retained lifecycle/cleanup ownership joins
        // it before any facade state or backend can be destroyed.
        std::unique_ptr<unified_detail::CommandScheduler> scheduler_;
    };

    HandDevice::HandDevice(HandDeviceConfig config)
        : impl_(Impl::create(std::move(config), TransportFactory{}, nullptr)) {}

    HandDevice::HandDevice(HandDeviceConfig config, TransportFactory transport_factory)
        : impl_(Impl::create(std::move(config), std::move(transport_factory), nullptr)) {
        if (!impl_->transport_factory_) {
            throw ValidationError("HandDevice transport factory must not be empty");
        }
    }

    HandDevice::HandDevice(HandDeviceConfig config, TransportFactory transport_factory,
                           DeviceIdentity expected_identity)
        : impl_(Impl::create(std::move(config), std::move(transport_factory), nullptr, std::move(expected_identity))) {
        if (!impl_->transport_factory_) {
            throw ValidationError("HandDevice transport factory must not be empty");
        }
    }

    HandDevice::HandDevice(HandDeviceConfig config, std::shared_ptr<transport::Session> shared_session)
        : impl_(Impl::create(std::move(config), TransportFactory{}, std::move(shared_session))) {
        if (!impl_->shared_session_) {
            throw ValidationError("HandDevice shared session must not be null");
        }
    }

    HandDevice::~HandDevice() {
        try {
            close();
        } catch (...) // destructor must not throw; cleanup retains the whole Impl
        {
            impl_->request_close(false);
        }
    }

    void HandDevice::open() {
        impl_->assert_external_lifecycle();
        const std::lock_guard<std::timed_mutex> lifecycle(impl_->lifecycle_mutex_);
        std::optional<DeviceIdentity> expected;
        uint64_t opening_generation;
        {
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            if (impl_->state_.load(std::memory_order_acquire) == Impl::State::kReady) {
                return;
            }
            if (impl_->state_.load(std::memory_order_acquire) != Impl::State::kClosed) {
                throw LifecycleError("HandDevice cannot open in its current lifecycle state",
                                     {"open", unified_detail::model_name(impl_->config_.model)});
            }
            opening_generation = ++impl_->generation_;
            impl_->refresh_error_.reset();
            impl_->state_.store(Impl::State::kOpening, std::memory_order_release);
            impl_->identity_.reset();
            impl_->verified_capabilities_.reset();
            impl_->snapshot_sequence_ = 0;
            impl_->last_snapshot_time_.reset();
            expected = impl_->expected_identity_;
        }
        std::shared_ptr<unified_detail::DeviceBackend> next;
        try {
            next = impl_->create_backend();
            auto initialized = unified_detail::initialize_connection(
                *next, impl_->config_, declared_capabilities(impl_->config_), expected ? &*expected : nullptr);
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            if (impl_->generation_ != opening_generation) {
                throw ClosedError("open interrupted by lifecycle shutdown");
            }
            impl_->backend_ = next;
            ++impl_->cache_generation_;
            impl_->cached_source_sequence_.reset();
            if (!impl_->expected_identity_) {
                impl_->expected_identity_ = initialized.identity;
            }
            impl_->identity_ = std::move(initialized.identity);
            impl_->verified_capabilities_ = std::move(initialized.capabilities);
            impl_->state_.store(Impl::State::kReady, std::memory_order_release);
            impl_->open_.store(true, std::memory_order_release);
        } catch (...) {
            const auto original = std::current_exception();
            const auto finish_failure = [&] {
                {
                    const std::lock_guard<std::mutex> lock(impl_->mutex_);
                    impl_->state_.store(Impl::State::kClosed, std::memory_order_release);
                    impl_->open_.store(false, std::memory_order_release);
                }
                impl_->snapshot_cv_.notify_all();
                std::rethrow_exception(original);
            };
            if (next) {
                try {
                    next->close();
                } catch (...) {
                    finish_failure();
                }
            }
            finish_failure();
        }
    }

    void HandDevice::close() {
        impl_->assert_external_lifecycle();
        impl_->request_close();
        impl_->lifecycle(impl_, "close", [owner = impl_] { owner->close_backend(); });
    }

    void HandDevice::Impl::close_backend() {
        auto *impl_ = this;
        std::shared_ptr<unified_detail::DeviceBackend> closing;
        {
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            if (impl_->state_.load(std::memory_order_acquire) == Impl::State::kClosed && !impl_->backend_) {
                return;
            }
            impl_->invalidate_commands("close");
            ++impl_->generation_;
            impl_->state_.store(Impl::State::kClosing, std::memory_order_release);
            closing = impl_->backend_;
            impl_->open_.store(false, std::memory_order_release);
            impl_->identity_.reset();
            impl_->verified_capabilities_.reset();
            impl_->polling_plan_.reset();
            impl_->snapshot_sequence_ = 0;
            impl_->last_snapshot_time_.reset();
        }
        impl_->snapshot_cv_.notify_all();
        impl_->stop_stream();
        std::exception_ptr error;
        // Close first to cancel native waiters, then drain facade leases. Taking
        // the exclusive lease first would leave close waiting for an ACK.
        try {
            if (closing) {
                closing->close();
            }
        } catch (...) {
            error = std::current_exception();
        }
        {
            const std::unique_lock<std::shared_mutex> drain(impl_->io_mutex_);
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            if (!error) {
                impl_->backend_.reset();
                impl_->state_.store(Impl::State::kClosed, std::memory_order_release);
            } else {
                impl_->state_.store(Impl::State::kFaulted, std::memory_order_release);
            }
        }
        impl_->snapshot_cv_.notify_all();
        if (error) {
            std::rethrow_exception(error);
        }
    }

    bool HandDevice::is_open() const noexcept { return impl_->open_.load(std::memory_order_acquire); }

    DeviceCapabilities HandDevice::get_capabilities() const {
        if (!impl_->open_.load(std::memory_order_acquire)) {
            return impl_->declared_capabilities_;
        }
        const std::lock_guard<std::mutex> lock(impl_->mutex_);
        return impl_->capabilities_unlocked();
    }

    HandHealth HandDevice::get_health() const {
        std::shared_ptr<unified_detail::DeviceBackend> selected;
        uint64_t generation;
        {
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            const auto state = impl_->state_.load(std::memory_order_acquire);
            if (!impl_->backend_ || (state != Impl::State::kReady && state != Impl::State::kFaulted)) {
                HandHealth health;
                Impl::project_health(health, state);
                health.command_families = impl_->scheduler_->metrics();
                return health;
            }
            selected = impl_->backend_;
            generation = impl_->generation_;
        }
        auto health = selected->health(); // Cache only, without the facade lock.
        const std::lock_guard<std::mutex> lock(impl_->mutex_);
        if (generation != impl_->generation_ || selected != impl_->backend_) {
            health = HandHealth{};
        }
        Impl::project_health(health, impl_->state_.load(std::memory_order_acquire));
        if (impl_->state_.load(std::memory_order_acquire) == Impl::State::kFaulted && impl_->refresh_error_) {
            health.recent_error = impl_->refresh_error_;
        }
        // Unified setter counters own logical acceptance/completion. Native
        // transaction/session counters remain available without being added a
        // second time to these family totals.
        health.command_families = impl_->scheduler_->metrics();
        return health;
    }

    DeviceIdentity HandDevice::verify_identity(std::chrono::milliseconds timeout) {
        impl_->assert_external_lifecycle();
        impl_->ensure_ready("verify_identity");
        const std::lock_guard<std::timed_mutex> lifecycle(impl_->lifecycle_mutex_);
        impl_->retain_snapshot();
        const auto context = impl_->capture("verify_identity");
        if (timeout.count() <= 0) {
            throw ValidationError("verify_identity timeout must be positive");
        }
        {
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            impl_->check_context(context, "verify_identity");
            impl_->invalidate_commands("verify_identity");
            ++impl_->generation_;
            impl_->state_.store(Impl::State::kOpening, std::memory_order_release);
            impl_->open_.store(false, std::memory_order_release);
            impl_->identity_.reset();
            impl_->verified_capabilities_.reset();
        }
        context.backend->cancel_commands();
        const std::unique_lock<std::shared_mutex> drain(impl_->io_mutex_);
        try {
            auto identity = context.backend->verify_identity(impl_->config_.side, timeout);
            unified_detail::validate_connection_identity(
                identity, impl_->config_,
                impl_->expected_identity_
                    ? &::linkerbot::hand::detail::checked_value(impl_->expected_identity_,
                                                                "missing runtime state: impl_->expected_identity_")
                    : nullptr,
                "verify_identity");
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            if (impl_->generation_ != context.generation + 1) {
                throw ClosedError("identity verification interrupted by shutdown");
            }
            impl_->verified_capabilities_ = unified_detail::capabilities_for_identity(identity, &impl_->config_);
            impl_->identity_ = identity;
            impl_->state_.store(Impl::State::kReady, std::memory_order_release);
            impl_->open_.store(true, std::memory_order_release);
            return identity;
        } catch (...) {
            const auto original = std::current_exception();
            const auto invalidate_identity = [&] {
                {
                    const std::lock_guard<std::mutex> lock(impl_->mutex_);
                    if (impl_->generation_ == context.generation + 1) {
                        impl_->polling_plan_.reset();
                        impl_->state_.store(Impl::State::kFaulted, std::memory_order_release);
                    }
                }
                impl_->snapshot_cv_.notify_all();
                impl_->stream_cv_.notify_all();
            };
            try {
                context.backend->stop_polling();
            } catch (...) {
                invalidate_identity();
                std::rethrow_exception(original);
            }
            invalidate_identity();
            std::rethrow_exception(original);
        }
    }

    DeviceIdentity HandDevice::get_device_info(std::chrono::milliseconds timeout) { return verify_identity(timeout); }

    DeviceIdentity HandDevice::refresh_device_info(std::chrono::milliseconds timeout) {
        if (timeout.count() <= 0) {
            throw ValidationError("refresh_device_info timeout must be positive");
        }
        impl_->assert_external_lifecycle();
        impl_->retain_snapshot();
        const auto context = impl_->capture("refresh_device_info");
        try {
            auto lease = impl_->begin_io(context, "refresh_device_info");
            auto identity = impl_->complete_io(context, "refresh_device_info", [&] {
                return context.backend->verify_identity(impl_->config_.side, timeout);
            });
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            impl_->check_context(context, "refresh_device_info");
            unified_detail::validate_connection_identity(
                identity, impl_->config_, impl_->expected_identity_ ? &*impl_->expected_identity_ : nullptr,
                "refresh_device_info");
            // Identity refresh leaves this generation's capability observations
            // and polling decisions intact, matching the Python read path.
            impl_->identity_ = identity;
            impl_->cached_source_sequence_.reset();
            return identity;
        } catch (const HandError &error) {
            if (error.code() != ErrorCode::kIdentityMismatch && error.code() != ErrorCode::kTransport &&
                error.code() != ErrorCode::kDeviceNotReady) {
                throw;
            }
            bool faulted = false;
            {
                const std::lock_guard<std::mutex> lock(impl_->mutex_);
                if (context.generation == impl_->generation_ && context.backend == impl_->backend_ &&
                    impl_->state_.load(std::memory_order_acquire) == Impl::State::kReady) {
                    impl_->invalidate_commands("refresh_device_info");
                    ++impl_->generation_;
                    impl_->state_.store(Impl::State::kFaulted, std::memory_order_release);
                    impl_->open_.store(false, std::memory_order_release);
                    impl_->identity_.reset();
                    impl_->verified_capabilities_.reset();
                    impl_->polling_plan_.reset();
                    impl_->refresh_error_ =
                        ErrorInfo{error.code(), error.what(),
                                  error.code() == ErrorCode::kIdentityMismatch ? "IdentityMismatchError"
                                  : error.code() == ErrorCode::kTransport      ? "TransportError"
                                                                               : "DeviceNotReadyError",
                                  error.context()};
                    faulted = true;
                }
            }
            if (faulted) {
                context.backend->cancel_commands();
                try {
                    context.backend->stop_polling();
                } catch (...) { /* Preserve the identity/transport error. */
                }
                impl_->snapshot_cv_.notify_all();
                impl_->stream_cv_.notify_all();
            }
            throw;
        }
    }

    std::optional<DeviceIdentity> HandDevice::get_identity() const {
        if (impl_->state_.load(std::memory_order_acquire) != Impl::State::kReady) {
            return std::nullopt;
        }
        const std::lock_guard<std::mutex> lock(impl_->mutex_);
        if (impl_->state_.load(std::memory_order_acquire) != Impl::State::kReady) {
            return std::nullopt;
        }
        return impl_->identity_;
    }

    std::optional<DeviceCapabilities> HandDevice::get_verified_capabilities() const {
        if (impl_->state_.load(std::memory_order_acquire) != Impl::State::kReady) {
            return std::nullopt;
        }
        const std::lock_guard<std::mutex> lock(impl_->mutex_);
        if (impl_->state_.load(std::memory_order_acquire) != Impl::State::kReady) {
            return std::nullopt;
        }
        return impl_->verified_capabilities_;
    }

    CommandHandle HandDevice::set_angles(const std::vector<int32_t> &values) {
        const auto context = impl_->capture("set_angles");
        const auto &capabilities = context.capabilities;
        const auto *command = capabilities.command(CommandSemantic::kAngles);
        if (!command) {
            throw UnsupportedOperationError("configured hand model does not support angle commands",
                                            {"set_angles", unified_detail::model_name(impl_->config_.model), "angles"});
        }
        std::vector<double> raw(values.begin(), values.end());
        raw = validate_raw_angles(raw, capabilities);
        const auto expectation =
            command->ack_support == AckSupport::kRequired ? AckExpectation::kRequired : AckExpectation::kNone;
        return impl_->enqueue(context,
                              {CommandSemantic::kAngles, std::move(raw), impl_->config_.command_timeout, expectation},
                              "set_angles");
    }

    CommandHandle HandDevice::set_percent_angles(const std::vector<float> &values) {
        const auto context = impl_->capture("set_percent_angles");
        const auto &capabilities = context.capabilities;
        const auto *command = capabilities.command(CommandSemantic::kAngles);
        if (!command) {
            throw UnsupportedOperationError(
                "configured hand model does not support angle commands",
                {"set_percent_angles", unified_detail::model_name(impl_->config_.model), "angles"});
        }
        std::vector<double> percent(values.begin(), values.end());
        auto raw = percent_to_raw_angles(percent, capabilities);
        const auto expectation =
            command->ack_support == AckSupport::kRequired ? AckExpectation::kRequired : AckExpectation::kNone;
        return impl_->enqueue(context,
                              {CommandSemantic::kAngles, std::move(raw), impl_->config_.command_timeout, expectation},
                              "set_percent_angles");
    }

#define LINKERBOT_HAND_DEFINE_SETTER(Method, Semantic)                                                                 \
    CommandHandle HandDevice::Method(const std::vector<double> &values) {                                              \
        return impl_->submit_raw(CommandSemantic::Semantic, values, #Method);                                          \
    }

    CommandHandle HandDevice::set_torques(const std::vector<double> &values, std::chrono::milliseconds timeout) {
        return impl_->submit_raw(CommandSemantic::kTorques, values, "set_torques", timeout);
    }
    LINKERBOT_HAND_DEFINE_SETTER(set_torques_limits, kTorqueLimits)
    LINKERBOT_HAND_DEFINE_SETTER(set_speeds, kSpeeds)
    LINKERBOT_HAND_DEFINE_SETTER(set_accelerations, kAccelerations)
    LINKERBOT_HAND_DEFINE_SETTER(set_position_kp, kPositionKp)
    LINKERBOT_HAND_DEFINE_SETTER(set_position_kd, kPositionKd)
    LINKERBOT_HAND_DEFINE_SETTER(set_torque_kd, kTorqueKd)
    LINKERBOT_HAND_DEFINE_SETTER(set_torque_limits, kTorqueLimits)
    LINKERBOT_HAND_DEFINE_SETTER(set_motion_times, kMotionTimes)
    LINKERBOT_HAND_DEFINE_SETTER(set_limit_compensations, kLimitCompensations)
    LINKERBOT_HAND_DEFINE_SETTER(set_current_limits, kCurrentLimits)
    LINKERBOT_HAND_DEFINE_SETTER(set_stall_detection_times, kStallDetectionTimes)
    LINKERBOT_HAND_DEFINE_SETTER(set_stall_thresholds, kStallThresholds)
    LINKERBOT_HAND_DEFINE_SETTER(set_stall_hold_currents, kStallHoldCurrents)

#undef LINKERBOT_HAND_DEFINE_SETTER

#define LINKERBOT_HAND_DEFINE_OPERATION(Method, Operation)                                                             \
    CommandHandle HandDevice::Method() { return impl_->operate(DeviceOperation::Operation, #Method); }

    LINKERBOT_HAND_DEFINE_OPERATION(clear_faults, kClearFaults)
    LINKERBOT_HAND_DEFINE_OPERATION(enable, kEnable)
    LINKERBOT_HAND_DEFINE_OPERATION(disable, kDisable)
    LINKERBOT_HAND_DEFINE_OPERATION(save_parameters, kSaveParameters)
    CommandHandle HandDevice::submit_operation(DeviceOperation operation, std::chrono::milliseconds timeout) {
        return impl_->enqueue_operation(operation, timeout, "submit_operation");
    }

    CommandHandle HandDevice::save_parameters_confirmed(std::chrono::milliseconds timeout) {
        return impl_->enqueue_operation(DeviceOperation::kSaveParametersConfirmed, timeout,
                                        "save_parameters_confirmed");
    }
    CommandHandle HandDevice::calibrate(std::chrono::milliseconds timeout) {
        return impl_->enqueue_operation(DeviceOperation::kCalibrate, timeout, "calibrate");
    }

#undef LINKERBOT_HAND_DEFINE_OPERATION

    AngleSample HandDevice::get_angles(std::chrono::milliseconds timeout) {
        return std::get<AngleSample>(impl_->read_raw(TelemetrySemantic::kAngles, timeout, "get_angles"));
    }

#define LINKERBOT_HAND_DEFINE_GETTER(Method, Semantic, Sample)                                                         \
    Sample HandDevice::Method(std::chrono::milliseconds timeout) {                                                     \
        return std::get<Sample>(impl_->read_raw(TelemetrySemantic::Semantic, timeout, #Method));                       \
    }

    LINKERBOT_HAND_DEFINE_GETTER(get_torques_limits, kTorqueLimits, TorqueLimitSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_speeds, kMeasuredSpeeds, MeasuredSpeedSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_configured_speeds, kConfiguredSpeeds, ConfiguredSpeedSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_configured_accelerations, kConfiguredAccelerations, ConfiguredAccelerationSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_position_kp, kPositionKp, PositionKpSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_position_kd, kPositionKd, PositionKdSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_torque_kd, kTorqueKd, TorqueKdSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_torque_limits, kTorqueLimits, TorqueLimitSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_motion_times, kMotionTimes, MotionTimeSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_limit_compensations, kLimitCompensations, LimitCompensationSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_currents, kCurrents, CurrentSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_current_limits, kCurrentLimits, CurrentLimitSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_torques, kTorques, TorqueSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_temperatures, kTemperatures, TemperatureSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_faults, kFaults, FaultSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_voltages, kVoltages, VoltageSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_stall_detection_times, kStallDetectionTimes, StallDetectionTimeSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_stall_thresholds, kStallThresholds, StallThresholdSample)
    LINKERBOT_HAND_DEFINE_GETTER(get_stall_hold_currents, kStallHoldCurrents, StallHoldCurrentSample)

#undef LINKERBOT_HAND_DEFINE_GETTER

    ForceSample HandDevice::get_force(std::chrono::milliseconds timeout) {
        return get_force(std::vector<std::string>{}, timeout);
    }

    ForceSample HandDevice::get_force(const std::vector<std::string> &fields, std::chrono::milliseconds timeout) {
        if (timeout.count() <= 0) {
            throw ValidationError("get_force timeout must be positive");
        }
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::vector<std::string> requested;
        for (const auto &name : fields.empty() ? std::vector<std::string>{"fn", "fns"} : fields) {
            unified_detail::validate_force_field(name);
            if (std::find(requested.begin(), requested.end(), name) == requested.end()) {
                requested.push_back(name);
            }
        }
        const auto context = impl_->capture("get_force");
        const auto &capabilities = context.capabilities;
        auto lease = impl_->begin_io(context, "get_force");
        auto &selected = *context.backend;
        std::vector<TelemetrySemantic> queries;
        for (const auto &name : requested) {
            const auto field = capabilities.force_field(name);
            if (field.supported == false) {
                if (!fields.empty()) {
                    throw UnsupportedOperationError("configured hand model does not support force field " + name);
                }
                continue;
            }
            if (field.query_channel &&
                std::find(queries.begin(), queries.end(), *field.query_channel) == queries.end()) {
                queries.push_back(*field.query_channel);
            }
        }
        if (queries.empty()) {
            throw UnsupportedOperationError("configured hand model does not support requested force fields");
        }
        const auto generation = selected.health().connection_generation;
        ForceSample result;
        for (const auto query : queries) {
            const auto remaining =
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            if (remaining.count() <= 0) {
                throw TimeoutError("get_force total timeout expired");
            }
            auto sample = std::get<ForceSample>(
                impl_->complete_io(context, "get_force", [&] { return selected.read(query, remaining); }));
            sample.meta.connection_generation = generation;
            if (sample.meta.sequence == 0 && sample.meta.valid) {
                const std::lock_guard<std::mutex> lock(impl_->mutex_);
                impl_->check_context(context, "get_force");
                sample.meta.sequence = ++impl_->snapshot_sequence_;
            }
            unified_detail::complete_force_meta(sample, generation, capabilities);
            unified_detail::merge_force_samples(result, sample);
        }
        if (selected.health().connection_generation != generation) {
            throw DeviceNotReadyError("force queries crossed a connection generation");
        }
        {
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            impl_->check_context(context, "get_force");
            impl_->observe_force_capabilities(result);
        }
        for (const auto &name : requested) {
            if (!fields.empty() && !result.field(name)) {
                throw UnsupportedOperationError("device sensors do not expose force field " + name);
            }
        }
        if (fields.empty() && !result.field("fn") && !result.field("fns")) {
            throw UnsupportedOperationError("device sensors do not expose fn or fns");
        }
        result.requested_fields = std::move(requested);
        unified_detail::complete_force_meta(result, generation, capabilities);
        return result;
    }

    NormalForceResultantSample HandDevice::get_normal_force_resultant(std::chrono::milliseconds timeout) {
        return unified_detail::normal_force_resultant(get_force({"fns"}, timeout));
    }

    HandSnapshot HandDevice::get_snapshot() const { return impl_->snapshot(); }

    HandSnapshot HandDevice::Impl::snapshot() {
        const auto context = capture("get_snapshot");
        const auto &capabilities = context.capabilities;
        auto result = context.backend->snapshot(capabilities.model, capabilities.side, capabilities);
        const auto health = context.backend->health();
        const auto generation = health.connection_generation;
        const std::lock_guard<std::mutex> lock(mutex_);
        check_context(context, "get_snapshot");

        std::optional<std::chrono::steady_clock::time_point> latest_sample;
        const auto observe_sample = [&latest_sample](const auto &sample) {
            if (sample && sample->meta.collected_at &&
                (!latest_sample || *sample->meta.collected_at > *latest_sample)) {
                latest_sample = sample->meta.collected_at;
            }
        };
        observe_sample(result.position_kp);
        observe_sample(result.position_kd);
        observe_sample(result.torque_kd);
        observe_sample(result.torque_limits);
        observe_sample(result.motion_times);
        observe_sample(result.limit_compensations);
        observe_sample(result.angles);
        observe_sample(result.measured_speeds);
        observe_sample(result.configured_speeds);
        observe_sample(result.configured_accelerations);
        observe_sample(result.currents);
        observe_sample(result.current_limits);
        observe_sample(result.torques);
        observe_sample(result.temperatures);
        observe_sample(result.force);
        observe_sample(result.normal_force_resultant);
        observe_sample(result.faults);
        observe_sample(result.voltages);
        observe_sample(result.stall_detection_times);
        observe_sample(result.stall_thresholds);
        observe_sample(result.stall_hold_currents);
        if (latest_sample && (!last_snapshot_time_ || *last_snapshot_time_ != *latest_sample)) {
            ++snapshot_sequence_;
            last_snapshot_time_ = latest_sample;
            snapshot_cv_.notify_all();
        }
        result.snapshot_sequence = snapshot_sequence_;
        result.connection_generation = generation;
        result.device_info = identity_;

        const auto now = std::chrono::steady_clock::now();
        const auto complete_meta = [&](auto &sample, TelemetrySemantic semantic) {
            if (!sample) {
                return;
            }
            auto &meta = sample->meta;
            if (meta.sequence == 0) {
                meta.sequence = result.snapshot_sequence;
            }
            meta.connection_generation = result.connection_generation;
            unified_detail::complete_freshness(meta, capabilities.telemetry_channel(semantic), now);
        };
        complete_meta(result.position_kp, TelemetrySemantic::kPositionKp);
        complete_meta(result.position_kd, TelemetrySemantic::kPositionKd);
        complete_meta(result.torque_kd, TelemetrySemantic::kTorqueKd);
        complete_meta(result.torque_limits, TelemetrySemantic::kTorqueLimits);
        complete_meta(result.motion_times, TelemetrySemantic::kMotionTimes);
        complete_meta(result.limit_compensations, TelemetrySemantic::kLimitCompensations);
        complete_meta(result.angles, TelemetrySemantic::kAngles);
        complete_meta(result.measured_speeds, TelemetrySemantic::kMeasuredSpeeds);
        complete_meta(result.configured_speeds, TelemetrySemantic::kConfiguredSpeeds);
        complete_meta(result.configured_accelerations, TelemetrySemantic::kConfiguredAccelerations);
        complete_meta(result.currents, TelemetrySemantic::kCurrents);
        complete_meta(result.current_limits, TelemetrySemantic::kCurrentLimits);
        complete_meta(result.torques, TelemetrySemantic::kTorques);
        complete_meta(result.temperatures, TelemetrySemantic::kTemperatures);
        complete_meta(result.force, TelemetrySemantic::kForce);
        if (result.force) {
            unified_detail::complete_force_meta(*result.force, result.connection_generation, capabilities);
            observe_force_capabilities(*result.force);
        }
        complete_meta(result.normal_force_resultant, TelemetrySemantic::kForceFns);
        complete_meta(result.faults, TelemetrySemantic::kFaults);
        complete_meta(result.voltages, TelemetrySemantic::kVoltages);
        complete_meta(result.stall_detection_times, TelemetrySemantic::kStallDetectionTimes);
        complete_meta(result.stall_thresholds, TelemetrySemantic::kStallThresholds);
        complete_meta(result.stall_hold_currents, TelemetrySemantic::kStallHoldCurrents);
        auto retained = result;
        unified_detail::shape_cached_samples(retained, capabilities);
        unified_detail::set_cached_generation(retained, cache_generation_);
        if (!cache_valid_ || cached_source_sequence_ != result.snapshot_sequence) {
            ++cache_revision_;
        }
        retained.snapshot_sequence = cache_revision_;
        cached_snapshot_ = std::move(retained);
        cached_source_sequence_ = result.snapshot_sequence;
        cache_valid_ = true;
        if (!health.connected()) {
            invalidate_cache_locked();
        }
        return result;
    }

    HandSnapshot HandDevice::get_cached_snapshot() const {
        try {
            // This only projects native caches, never a device transaction.
            if (impl_->open_.load(std::memory_order_acquire)) {
                (void)impl_->snapshot();
            }
        } catch (const DeviceNotReadyError &) {
            // The lifecycle transition owns invalidation under the same mutex.
        }
        const std::lock_guard<std::mutex> lock(impl_->mutex_);
        if (impl_->state_.load(std::memory_order_acquire) != Impl::State::kReady) {
            impl_->invalidate_cache_locked();
        }
        return impl_->cached_snapshot_;
    }

    HandSnapshot HandDevice::wait_for_snapshot(const HandSnapshot &previous, std::chrono::milliseconds timeout) const {
        if (timeout.count() <= 0) {
            throw ValidationError("wait_for_snapshot timeout must be positive",
                                  {"wait_for_snapshot", unified_detail::model_name(impl_->config_.model)});
        }

        const auto changed = [&previous](const HandSnapshot &current) {
            return current.connection_generation != previous.connection_generation ||
                   current.snapshot_sequence != previous.snapshot_sequence;
        };
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        auto current = get_snapshot();
        while (!changed(current)) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                return current;
            }

            std::unique_lock<std::mutex> wait_lock(impl_->snapshot_wait_mutex_);
            impl_->snapshot_cv_.wait_until(wait_lock, std::min(deadline, now + std::chrono::milliseconds{2}));
            wait_lock.unlock();
            current = get_snapshot();
        }
        return current;
    }

    void HandDevice::start_polling() {
        start_polling({PollingChannel{TelemetrySemantic::kAngles, impl_->config_.polling_interval}});
    }

    void HandDevice::start_polling(const PollingPlan &requested) {
        if (requested.empty() && impl_->config_.empty_polling_plan_disables) {
            stop_polling();
            return;
        }
        impl_->assert_external_lifecycle();
        impl_->ensure_ready("start_polling");
        const std::lock_guard<std::timed_mutex> lifecycle(impl_->lifecycle_mutex_);
        const auto context = impl_->capture("start_polling");
        const auto &capabilities = context.capabilities;
        auto plan = requested;
        if (plan.empty()) {
            plan.push_back({TelemetrySemantic::kAngles, impl_->config_.polling_interval});
        }

        std::set<TelemetrySemantic> unique;
        for (const auto &channel : plan) {
            if (channel.interval.count() <= 0) {
                throw PollingPlanError("polling intervals must be positive",
                                       {"start_polling", unified_detail::model_name(impl_->config_.model)});
            }
            if (!unique.insert(channel.semantic).second) {
                throw PollingPlanError("polling semantics must be unique",
                                       {"start_polling", unified_detail::model_name(impl_->config_.model)});
            }
            if (!capabilities.supports(channel.semantic) ||
                (impl_->config_.l6_protocol != L6Protocol::kHopV004 &&
                 !unified_detail::polling_supported(impl_->config_.model, channel.semantic))) {
                throw UnsupportedOperationError("configured telemetry is not supported for unified polling",
                                                {"start_polling", unified_detail::model_name(impl_->config_.model)});
            }
        }
        auto lease = impl_->begin_io(context, "start_polling");
        context.backend->start_polling(plan);
        const std::lock_guard<std::mutex> lock(impl_->mutex_);
        impl_->check_context(context, "start_polling");
        impl_->polling_plan_ = std::move(plan);
    }

    void HandDevice::stop_polling() {
        impl_->assert_external_lifecycle();
        impl_->ensure_ready("stop_polling");
        const std::lock_guard<std::timed_mutex> lifecycle(impl_->lifecycle_mutex_);
        const auto context = impl_->capture("stop_polling");
        auto lease = impl_->begin_io(context, "stop_polling");
        context.backend->stop_polling();
        const std::lock_guard<std::mutex> lock(impl_->mutex_);
        impl_->check_context(context, "stop_polling");
        impl_->polling_plan_.reset();
    }

    void HandDevice::recover() {
        impl_->assert_external_lifecycle();
        impl_->ensure_ready("recover");
        impl_->lifecycle(impl_, "recover", [owner = impl_] { owner->recover_backend(); });
    }

    void HandDevice::Impl::recover_backend() {
        auto *impl_ = this;
        impl_->retain_snapshot();
        const auto context = impl_->capture("recover");
        auto &selected = *context.backend;
        std::optional<PollingPlan> restore;
        DeviceIdentity expected_identity;
        {
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            impl_->check_context(context, "recover");
            restore = impl_->polling_plan_;
            expected_identity = ::linkerbot::hand::detail::checked_value(
                impl_->expected_identity_, "missing runtime state: impl_->expected_identity_");
            impl_->invalidate_commands("recover");
            ++impl_->generation_;
            impl_->state_.store(Impl::State::kRecovering, std::memory_order_release);
            impl_->open_.store(false, std::memory_order_release);
            impl_->identity_.reset();
            impl_->verified_capabilities_.reset();
            impl_->snapshot_sequence_ = 0;
            impl_->last_snapshot_time_.reset();
        }
        impl_->snapshot_cv_.notify_all();
        impl_->stop_stream();
        context.backend->cancel_commands();
        // Drain composite facade operations before native recovery clears its
        // caches. A captured old request must pass its epoch check after taking
        // a lease, so it cannot start sending in the recovered runtime.
        const std::unique_lock<std::shared_mutex> drain(impl_->io_mutex_);
        const auto recovery_generation = context.generation + 1;
        const auto previous_generation = selected.health().connection_generation;
        const auto declared = declared_capabilities(impl_->config_);

        std::exception_ptr last_error;
        for (uint32_t attempt = 0; attempt < impl_->config_.recovery_attempts; ++attempt) {
            if (attempt != 0 && impl_->config_.recovery_backoff.count() > 0) {
                std::this_thread::sleep_for(impl_->config_.recovery_backoff);
            }
            try {
                if (impl_->generation_.load(std::memory_order_acquire) != recovery_generation) {
                    throw ClosedError("recovery generation ended before attempt");
                }
                selected.recover();
                if (impl_->generation_.load(std::memory_order_acquire) != recovery_generation) {
                    throw ClosedError("recovery generation ended during reconnect");
                }
                if (selected.health().connection_generation <= previous_generation) {
                    throw ProtocolError("backend recovery did not advance the connection generation",
                                        {"recover", unified_detail::model_name(impl_->config_.model)});
                }
                auto initialized = unified_detail::initialize_connection(selected, impl_->config_, declared,
                                                                         &expected_identity, "recover");
                if (impl_->generation_.load(std::memory_order_acquire) != recovery_generation) {
                    throw ClosedError("recovery generation ended before baseline publication");
                }
                if (restore) {
                    selected.start_polling(*restore);
                }
                const std::lock_guard<std::mutex> lock(impl_->mutex_);
                if (impl_->generation_ != recovery_generation) {
                    throw ClosedError("recovery generation ended before Ready");
                }
                ++impl_->cache_generation_;
                impl_->cached_source_sequence_.reset();
                impl_->identity_ = std::move(initialized.identity);
                impl_->verified_capabilities_ = std::move(initialized.capabilities);
                impl_->state_.store(Impl::State::kReady, std::memory_order_release);
                impl_->open_.store(true, std::memory_order_release);
                impl_->snapshot_cv_.notify_all();
                return;
            } catch (const ClosedError &) {
                last_error = std::current_exception();
                break;
            } catch (const IdentityMismatchError &) {
                last_error = std::current_exception();
                break;
            } catch (...) {
                last_error = std::current_exception();
            }
            try {
                selected.stop_polling();
            } catch (...) {
                continue;
            }
        }
        const auto finish_recovery_failure = [&] {
            const std::lock_guard<std::mutex> lock(impl_->mutex_);
            if (impl_->generation_ == recovery_generation) {
                impl_->identity_.reset();
                impl_->verified_capabilities_.reset();
                impl_->polling_plan_.reset();
                impl_->snapshot_sequence_ = 0;
                impl_->last_snapshot_time_.reset();
                impl_->state_.store(Impl::State::kFaulted, std::memory_order_release);
            }
            impl_->snapshot_cv_.notify_all();
            impl_->stream_cv_.notify_all();
            std::rethrow_exception(last_error);
        };
        try {
            selected.stop_polling();
        } catch (...) {
            finish_recovery_failure();
        }
        finish_recovery_failure();
    }

    std::shared_ptr<EventQueue<HandEvent>> HandDevice::stream_handle(size_t max_size) {
        impl_->assert_external_lifecycle();
        impl_->ensure_ready("stream_handle");
        const std::lock_guard<std::timed_mutex> lifecycle(impl_->lifecycle_mutex_);
        impl_->stop_stream();
        const std::lock_guard<std::mutex> lock(impl_->mutex_);
        static_cast<void>(impl_->require_ready("stream_handle"));
        auto queue = std::make_shared<EventQueue<HandEvent>>(max_size);
        {
            const std::lock_guard<std::mutex> stream_lock(impl_->stream_mutex_);
            impl_->event_queue_ = queue;
            impl_->stream_stop_.store(false, std::memory_order_release);
            impl_->stream_thread_ = std::thread([impl = impl_.get(), queue] { impl->run_stream(queue); });
        }
        return queue;
    }

    void HandDevice::stop_stream() {
        impl_->assert_external_lifecycle();
        const std::lock_guard<std::timed_mutex> lifecycle(impl_->lifecycle_mutex_);
        impl_->stop_stream();
    }

} // namespace linkerbot::hand
