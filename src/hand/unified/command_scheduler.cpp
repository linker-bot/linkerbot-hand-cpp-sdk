#include "detail/command_scheduler.hpp"
#include "../detail/checked_optional.hpp"
#include "../detail/failure_kind.hpp"
#include "detail/command_handle_access.hpp"
#include <linkerbot/hand/runtime/device_runtime.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace linkerbot::hand::unified_detail {
    CommandFamily command_family(CommandSemantic semantic) {
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

    const char *command_family_name(CommandFamily family) {
        switch (family) {
        case CommandFamily::kAngleTarget:
            return "angle_targets";
        case CommandFamily::kSpeedSetting:
            return "speed_targets";
        case CommandFamily::kAccelerationSetting:
            return "acceleration_targets";
        case CommandFamily::kCurrentLimit:
            return "current_limits";
        case CommandFamily::kControlGain:
            return "control_gains";
        case CommandFamily::kTorqueLimit:
            return "torque_limits";
        case CommandFamily::kTorqueTarget:
            return "torque_targets";
        case CommandFamily::kMotionTime:
            return "motion_times";
        case CommandFamily::kLimitCompensation:
            return "limit_compensation";
        case CommandFamily::kStallConfiguration:
            return "stall";
        case CommandFamily::kDiscreteOperation:
            return "discrete_operations";
        }
        throw UnsupportedOperationError("unknown command family");
    }

    namespace {
        CommandScheduler::Clock::duration interval(double rate) {
            using Clock = CommandScheduler::Clock;
            const long double ticks = static_cast<long double>(Clock::period::den) / Clock::period::num / rate;
            if (!std::isfinite(rate) || rate <= 0 ||
                ticks >= static_cast<long double>(std::numeric_limits<Clock::rep>::max())) {
                throw ValidationError("command_rate_hz must have a representable positive interval");
            }
            return Clock::duration{static_cast<Clock::rep>(std::max(1.0L, std::ceil(ticks)))};
        }
        LatencyMetrics latency(const std::deque<HealthDuration> &window) {
            std::vector<HealthDuration> values(window.begin(), window.end());
            std::sort(values.begin(), values.end());
            LatencyMetrics result;
            result.count = values.size();
            if (!values.empty()) {
                result.p50 = values[((values.size() - 1) * 50 + 50) / 100];
                result.p95 = values[((values.size() - 1) * 95 + 50) / 100];
                result.p99 = values[((values.size() - 1) * 99 + 50) / 100];
            }
            return result;
        }
        void sample(std::deque<HealthDuration> &window, HealthDuration value) {
            window.push_back(value);
            if (window.size() > kHealthLatencyWindowCapacity) {
                window.pop_front();
            }
        }
    }

    CommandScheduler::CommandScheduler(double rate_hz, size_t capacity, Now now, bool python_behavior)
        : CommandScheduler(ConstructorTag{}, ConstructorArguments{rate_hz, capacity, std::move(now), python_behavior}) {
    }

    CommandScheduler::CommandScheduler(ConstructorTag, ConstructorArguments arguments)
        : interval_(interval(arguments.rate_hz)), capacity_(arguments.capacity),
          manual_clock_(static_cast<bool>(arguments.now)), python_behavior_(arguments.python_behavior),
          now_(arguments.now ? std::move(arguments.now) : Now{[] { return Clock::now(); }}) {
        if (!arguments.capacity) {
            throw ValidationError("discrete_queue_capacity must be positive");
        }
        worker_ = std::thread([this] { run(); });
        try {
            watcher_ = std::thread([this] { watch_deadlines(); });
        } catch (...) {
            {
                const std::lock_guard<std::mutex> lock(mutex_);
                stop_ = true;
            }
            changed_.notify_all();
            worker_.join();
            throw;
        }
    }

    CommandScheduler::~CommandScheduler() {
        invalidate(std::make_exception_ptr(ClosedError("command scheduler closed")));
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        changed_.notify_all();
        worker_.join();
        watcher_.join();
    }

    bool CommandScheduler::latest(CommandFamily family) {
        return family == CommandFamily::kAngleTarget || family == CommandFamily::kSpeedSetting ||
               family == CommandFamily::kCurrentLimit || family == CommandFamily::kTorqueLimit;
    }

    size_t CommandScheduler::pending_size() const { return fifo_.size() + latest_.size(); }

    CommandHandle CommandScheduler::accept(CommandRequest request, AckSupport support, ErrorContext context,
                                           Execute execute) {
        const auto family = request.operation ? CommandFamily::kDiscreteOperation : command_family(request.semantic);
        std::unique_lock<std::mutex> lock(mutex_);
        const auto now = now_();
        expire(now);
        if (stop_) {
            throw DeviceNotReadyError("command scheduler is stopping", context);
        }
        if (!latest(family) && fifo_.size() >= capacity_) {
            throw QueueFullError("HandDevice FIFO pending queue is full", context);
        }
        if (next_id_ == std::numeric_limits<uint64_t>::max()) {
            throw InternalError("command sequence exhausted", context);
        }
        CommandHandle handle(next_id_, family, support, request.ack_expectation, now);
        Job job{std::move(request), handle, std::move(context), now, {}, std::move(execute)};
        job.deadline = python_behavior_ ? Clock::time_point::max() : now + job.request.timeout;
        job.request.deadline = job.deadline;
        job.request.completion = handle;
        if (python_behavior_) {
            job.request.delivery = std::make_shared<CommandDelivery>();
        }
        job.request.cancellation = job.cancellation.token();
        job.request.dispatched = [this, handle, family](uint64_t frames, Clock::time_point timestamp) mutable {
            const std::lock_guard<std::mutex> lock(mutex_);
            if (handle.done()) {
                return;
            }
            command_detail::CommandHandleAccess::mark_dispatched(handle, frames, timestamp);
            auto &m = metrics_.at(family).value;
            ++m.dispatched;
            if (handle.done()) {
                --m.pending;
            }
        };
        job.request.acknowledged = [this, handle, family]() mutable {
            const std::lock_guard<std::mutex> lock(mutex_);
            if (handle.done()) {
                return;
            }
            command_detail::CommandHandleAccess::mark_acknowledged(handle);
            auto &m = metrics_.at(family).value;
            ++m.acknowledged;
            --m.pending;
        };
        auto &metrics = metrics_[family];
        metrics.value.family = command_family_name(family);
        if (latest(family)) {
            auto old = latest_.find(family);
            if (old != latest_.end()) {
                // Construct the replacement completely before touching the old slot.
                command_detail::CommandHandleAccess::mark_superseded(old->second.handle, now);
                removed(old->second, true);
                ++metrics.value.superseded;
                old->second = std::move(job);
            } else {
                latest_.emplace(family, std::move(job));
            }
        } else {
            fifo_.push_back(std::move(job));
        }
        ++next_id_;
        ++metrics.value.accepted;
        ++metrics.value.pending;
        ++metrics.value.queue.accepted;
        ++metrics.value.queue.depth;
        metrics.value.queue.high_water = std::max(metrics.value.queue.high_water, metrics.value.queue.depth);
        lock.unlock();
        changed_.notify_all();
        return handle;
    }

    void CommandScheduler::removed(Job &job, bool dropped) {
        auto &m = metrics_.at(job.handle.family()).value;
        --m.queue.depth;
        if (dropped) {
            --m.pending;
            ++m.queue.dropped;
        }
    }

    void CommandScheduler::fail(Job &job, std::exception_ptr error) {
        if (job.handle.done()) {
            return;
        }
        if (detail::failure_kind(error) == detail::FailureKind::closed) {
            error = std::make_exception_ptr(ClosedError("accepted command generation ended", job.context, error));
        }
        command_detail::CommandHandleAccess::mark_failed(job.handle, error, job.handle.sent_frames(), now_());
        job.cancellation.cancel();
        auto &m = metrics_.at(job.handle.family()).value;
        ++m.failed;
        if (detail::failure_kind(error) == detail::FailureKind::timeout) {
            ++m.timed_out;
        }
    }

    void CommandScheduler::expire(Clock::time_point now) {
        const auto expired = [&](Job &job) {
            if (now < job.deadline) {
                return false;
            }
            fail(job, std::make_exception_ptr(TimeoutError("command expired in pending queue", job.context)));
            removed(job, true);
            return true;
        };
        for (auto it = fifo_.begin(); it != fifo_.end();) {
            if (expired(*it)) {
                it = fifo_.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = latest_.begin(); it != latest_.end();) {
            if (expired(it->second)) {
                it = latest_.erase(it);
            } else {
                ++it;
            }
        }
    }

    CommandScheduler::Clock::time_point CommandScheduler::next_deadline() const {
        auto deadline = Clock::time_point::max();
        for (const auto &job : fifo_) {
            deadline = std::min(deadline, job.deadline);
        }
        for (const auto &entry : latest_) {
            deadline = std::min(deadline, entry.second.deadline);
        }
        return deadline;
    }

    void CommandScheduler::invalidate(std::exception_ptr error) {
        const std::lock_guard<std::mutex> lock(mutex_);
        ++invalidation_;
        if (preparing_) {
            preparing_->cancel();
        }
        for (auto &job : fifo_) {
            fail(job, error);
            removed(job, true);
        }
        for (auto &entry : latest_) {
            fail(entry.second, error);
            removed(entry.second, true);
        }
        fifo_.clear();
        latest_.clear();
        if (active_ && !active_->handle.done()) {
            const bool preserved =
                python_behavior_ && active_->request.delivery &&
                active_->request.delivery->preserve_claimed(active_->request.ack_expectation == AckExpectation::kNone);
            if (!preserved) {
                fail(*active_, error);
                --metrics_.at(active_->handle.family()).value.pending;
            }
        }
        changed_.notify_all();
    }

    void CommandScheduler::watch_deadlines() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stop_) {
            expire(now_());
            const auto deadline = next_deadline();
            if (manual_clock_ || deadline == Clock::time_point::max()) {
                changed_.wait(lock);
            } else {
                changed_.wait_until(lock, deadline);
            }
        }
    }

    void CommandScheduler::run() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stop_) {
            auto now = now_();
            expire(now);
            if (!pending_size()) {
                changed_.wait(lock);
                continue;
            }
            if (now < next_start_) {
                if (manual_clock_) {
                    changed_.wait(lock);
                } else {
                    changed_.wait_until(lock, next_start_);
                }
                continue;
            }
            // Claim a target only after the device transaction lane is available.
            // Until then latest replacement and FIFO capacity still apply to it.
            std::shared_ptr<void> execution_lease;
            if (python_behavior_) {
                const auto &candidate = fifo_.empty() ? latest_.begin()->second : fifo_.front();
                const auto prepare = candidate.request.prepare_execution;
                if (prepare) {
                    const auto epoch = invalidation_;
                    preparing_.emplace();
                    const auto cancellation = preparing_->token();
                    lock.unlock();
                    std::exception_ptr error;
                    try {
                        execution_lease = prepare(cancellation);
                    } catch (...) {
                        error = std::current_exception();
                    }
                    lock.lock();
                    preparing_.reset();
                    if (epoch != invalidation_ || stop_ || !pending_size()) {
                        continue;
                    }
                    if (error) {
                        for (auto &queued : fifo_) {
                            fail(queued, error);
                            removed(queued, true);
                        }
                        for (auto &[family, queued] : latest_) {
                            (void)family;
                            fail(queued, error);
                            removed(queued, true);
                        }
                        fifo_.clear();
                        latest_.clear();
                        continue;
                    }
                    now = now_();
                }
            }
            auto selected = latest_.end();
            auto sequence = fifo_.empty() ? std::numeric_limits<uint64_t>::max() : fifo_.front().handle.command_id();
            for (auto it = latest_.begin(); it != latest_.end(); ++it) {
                if (it->second.handle.command_id() < sequence) {
                    selected = it;
                    sequence = it->second.handle.command_id();
                }
            }
            active_.emplace(selected != latest_.end() ? std::move(selected->second) : std::move(fifo_.front()));
            auto &job = ::linkerbot::hand::detail::checked_value(active_, "missing runtime state: active_");
            if (selected != latest_.end()) {
                latest_.erase(selected);
            } else {
                fifo_.pop_front();
            }
            job.request.execution_lease = std::move(execution_lease);
            removed(job, false);
            auto &m = metrics_.at(job.handle.family());
            sample(m.queue_times, now - job.accepted);
            next_start_ = now + interval_;
            lock.unlock();
            std::optional<CommandHandle> result;
            std::exception_ptr error;
            try {
                // The absolute admission deadline also travels with the request;
                // this rounded duration is only for native API compatibility.
                if (!python_behavior_) {
                    job.request.timeout = runtime::remaining(job.deadline);
                }
                result = job.execute(job.request);
            } catch (...) {
                error = classify_exception(std::current_exception(), job.context, ErrorCode::kInternal);
            }
            lock.lock();
            // Leave a full interval after completion as well. Arbitration may
            // delay the physical start; this prevents catch-up bursts thereafter.
            if (!python_behavior_) {
                next_start_ = now_() + interval_;
            }
            sample(m.execution_times, now_() - now);
            if (!job.handle.done()) {
                --m.value.pending;
            }
            // A no-ACK receipt is terminal. Optional feedback can subsequently
            // fail, but cannot change successful delivery into another terminal.
            if (error && !job.handle.done()) {
                fail(job, error);
            } else {
                // Internal scheduler tests/adapters may return a distinct handle.
                // Production backend updates the accepted handle from receipts.
                if (result && !job.handle.done() && job.handle.stage() == CommandStage::kAccepted) {
                    command_detail::CommandHandleAccess::mark_dispatched(job.handle, result->sent_frames());
                    ++m.value.dispatched;
                }
                if (result && result->stage() == CommandStage::kAcknowledged) {
                    if (!job.handle.done()) {
                        command_detail::CommandHandleAccess::mark_acknowledged(job.handle);
                        ++m.value.acknowledged;
                    }
                }
            }
            active_.reset();
            changed_.notify_all();
        }
    }

    std::vector<CommandFamilyMetrics> CommandScheduler::metrics() const {
        const std::lock_guard<std::mutex> lock(mutex_);
        std::vector<CommandFamilyMetrics> result;
        for (const auto &entry : metrics_) {
            auto value = entry.second.value;
            value.queue_latency = latency(entry.second.queue_times);
            value.execution_latency = latency(entry.second.execution_times);
            result.push_back(std::move(value));
        }
        return result;
    }

    void CommandScheduler::notify_clock_advanced() {
        const std::lock_guard<std::mutex> lock(mutex_);
        expire(now_());
        changed_.notify_all();
    }
}
