#include <array>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/l6/force_sensor_manager.hpp>
#include <linkerbot/hand/l6/protocol_backend.hpp>
#include <stdexcept>

namespace linkerbot::hand::l6 {
    namespace {
        struct Finger {
            const char *name;
            uint8_t opcode;
        };
        constexpr std::array<Finger, 5> fingers{
            {{"thumb", 0xb1}, {"index", 0xb2}, {"middle", 0xb3}, {"ring", 0xb4}, {"pinky", 0xb5}}};
    }
    SingleForceSensorManager::SingleForceSensorManager(uint32_t id, can::CANDispatcher &dispatcher, uint8_t opcode,
                                                       std::shared_ptr<std::timed_mutex> transaction_gate)
        : SingleForceSensorManager(id, detail::borrow_runtime(dispatcher, id), opcode) {
        external_gate_ = std::move(transaction_gate);
    }
    SingleForceSensorManager::SingleForceSensorManager(uint32_t id, std::shared_ptr<runtime::DeviceRuntime> core,
                                                       uint8_t opcode)
        : arbitration_id_(id), command_prefix_(opcode), channel_(std::move(core), protocol::force_domain(opcode)) {}
    SingleForceSensorManager::~SingleForceSensorManager() {
        try {
            stop();
            // Destruction is nonthrowing; explicit close/stop remains the error-reporting path.
            // NOLINTNEXTLINE(bugprone-empty-catch)
        } catch (...) {
        }
        channel_.drain_calls();
    }
    void SingleForceSensorManager::start() { channel_.start(); }
    void SingleForceSensorManager::stop() { channel_.stop(); }
    ForceSensorData SingleForceSensorManager::get_blocking(std::chrono::milliseconds timeout) {
        const auto call = channel_.enter();
        return get_until(runtime::deadline_after(timeout));
    }
    ForceSensorData SingleForceSensorManager::get_until(runtime::Deadline deadline, runtime::Sample *provenance) {
        const auto call = channel_.enter();
        auto *stats = stats_.load();
        try {
            std::unique_lock<std::timed_mutex> external;
            if (external_gate_) {
                external = channel_.core()->lock_external(*external_gate_, deadline, channel_.token());
            }
            auto plan = protocol::read_force(arbitration_id_, command_prefix_,
                                             std::chrono::milliseconds(late_guard_ms_.load()));
            auto request = channel_.core()->submit(std::move(plan), deadline, channel_.token());
            auto sample = request.wait_until(deadline);
            if (provenance) {
                *provenance = sample;
            }
            return detail::Channel<ForceSensorData>::project(sample);
        } catch (const ::linkerbot::hand::TimeoutError &error) {
            if (stats) {
                stats->inc_timeout();
                stats->inc_tactile_timeout(static_cast<size_t>(command_prefix_ - 0xB1));
            }
            if (dynamic_cast<const ::linkerbot::hand::ResponseTimeoutError *>(&error)) {
                throw;
            }
            throw internal::DataRelayTimeout(error.what());
        }
    }
    std::optional<ForceSensorData> SingleForceSensorManager::get_snapshot() const { return channel_.snapshot(); }
    size_t SingleForceSensorManager::pending_count() const { return channel_.pending_count(); }
    uint64_t SingleForceSensorManager::in_flight_count() const { return channel_.pending_count(); }
    void SingleForceSensorManager::set_event_sink(std::function<void(const ForceSensorData &)> sink) {
        channel_.set_sink(std::move(sink));
    }
    void SingleForceSensorManager::attach_stats(RuntimeStats *stats) { stats_.store(stats); }
    void SingleForceSensorManager::set_late_response_guard(std::chrono::milliseconds guard) {
        if (guard.count() < 0) {
            throw ValidationError("late_response_guard must be non-negative");
        }
        late_guard_ms_.store(guard.count());
    }
    void SingleForceSensorManager::cancel_waiters() { channel_.cancel(); }
    void SingleForceSensorManager::resume_transactions() { channel_.start(); }

    ForceSensorManager::ForceSensorManager(uint32_t id, can::CANDispatcher &dispatcher,
                                           const std::shared_ptr<std::timed_mutex> &transaction_gate)
        : ForceSensorManager(id, detail::borrow_runtime(dispatcher, id)) {
        for (auto &[name, finger] : fingers_) {
            (void)name;
            finger->external_gate_ = transaction_gate;
        }
    }
    ForceSensorManager::ForceSensorManager(uint32_t id, std::shared_ptr<runtime::DeviceRuntime> core)
        : channel_(core, "tactile") {
        for (const auto &finger : fingers) {
            fingers_.emplace(finger.name, std::make_unique<SingleForceSensorManager>(id, core, finger.opcode));
        }
    }
    ForceSensorManager::~ForceSensorManager() {
        try {
            stop();
            // Destruction is nonthrowing; explicit close/stop remains the error-reporting path.
            // NOLINTNEXTLINE(bugprone-empty-catch)
        } catch (...) {
        }
        channel_.drain_calls();
        for (auto &[name, finger] : fingers_) {
            (void)name;
            finger->channel_.drain_calls();
        }
    }
    void ForceSensorManager::start() {
        channel_.start();
        for (auto &[name, sensor] : fingers_) {
            (void)name;
            sensor->start();
        }
    }
    void ForceSensorManager::stop() {
        cancel_waiters();
        channel_.stop();
        for (auto &[name, sensor] : fingers_) {
            (void)name;
            sensor->stop();
        }
    }
    AllFingersData ForceSensorManager::get_blocking(std::chrono::milliseconds timeout) {
        const auto call = channel_.enter();
        return get_staged(runtime::deadline_after(timeout), {});
    }
    AllFingersData ForceSensorManager::get_staged(runtime::Deadline deadline, const BeforeFinger &before_finger) {
        const auto call = channel_.enter();
        auto core = channel_.core();
        auto token = channel_.token();
        auto operation = [&] {
            try {
                return core->operation("tactile.batch", deadline, token);
            } catch (const ::linkerbot::hand::TimeoutError &error) {
                if (dynamic_cast<const ::linkerbot::hand::ResponseTimeoutError *>(&error)) {
                    throw;
                }
                throw internal::DataRelayTimeout(error.what());
            }
        }();
        const auto started = runtime::Clock::now();
        std::array<ForceSensorData, 5> values{};
        runtime::Sample provenance;
        try {
            for (size_t i = 0; i < fingers.size(); ++i) {
                token.throw_if_cancelled();
                if (before_finger) {
                    before_finger(i, deadline);
                }
                token.throw_if_cancelled();
                values[i] = fingers_.at(fingers[i].name)->get_until(deadline, &provenance);
                if (stats_) {
                    stats_->record_finger_latency(i, values[i].latency);
                }
            }
            token.throw_if_cancelled();
            AllFingersData value{values[0], values[1], values[2], values[3], values[4], provenance.received_at};
            provenance.started_at = started;
            auto sample = core->publish("tactile", std::move(value), provenance);
            auto data = detail::Channel<AllFingersData>::project(sample);
            if (stats_) {
                stats_->inc_completed();
                stats_->inc_tactile_completed();
                stats_->record_tactile_latency(data.latency);
            }
            return data;
        } catch (const transport::CancelledError &) {
            ErrorContext context;
            context.operation = "force_sensor.get_staged";
            context.model = "L6";
            context.channel = "tactile";
            context.generation = core->runtime_health().device_generation;
            rethrow_as_hand_error(std::current_exception(), std::move(context), ErrorCode::kLifecycle);
        }
    }
    std::optional<AllFingersData> ForceSensorManager::get_snapshot() const { return channel_.snapshot(); }
    size_t ForceSensorManager::pending_count() const {
        size_t pending = 0;
        for (const auto &[name, sensor] : fingers_) {
            (void)name;
            pending += sensor->pending_count();
        }
        return pending;
    }
    uint64_t ForceSensorManager::in_flight_count() const { return pending_count() ? 1 : 0; }
    SingleForceSensorManager &ForceSensorManager::get_finger(const std::string &name) {
        auto found = fingers_.find(name);
        if (found == fingers_.end()) {
            throw ValidationError("Unknown finger: " + name);
        }
        return *found->second;
    }
    void ForceSensorManager::set_event_sink(std::function<void(const AllFingersData &)> sink) {
        channel_.set_sink(std::move(sink));
    }
    void ForceSensorManager::attach_stats(RuntimeStats *stats) {
        stats_ = stats;
        for (auto &[name, sensor] : fingers_) {
            (void)name;
            sensor->attach_stats(stats);
        }
    }
    void ForceSensorManager::set_late_response_guard(std::chrono::milliseconds guard) {
        for (auto &[name, sensor] : fingers_) {
            (void)name;
            sensor->set_late_response_guard(guard);
        }
    }
    void ForceSensorManager::cancel_waiters() {
        channel_.cancel();
        for (auto &[name, sensor] : fingers_) {
            (void)name;
            sensor->cancel_waiters();
        }
    }
    void ForceSensorManager::resume_transactions() {
        channel_.start();
        for (auto &[name, sensor] : fingers_) {
            (void)name;
            sensor->resume_transactions();
        }
    }
} // namespace linkerbot::hand::l6
