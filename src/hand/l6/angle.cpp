#include "linkerbot/hand/l6/angle.hpp"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/l6/angle_manager.hpp>
#include <linkerbot/hand/l6/frame.hpp>
#include <linkerbot/hand/l6/joints.hpp>
#include <linkerbot/hand/l6/protocol_backend.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace linkerbot::hand::l6 {
    namespace {
        std::vector<std::string> joint_name_list() {
            std::vector<std::string> names;
            names.reserve(kJointNames.size());
            for (auto name : kJointNames) {
                names.emplace_back(name);
            }
            return names;
        }

        std::array<uint8_t, Angle::kJointCount> angles_to_raw(const Angle &angles) {
            return percents_to_raw(angles.to_array());
        }
        // 把原始数值（0～255），转换成百分比（0～100）对象Angle
        Angle angles_from_raw(const std::array<uint8_t, Angle::kJointCount> &raw) {
            return Angle::from_array(raw_to_percents(raw));
        }
        float to_seconds(std::chrono::milliseconds timeout_ms) {
            if (timeout_ms.count() <= 0) {
                throw ValidationError("timeout_ms must be positive");
            }
            return std::chrono::duration_cast<std::chrono::duration<float>>(timeout_ms).count();
        }
    } // namespace

    // 1.提供CANDispatcher -- 委托构造函数
    AngleManager::AngleManager(uint32_t id, can::CANDispatcher &dispatcher,
                               const std::optional<std::filesystem::path> &path, std::string_view side,
                               std::string_view interface_name, std::shared_ptr<std::timed_mutex> transaction_gate)
        : AngleManager(id, detail::borrow_runtime(dispatcher, id), path, side, interface_name) {
        external_gate_ = std::move(transaction_gate);
    }

    //  2.直接提供 DeviceRuntime
    AngleManager::AngleManager(uint32_t id, std::shared_ptr<runtime::DeviceRuntime> core,
                               const std::optional<std::filesystem::path> &path, std::string_view side,
                               std::string_view interface_name)
        : arbitration_id_(id), mapping_("l6", joint_name_list(), path, std::string(side), std::string(interface_name)),
          channel_(std::move(core), "angle") {}
    AngleManager::~AngleManager() {
        try {
            stop();
            // Destruction is nonthrowing; explicit close/stop remains the error-reporting path.
            // NOLINTNEXTLINE(bugprone-empty-catch)
        } catch (...) {
        }
        channel_.drain_calls();
    }
    void AngleManager::start() { channel_.start(); }
    void AngleManager::stop() { channel_.stop(); }
    void AngleManager::set_angles(const std::array<uint8_t, 6> &raw) {
        const auto call = channel_.enter();
        set_angles_impl(raw, false);
    }
    void AngleManager::set_angles_percent(const Angle &value) {
        const auto call = channel_.enter();
        value.validate();
        set_angles_impl(angles_to_raw(value), true);
    }
    void AngleManager::set_angles_percent(const std::array<float, 6> &value) {
        const auto call = channel_.enter();
        set_angles_percent(Angle::from_array(value));
    }
    std::array<uint8_t, 6> AngleManager::mapped(const std::array<uint8_t, 6> &raw, bool apply) const {
        if (!apply) {
            return raw;
        }
        const auto values = mapping_.map_values({raw.begin(), raw.end()});
        std::array<uint8_t, 6> out{};
        std::copy(values.begin(), values.end(), out.begin());
        return out;
    }
    void AngleManager::set_angles_impl(const std::array<uint8_t, 6> &raw, bool apply) {
        const auto call = channel_.enter();
        std::optional<bool> reply;
        std::chrono::milliseconds timeout, guard;
        {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            reply = reply_enabled_;
            timeout = command_timeout_;
            guard = late_response_guard_;
        }
        const auto deadline = runtime::deadline_after(timeout);
        auto plan = protocol::angle(arbitration_id_, mapped(raw, apply), reply.value_or(true), guard,
                                    reply ? std::optional<std::chrono::microseconds>{}
                                          : std::optional<std::chrono::microseconds>{guard});
        (void)transact(std::move(plan), deadline);
    }
    AngleData AngleManager::set_angles_blocking(const std::array<uint8_t, 6> &raw, std::chrono::milliseconds timeout) {
        const auto call = channel_.enter();
        return set_angles_blocking_impl(raw, timeout, false);
    }
    AngleData AngleManager::set_angles_percent_blocking(const Angle &angles, std::chrono::milliseconds timeout) {
        const auto call = channel_.enter();
        angles.validate();
        return set_angles_blocking_impl(angles_to_raw(angles), timeout, true);
    }
    AngleData AngleManager::set_angles_blocking_impl(const std::array<uint8_t, 6> &raw,
                                                     std::chrono::milliseconds timeout, bool apply) {
        const auto call = channel_.enter();
        const auto deadline = runtime::deadline_after(timeout);
        std::chrono::milliseconds guard;
        {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            if (reply_enabled_ && !*reply_enabled_) {
                throw DeviceNotReadyError("L6 angle command feedback requires the 0xCC reply switch to be enabled");
            }
            guard = late_response_guard_;
        }
        return detail::Channel<AngleData>::project(
            transact(protocol::angle(arbitration_id_, mapped(raw, apply), true, guard), deadline));
    }
    AngleMapping AngleManager::get_angle_mapping() const { return mapping_.get_mapping(); }

    AngleMapping AngleManager::get_default_angle_mapping() const { return mapping_.get_default_mapping(); }

    void AngleManager::set_angle_mapping(const AngleMapping &mapping) { mapping_.set_mapping(mapping); }

    void AngleManager::reset_angle_mapping() { mapping_.reset_mapping(); }

    AngleData AngleManager::get_blocking(std::chrono::milliseconds timeout) {
        const auto call = channel_.enter();
        return get_until(runtime::deadline_after(timeout));
    }
    AngleData AngleManager::get_until(runtime::Deadline deadline) {
        std::chrono::milliseconds guard;
        {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            guard = late_response_guard_;
        }
        return detail::Channel<AngleData>::project(
            transact(protocol::angle(arbitration_id_, {}, true, guard), deadline));
    }
    std::optional<AngleData> AngleManager::get_snapshot() const { return channel_.snapshot(); }
    std::optional<std::chrono::steady_clock::time_point> AngleManager::last_command_feedback_time() const {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        return last_command_feedback_;
    }
    size_t AngleManager::pending_count() const { return channel_.pending_count(); }
    uint64_t AngleManager::in_flight_count() const { return channel_.pending_count(); }
    void AngleManager::set_event_sink(std::function<void(const AngleData &)> sink) {
        channel_.set_sink(std::move(sink));
    }
    void AngleManager::attach_stats(RuntimeStats *stats) {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        stats_ = stats;
    }
    void AngleManager::set_reply_enabled(std::optional<bool> enabled) {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        reply_enabled_ = enabled;
    }
    void AngleManager::set_late_response_guard(std::chrono::milliseconds guard) {
        if (guard.count() < 0) {
            throw ValidationError("late_response_guard must be non-negative");
        }
        std::lock_guard<std::mutex> lock(tx_mutex_);
        late_response_guard_ = guard;
    }
    void AngleManager::set_command_timeout(std::chrono::milliseconds timeout) {
        if (timeout.count() <= 0) {
            throw ValidationError("command timeout must be positive");
        }
        std::lock_guard<std::mutex> lock(tx_mutex_);
        command_timeout_ = timeout;
    }
    void AngleManager::set_command_feedback_listener(std::function<void()> listener) {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        command_feedback_listener_ = std::move(listener);
    }
    void AngleManager::cancel_waiters() { channel_.cancel(); }
    void AngleManager::resume_transactions() { channel_.start(); }
    runtime::Sample AngleManager::transact(runtime::TransactionPlan plan, runtime::Deadline deadline) {
        const auto call = channel_.enter();
        RuntimeStats *stats;
        {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            stats = stats_;
        }
        try {
            std::unique_lock<std::timed_mutex> external;
            if (external_gate_) {
                external = channel_.core()->lock_external(*external_gate_, deadline, channel_.token());
            }
            auto request = channel_.core()->submit(std::move(plan), deadline, channel_.token());
            const auto sample = request.wait_until(deadline);
            if (sample.valid && sample.value.has_value()) {
                const auto data = detail::Channel<AngleData>::project(sample);
                if (stats) {
                    stats->inc_completed();
                    stats->inc_angle_completed();
                    stats->record_angle_latency(data.latency);
                    if (data.origin == SampleOrigin::Query) {
                        stats->inc_query_angle();
                    } else if (data.origin == SampleOrigin::CommandFeedback) {
                        stats->inc_command_feedback_angle();
                    } else {
                        stats->inc_unknown_origin();
                    }
                }
                std::function<void()> listener;
                if (data.origin == SampleOrigin::CommandFeedback) {
                    std::lock_guard<std::mutex> lock(tx_mutex_);
                    last_command_feedback_ = sample.received_at;
                    listener = command_feedback_listener_;
                }
                if (listener) {
                    listener();
                }
            }
            return sample;
        } catch (const ::linkerbot::hand::TimeoutError &error) {
            if (stats) {
                stats->inc_timeout();
                stats->inc_angle_timeout();
            }
            if (dynamic_cast<const ::linkerbot::hand::ResponseTimeoutError *>(&error)) {
                throw;
            }
            throw internal::DataRelayTimeout(error.what(), error.context(), std::current_exception());
        }
    }
    void AngleManager::send_sense_request() {
        const auto call = channel_.enter();
        std::chrono::milliseconds timeout;
        {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            timeout = command_timeout_;
        }
        (void)get_blocking(timeout);
    }
} // namespace linkerbot::hand::l6
