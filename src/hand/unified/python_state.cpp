#include "detail/python_state.hpp"
#include "detail/cached_snapshot.hpp"
#include "detail/force.hpp"
#include "detail/freshness.hpp"
#include "detail/python_transactions.hpp"
#include <algorithm>
#include <linkerbot/hand/o30/v25.hpp>

namespace linkerbot::hand::unified_detail {
    namespace {
        using S = TelemetrySemantic;
        bool force_scalar(S semantic) {
            return semantic == S::kForceFns || semantic == S::kForceFts || semantic == S::kForceFds ||
                   semantic == S::kForcePx;
        }
        bool same_plan(const PollingPlan &left, const PollingPlan &right) {
            if (left.size() != right.size()) {
                return false;
            }
            return std::all_of(left.begin(), left.end(), [&](const auto &entry) {
                return std::any_of(right.begin(), right.end(), [&](const auto &other) {
                    return entry.semantic == other.semantic && entry.interval == other.interval &&
                           entry.angle_feedback == other.angle_feedback;
                });
            });
        }
        std::string health_name(S semantic) {
            switch (semantic) {
            case S::kAngles:
                return "angles";
            case S::kMeasuredSpeeds:
                return "speeds";
            case S::kCurrents:
                return "currents";
            case S::kTemperatures:
                return "temperatures";
            case S::kFaults:
                return "faults";
            case S::kTorques:
                return "torques";
            case S::kVoltages:
                return "voltages";
            default:
                return python_polling_domain(semantic);
            }
        }
    }
    PythonState::PythonState(const HandDeviceConfig &config) : config_(config) {
        const auto caps = declared_capabilities(config);
        for (const auto &channel : caps.telemetry) {
            domains_.emplace(domain(channel.semantic), channel.semantic);
        }
    }
    std::string PythonState::domain(S semantic) const {
        if (config_.model == Model::kO30 && semantic != S::kForce && semantic != S::kForceFns) {
            return o30::v25::domain(o30::v25::object(semantic));
        }
        if (config_.model == Model::kL20Lite && force_scalar(semantic)) {
            return "python/" + std::string(python_polling_domain(semantic));
        }
        std::string result = python_polling_domain(semantic);
        if (semantic == S::kConfiguredSpeeds) {
            result = config_.model == Model::kL20Lite || config_.model == Model::kL30 ? "configured_speed" : "speed";
        } else if (semantic == S::kForce) {
            result = config_.model == Model::kL6 ? "tactile" : "force_sensor_all";
        }
        if (config_.model == Model::kL30) {
            result = "l30." + (semantic == S::kForce ? std::string("force_sensor") : result);
        }
        if (config_.model == Model::kO30 && (semantic == S::kForce || semantic == S::kForceFns)) {
            result = "force";
        }
        return result;
    }
    void PythonState::visible_locked(bool command_feedback) {
        ++current_.sequence;
        if (!command_feedback) {
            non_command_ = current_;
        }
        changed_.notify_all();
    }
    void PythonState::publish(const std::string &name, const runtime::Sample &sample) {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (stopped_) {
            return;
        }
        const auto found = domains_.find(name);
        const bool identity = name == "python/identity";
        if (!sample.valid) {
            // Internal optional-feedback timeout notification. Keep the last
            // successful acquisition and count only the active angle policy.
            if (found != domains_.end() && found->second == S::kAngles) {
                if (auto poll = polls_.find(S::kAngles); poll != polls_.end()) {
                    ++poll->second.timeouts;
                }
            }
            return;
        }
        // Native domains are SDK-owned and protocol-bounded, not user queues.
        current_.samples[name] = sample;
        if (found == domains_.end() && !identity) {
            return;
        }
        if (identity) {
            auto value = std::any_cast<DeviceIdentity>(sample.value);
            value.meta.sequence = ++identity_sequence_;
            current_.samples[name].value = std::move(value);
        }
        const bool command = sample.origin == protocol::Origin::command_feedback;
        if (found != domains_.end()) {
            const auto semantic = found->second;
            auto &meta = current_.metadata[semantic];
            meta.sequence = ++sequences_[semantic];
            meta.collected_at = sample.received_at;
            meta.connection_generation = sample.device_generation;
            meta.valid = true;
            meta.stale = false;
            meta.delivery = command                                     ? TelemetryDelivery::kCommandResponse
                            : sample.origin == protocol::Origin::report ? TelemetryDelivery::kPeriodicReport
                                                                        : TelemetryDelivery::kQueryResponse;
            if (auto poll = polls_.find(semantic); poll != polls_.end() && poll->second.enabled) {
                auto &state = poll->second;
                if (command && semantic == S::kAngles) {
                    ++state.commands;
                    if (state.channel.angle_feedback != AngleFeedbackPolicy::kQueryOnly) {
                        ++state.feedback;
                        state.next = std::max(state.next, runtime::Clock::now() + state.channel.interval);
                    }
                } else if (!command && !(semantic == S::kAngles &&
                                         state.channel.angle_feedback == AngleFeedbackPolicy::kCommandFeedbackOnly)) {
                    ++state.feedback;
                }
            }
        }
        visible_locked(command);
    }
    PythonState::View PythonState::view(bool include_feedback) const {
        const std::lock_guard<std::mutex> lock(mutex_);
        return include_feedback ? current_ : non_command_;
    }
    void PythonState::wait(uint64_t sequence, runtime::Deadline deadline, bool include_feedback,
                           const std::function<bool()> &generation_valid) const {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait_until(lock, deadline, [&] {
            return stopped_ || (generation_valid && !generation_valid()) ||
                   (include_feedback ? current_.sequence != sequence : non_command_.sequence > sequence);
        });
    }
    void PythonState::stop() {
        const std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
        changed_.notify_all();
    }
    void PythonState::reset() {
        const std::lock_guard<std::mutex> lock(mutex_);
        current_ = {};
        non_command_ = {};
        sequences_.clear();
        verified_.clear();
        failures_.clear();
        disabled_.clear();
        polls_.clear();
        plan_.reset();
        identity_sequence_ = 0;
        stopped_ = false;
        ready_ = false;
        changed_.notify_all();
    }
    void PythonState::resume() {
        const std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = false;
    }
    void PythonState::initialized() {
        const std::lock_guard<std::mutex> lock(mutex_);
        ready_ = true;
    }
    bool PythonState::optional(S semantic) const {
        if (semantic == S::kForce) {
            return config_.model != Model::kL30;
        }
        if (config_.model == Model::kL6) {
            return semantic == S::kTorques || semantic == S::kPositionKp || semantic == S::kPositionKd ||
                   semantic == S::kTorqueKd || semantic == S::kTemperatures || semantic == S::kCurrents ||
                   semantic == S::kLimitCompensations || semantic == S::kStallDetectionTimes ||
                   semantic == S::kStallThresholds || semantic == S::kStallHoldCurrents;
        }
        if (config_.model == Model::kO6) {
            return semantic == S::kConfiguredSpeeds || semantic == S::kConfiguredAccelerations ||
                   semantic == S::kTemperatures || semantic == S::kStallDetectionTimes ||
                   semantic == S::kStallThresholds || semantic == S::kStallHoldCurrents;
        }
        if (config_.model == Model::kL20Lite || config_.model == Model::kL20) {
            return semantic == S::kMeasuredSpeeds || semantic == S::kTemperatures ||
                   (config_.model == Model::kL20Lite && force_scalar(semantic));
        }
        return false;
    }
    bool PythonState::disabled(S semantic) const {
        const std::lock_guard<std::mutex> lock(mutex_);
        return disabled_.count(semantic) != 0;
    }
    void PythonState::disable_locked(S semantic) {
        disabled_.insert(semantic);
        failures_.erase(semantic);
        if (auto poll = polls_.find(semantic); poll != polls_.end()) {
            poll->second.enabled = false;
        }
        visible_locked(false);
    }
    void PythonState::read_succeeded(S semantic, bool polling) {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (optional(semantic)) {
            failures_.erase(semantic);
            verified_.insert(semantic);
        }
        if (auto poll = polls_.find(semantic); poll != polls_.end()) {
            ++poll->second.reads;
            if (!polling && !(semantic == S::kAngles &&
                              poll->second.channel.angle_feedback == AngleFeedbackPolicy::kCommandFeedbackOnly)) {
                poll->second.next = std::max(poll->second.next, runtime::Clock::now() + poll->second.channel.interval);
            }
        }
    }
    void PythonState::read_failed(S semantic, bool sent, const HandError &error, bool polling) {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (polling) {
            if (auto poll = polls_.find(semantic); poll != polls_.end()) {
                ++poll->second.timeouts;
            }
        }
        if (!ready_ || !sent || !optional(semantic) || verified_.count(semantic) || disabled_.count(semantic)) {
            return;
        }
        const auto status = error.context().native_status.value_or(0);
        const bool unavailable_o30 = config_.model == Model::kO30 && semantic == S::kForce &&
                                     (error.code() == ErrorCode::kUnsupportedOperation ||
                                      (error.code() == ErrorCode::kProtocol && (status & 0x03) && !(status & ~0x23)));
        if (unavailable_o30 || error.code() == ErrorCode::kUnsupportedFirmware ||
            ((error.code() == ErrorCode::kTimeout || error.code() == ErrorCode::kProtocol) &&
             ++failures_[semantic] >= 3)) {
            disable_locked(semantic);
            throw UnsupportedFirmwareError("firmware does not support this optional query", error.context(),
                                           std::current_exception());
        }
    }
    DeviceCapabilities PythonState::capabilities(DeviceCapabilities caps) const {
        const std::lock_guard<std::mutex> lock(mutex_);
        caps.firmware_unsupported_telemetry.assign(disabled_.begin(), disabled_.end());
        auto fields = caps.force_fields();
        for (auto &field : fields) {
            if (field.query_channel && disabled_.count(*field.query_channel)) {
                field.supported = false;
                field.firmware_unsupported = true;
            }
        }
        caps.observed_force_fields = std::move(fields);
        return caps;
    }
    bool PythonState::replace(const PollingPlan &plan) {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (plan_ && same_plan(*plan_, plan)) {
            return false;
        }
        std::map<S, Poll> next;
        const auto now = runtime::Clock::now();
        for (const auto &channel : plan) {
            next.emplace(channel.semantic,
                         Poll{channel,
                              now + (channel.semantic == S::kAngles &&
                                             channel.angle_feedback == AngleFeedbackPolicy::kCommandFeedbackOnly
                                         ? channel.interval
                                         : std::chrono::milliseconds{0}),
                              now});
        }
        plan_ = plan;
        polls_ = std::move(next);
        return true;
    }
    void PythonState::stop_polling() {
        const std::lock_guard<std::mutex> lock(mutex_);
        plan_.reset();
        polls_.clear();
    }
    std::optional<runtime::Deadline> PythonState::next_poll() const {
        const std::lock_guard<std::mutex> lock(mutex_);
        std::optional<runtime::Deadline> result;
        for (const auto &[semantic, state] : polls_) {
            (void)semantic;
            if (state.enabled && (!result || state.next < *result)) {
                result = state.next;
            }
        }
        return result;
    }
    std::optional<S> PythonState::take_poll() {
        const std::lock_guard<std::mutex> lock(mutex_);
        const auto now = runtime::Clock::now();
        auto selected = polls_.end();
        for (auto it = polls_.begin(); it != polls_.end(); ++it) {
            if (it->second.enabled && it->second.next <= now &&
                (selected == polls_.end() || it->second.next < selected->second.next)) {
                selected = it;
            }
        }
        if (selected == polls_.end()) {
            return std::nullopt;
        }
        auto &state = selected->second;
        const auto periods = static_cast<uint64_t>((now - state.next) / state.channel.interval) + 1;
        state.next += state.channel.interval * periods;
        const bool command_only =
            selected->first == S::kAngles && state.channel.angle_feedback == AngleFeedbackPolicy::kCommandFeedbackOnly;
        state.missed += command_only ? periods : periods - 1;
        return command_only ? std::nullopt : std::optional<S>{selected->first};
    }
    void PythonState::project_health(HandHealth &health) const {
        const std::lock_guard<std::mutex> lock(mutex_);
        const auto now = runtime::Clock::now();
        health.polling_active =
            std::any_of(polls_.begin(), polls_.end(), [](const auto &item) { return item.second.enabled; });
        for (const auto &[semantic, state] : polls_) {
            const auto name = health_name(semantic);
            auto found = std::find_if(health.channels.begin(), health.channels.end(),
                                      [&](const auto &c) { return c.channel == name; });
            if (found == health.channels.end()) {
                health.channels.push_back({});
                found = std::prev(health.channels.end());
                found->channel = name;
            }
            found->polling_active = state.enabled;
            found->supported = disabled_.count(semantic) == 0;
            found->requested_hz = state.enabled ? 1000.0 / state.channel.interval.count() : 0;
            const auto seconds = std::chrono::duration<double>(now - state.started).count();
            found->achieved_hz = seconds > 0 ? state.feedback / seconds : 0;
            found->completed_reads = state.reads;
            found->feedback_samples = state.feedback;
            found->command_feedback_samples = state.commands;
            found->missed_deadlines = state.missed;
            found->timeouts = state.timeouts;
            if (semantic == S::kAngles) {
                found->feedback_policy = state.channel.angle_feedback;
            }
        }
    }
    void PythonState::project(HandSnapshot &snapshot, const View &view, const DeviceCapabilities &caps) const {
        snapshot.snapshot_sequence = view.sequence;
        visit_snapshot_samples(snapshot, [&](auto &sample, S semantic) {
            if (!sample) {
                return;
            }
            const auto found = view.metadata.find(semantic);
            if (found != view.metadata.end()) {
                const auto origin = sample->meta.origin;
                const auto range = sample->meta.within_declared_range;
                sample->meta = found->second;
                sample->meta.origin = origin;
                sample->meta.within_declared_range = range;
                complete_freshness(sample->meta, caps.telemetry_channel(semantic), runtime::Clock::now());
            }
        });
        if (const auto identity = view.samples.find("python/identity"); identity != view.samples.end()) {
            snapshot.device_info = std::any_cast<DeviceIdentity>(identity->second.value);
            // Identity owns its logical sequence, independently of native queries.
            snapshot.device_info->meta.connection_generation = identity->second.device_generation;
            complete_freshness(snapshot.device_info->meta, nullptr, runtime::Clock::now());
        }
        std::vector<std::pair<S, runtime::Sample>> scalars;
        for (const auto semantic : {S::kForceFns, S::kForceFts, S::kForceFds, S::kForcePx}) {
            if (config_.model != Model::kL20Lite) {
                break;
            }
            const auto scalar = view.samples.find(domain(semantic));
            if (scalar != view.samples.end() && scalar->second.value.type() == typeid(ForceSample)) {
                scalars.emplace_back(semantic, scalar->second);
            }
        }
        std::sort(scalars.begin(), scalars.end(), [](const auto &left, const auto &right) {
            return left.second.received_at < right.second.received_at;
        });
        for (const auto &[semantic, scalar] : scalars) {
            auto sample = std::any_cast<ForceSample>(scalar.value);
            if (const auto meta = view.metadata.find(semantic); meta != view.metadata.end()) {
                sample.meta = meta->second;
                sample.meta.origin = TelemetryValueOrigin::kDeviceMeasured;
                for (auto &field : sample.fields) {
                    field.meta = sample.meta;
                }
            }
            if (!snapshot.force) {
                snapshot.force = sample;
            } else {
                merge_python_force_samples(*snapshot.force, sample);
            }
        }

        if (!snapshot.force && caps.telemetry_channel(S::kForce)) {
            snapshot.force.emplace();
        }
        if (snapshot.force) {
            for (auto &field : snapshot.force->fields) {
                const auto meta = view.metadata.find(field.query_channel);
                if (meta != view.metadata.end()) {
                    const auto origin = field.meta.origin;
                    field.meta = meta->second;
                    field.meta.origin =
                        origin == TelemetryValueOrigin::kNone ? TelemetryValueOrigin::kDeviceMeasured : origin;
                }
            }
            for (const auto &field : caps.force_fields()) {
                if (field.supported == false && !snapshot.force->field(field.name) &&
                    std::find(snapshot.force->unsupported_fields.begin(), snapshot.force->unsupported_fields.end(),
                              field.name) == snapshot.force->unsupported_fields.end()) {
                    snapshot.force->unsupported_fields.emplace_back(field.name);
                }
            }
            complete_force_meta(*snapshot.force, snapshot.force->meta.connection_generation, caps);
            complete_freshness(snapshot.force->meta, caps.telemetry_channel(S::kForce), runtime::Clock::now());
        }
    }
}

namespace linkerbot::hand::unified_detail {
    void read_python_scalar(runtime::DeviceRuntime &device, const HandDeviceConfig &config,
                            TelemetrySemantic semantic) {
        const auto caps = declared_capabilities(config);
        const auto field = semantic == TelemetrySemantic::kForceFns   ? "fns"
                           : semantic == TelemetrySemantic::kForceFts ? "fts"
                           : semantic == TelemetrySemantic::kForceFds ? "fds"
                                                                      : "px";
        const uint8_t opcode = semantic == TelemetrySemantic::kForceFns   ? 0x20
                               : semantic == TelemetrySemantic::kForceFts ? 0x21
                               : semantic == TelemetrySemantic::kForceFds ? 0x22
                                                                          : 0x23;
        const auto id = caps.transport.request_id.value_or(config.side == HandSide::kLeft ? 0x28U : 0x27U);
        runtime::TransactionPlan plan;
        plan.domain = "python/" + std::string(python_polling_domain(semantic));
        plan.keys = {plan.domain};
        can::Message request{};
        request.id = id;
        request.dlc = 1;
        request.data[0] = opcode;
        request.format = can::FrameFormat::kClassic;
        plan.frames = {request};
        plan.consume = [id, opcode, semantic, field,
                        names = caps.force_field(field).sensor_names](const can::Message &message) {
            if (message.id != id || message.is_fd || message.is_extended || message.is_brs || message.dlc == 0 ||
                message.data[0] != opcode) {
                return protocol::Progress::unmatched();
            }
            if (message.dlc != 6) {
                throw ProtocolError("force scalar response must contain five values");
            }
            ForceSample result;
            result.shape = {5};
            ForceFieldSample sample;
            sample.name = field;
            sample.query_channel = semantic;
            for (size_t i = 0; i < 5; ++i) {
                const auto value = message.data[i + 1];
                result.raw.push_back(value);
                sample.sensor_names.emplace_back(names.at(i));
                sample.raw.push_back(std::vector<double>{static_cast<double>(value)});
                sample.units.push_back(std::nullopt);
                sample.encodings.emplace_back("u8");
                sample.valid_values.push_back(
                    std::vector<bool>{semantic != TelemetrySemantic::kForceFds || value <= 127});
            }
            result.fields.push_back(std::move(sample));
            return protocol::Progress::complete(std::move(result));
        };
        (void)device.execute(std::move(plan), runtime::Deadline::max());
    }
}
