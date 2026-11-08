#include "detail/python_polling.hpp"
#include "detail/force.hpp"
#include "detail/python_transactions.hpp"
#include <algorithm>
#include <linkerbot/hand/errors.hpp>
#include <set>

namespace linkerbot::hand::unified_detail {
    namespace {
        using S = TelemetrySemantic;
        void validate_interval(std::chrono::milliseconds interval) {
            const auto available =
                std::chrono::duration_cast<std::chrono::milliseconds>(runtime::Deadline::max() - runtime::Clock::now());
            if (interval.count() <= 0 || interval >= available) {
                throw PollingPlanError("polling interval must be positive and representable");
            }
        }
        bool is_force(S semantic) { return semantic >= S::kForce && semantic <= S::kForcePx; }
        const char *field_name(S semantic) {
            switch (semantic) {
            case S::kForceFns:
                return "fns";
            case S::kForceFt:
                return "ft";
            case S::kForceFts:
                return "fts";
            case S::kForceFd:
                return "fd";
            case S::kForceFds:
                return "fds";
            case S::kForcePx:
                return "px";
            default:
                throw PollingPlanError("unknown force query");
            }
        }
    }
    std::string python_health_channel(S semantic) {
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
    DeviceCapabilities python_force_capabilities(DeviceCapabilities caps) {
        auto fields = caps.force_fields();
        if (caps.model == Model::kL20Lite) {
            for (auto &field : fields) {
                for (const auto semantic : {S::kForceFns, S::kForceFts, S::kForceFds, S::kForcePx}) {
                    if (field.name == field_name(semantic) && caps.supports(semantic)) {
                        field.query_channel = semantic;
                        field.shared_fields = {field.name};
                    }
                }
            }
        }
        caps.observed_force_fields = std::move(fields);
        return caps;
    }
    PythonPollingPlan resolve_python_polling(const PollingPlan &plan, const DeviceCapabilities &capabilities,
                                             bool strict) {
        const auto caps = python_force_capabilities(capabilities);
        PythonPollingPlan result;
        std::map<S, PollingChannel> queries;
        std::set<S> unique;
        std::set<std::string> defaults;
        bool default_force = false;
        bool default_supported = false;
        for (const auto &channel : plan) {
            validate_interval(channel.interval);
            if (!unique.insert(channel.semantic).second) {
                throw PollingPlanError("polling semantics must be unique");
            }
            if (channel.angle_feedback != AngleFeedbackPolicy::kQueryOnly &&
                channel.angle_feedback != AngleFeedbackPolicy::kCommandFeedbackOnly &&
                channel.angle_feedback != AngleFeedbackPolicy::kAdaptive) {
                throw PollingPlanError("unknown angle feedback policy");
            }
            if (!channel.force_fields.empty() && channel.semantic != S::kForce) {
                throw PollingPlanError("force field intervals require the force selector");
            }
            if (!is_force(channel.semantic)) {
                queries.emplace(channel.semantic, channel);
                continue;
            }
            auto fields = channel.force_fields;
            if (fields.empty()) {
                if (channel.semantic == S::kForce) {
                    default_force = true;
                    fields = {{"fn", channel.interval}, {"fns", channel.interval}};
                    defaults = {"fn", "fns"};
                } else {
                    fields = {{field_name(channel.semantic), channel.interval}};
                }
            }
            for (const auto &field : fields) {
                validate_force_field(field.field);
                validate_interval(field.interval);
                if (!result.fields.emplace(field.field, field.interval).second) {
                    throw PollingPlanError("force fields must be unique");
                }
            }
        }
        for (const auto &[name, interval] : result.fields) {
            const auto field = caps.force_field(name);
            const bool allowed = field.supported.value_or(true);
            if (default_force && defaults.count(name) && allowed) {
                default_supported = true;
            }
            if (!allowed) {
                if (!strict || defaults.count(name)) {
                    continue;
                }
                if (field.firmware_unsupported) {
                    throw UnsupportedFirmwareError("firmware does not support requested force field");
                }
                throw UnsupportedOperationError("requested force field is not supported");
            }
            if (!field.query_channel) {
                continue;
            }
            const auto semantic = field.query_channel.value_or(S::kForce);
            const auto found = queries.find(semantic);
            if (found == queries.end()) {
                queries.emplace(semantic, PollingChannel{semantic, interval});
            } else {
                found->second.interval = std::min(found->second.interval, interval);
            }
        }
        if (strict && default_force && !default_supported) {
            const bool firmware =
                caps.force_field("fn").firmware_unsupported || caps.force_field("fns").firmware_unsupported;
            if (firmware) {
                throw UnsupportedFirmwareError("firmware does not support default force polling");
            }
            throw UnsupportedOperationError("default force polling is not supported");
        }
        for (auto &[semantic, query] : queries) {
            (void)semantic;
            result.queries.push_back(std::move(query));
        }
        return result;
    }
}
