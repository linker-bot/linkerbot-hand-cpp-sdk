#pragma once
#include "classic_response.hpp"
#include "python_l30.hpp"
#include "python_tactile.hpp"
#include "runtime_access.hpp"
#include <algorithm>
#include <array>
#include <linkerbot/hand/api/config.hpp>
#include <linkerbot/hand/runtime/device_runtime.hpp>

namespace linkerbot::hand::unified_detail {
    class PythonHealthMetrics;
    void execute_python_transactions(runtime::DeviceRuntime &device, const HandDeviceConfig &config,
                                     const std::function<void()> &execute, runtime::Deadline deadline,
                                     const std::function<bool()> &valid, bool identity, PythonHealthMetrics &metrics,
                                     RuntimeAccess::ErrorCapture error_capture = {});

    inline const char *python_polling_domain(TelemetrySemantic semantic) {
        static constexpr std::array names{"angle",
                                          "speed",
                                          "configured_speed",
                                          "acceleration",
                                          "current",
                                          "position_kp",
                                          "position_kd",
                                          "torque_kd",
                                          "current_limit",
                                          "torque_limits",
                                          "torque",
                                          "temperature",
                                          "force",
                                          "force_fns",
                                          "force_ft",
                                          "force_fts",
                                          "force_fd",
                                          "force_fds",
                                          "force_px",
                                          "fault",
                                          "voltages",
                                          "motion_times",
                                          "stall_time",
                                          "stall_threshold",
                                          "stall_torque",
                                          "limit_compensations"};
        static_assert(names.size() == static_cast<size_t>(TelemetrySemantic::kLimitCompensations) + 1);
        const auto index = static_cast<size_t>(semantic);
        if (index >= names.size()) {
            throw ValidationError("unknown polling semantic");
        }
        return names[index];
    }

    // Values are protocol response windows, not caller wait budgets. Sources:
    // Python protocols/common/{six_axis,classic_multi_axis,classic_identity,
    // classic_management}.py and each model's tactile/identity transactions.
    inline protocol::TransactionPlan python_transaction(protocol::TransactionPlan plan,
                                                        const HandDeviceConfig &config) {
        if (!plan.consume) {
            return plan;
        }
        using namespace std::chrono_literals;
        auto window = 100ms;
        auto guard = 5ms;
        const bool six = config.model == Model::kL6 || config.model == Model::kO6;
        const bool identity = plan.domain.find("version") != std::string::npos;
        const bool tactile = (plan.domain.find("force") != std::string::npos && plan.domain.rfind("python/", 0) != 0) ||
                             plan.domain.rfind("tactile", 0) == 0 ||
                             (config.model == Model::kL30 && python_l30_tactile(plan));
        if (six && !identity && !tactile) {
            window = 50ms;
        }
        if (identity) {
            guard = config.model == Model::kL30 || config.model == Model::kO30 ? 10ms : 20ms;
        } else if (tactile) {
            if (config.model == Model::kO6) {
                window = 80ms;
                guard = 30ms;
            } else if (config.model == Model::kL30) {
                window = 50ms;
                guard = 20ms;
            } else if (config.model != Model::kO30) {
                window = 150ms;
                guard = 100ms;
            }
        }
        if ((config.model == Model::kL30 && !tactile) || config.model == Model::kO30) {
            guard = 10ms;
        }
        if (config.model == Model::kL20Lite && plan.domain.rfind("tip_", 0) == 0) {
            window = 100ms;
        }
        if (config.model == Model::kL20Lite && plan.optional_response_window) {
            guard = 100ms;
        }
        if (!plan.frames.empty() && six && !plan.frames.front().is_extended && !plan.frames.front().is_fd) {
            const auto frame = plan.frames.front();
            const auto opcode = frame.data[0];
            if (opcode == 0xCF) {
                window = 200ms;
                guard = 20ms;
            } else if (!identity && !tactile && opcode != 0x83) {
                plan = strict_classic_vector(std::move(plan), {frame.id, opcode, 6});
                if (frame.dlc > 1 && opcode != 1) {
                    window = 100ms;
                }
            }
        }
        if (identity && config.model != Model::kL30 && config.model != Model::kO30 && !plan.frames.empty() &&
            !plan.frames.front().is_fd && !plan.frames.front().is_extended) {
            auto decode = std::move(plan.consume);
            const auto id = plan.frames.front().id;
            const bool l6 = config.model == Model::kL6;
            plan.consume = [decode = std::move(decode), id, l6](const can::Message &message) mutable {
                if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc == 0) {
                    return protocol::Progress::unmatched();
                }
                const auto opcode = message.data[0];
                if (opcode == 0xC0 && (message.dlc < 3 || message.dlc > 8)) {
                    throw ProtocolError("serial response has an invalid length");
                }
                if (opcode == 0xC1 || opcode == 0xC2 || opcode == 0xC4) {
                    const auto expected = opcode == 0xC1 && l6 ? 5 : 4;
                    if (message.dlc != expected || (l6 && opcode == 0xC1 && message.data[1] != 1)) {
                        throw ProtocolError("version response has an invalid length or selector");
                    }
                }
                return decode(message);
            };
        }
        if ((config.model == Model::kL20 || config.model == Model::kL20Lite) && !identity && !tactile) {
            auto decode = std::move(plan.consume);
            const auto frames = plan.frames;
            const bool lite = config.model == Model::kL20Lite;
            plan.consume = [decode = std::move(decode), frames, lite](const can::Message &message) mutable {
                for (const auto &request : frames) {
                    if (message.id != request.id || message.is_extended || message.is_fd || message.is_brs ||
                        message.dlc == 0 || message.data[0] != request.data[0]) {
                        continue;
                    }
                    const auto opcode = request.data[0];
                    const auto expected = !lite                                                  ? 7
                                          : opcode == 1                                          ? 7
                                          : opcode == 4                                          ? 5
                                          : (opcode == 0xC5 || opcode == 0xC6 || opcode == 0xC7) ? 7
                                                                                                 : 6;
                    if (message.dlc != expected) {
                        throw ProtocolError("classic bank response has an invalid length");
                    }
                    if (lite && (opcode == 0xC5 || opcode == 0xC6 || opcode == 0xC7) && message.data[1] != 1 &&
                        message.data[1] != 2) {
                        throw ProtocolError("classic stall response has an invalid group");
                    }
                }
                return decode(message);
            };
        }
        if (config.model == Model::kL30) {
            plan = python_l30_response(std::move(plan));
        }
        if (tactile) {
            plan = python_tactile(std::move(plan), config.model);
        }
        plan.response_window = config.response_timeout.value_or(window);
        if (plan.optional_response_window) {
            plan.optional_response_window = plan.response_window;
            plan.response_window.reset();
        }
        plan.defer_errors_until_send = true;
        plan.late_guard = guard;
        plan.poison_on_timeout = false;
        plan.preserve_last_success = true;
        plan.publish_after_send = true;
        return plan;
    }
}
