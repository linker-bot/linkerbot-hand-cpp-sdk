#pragma once

#include <linkerbot/hand/runtime/device_runtime.hpp>

namespace linkerbot::hand::unified_detail {
    // Internal hook: preserve native polling plans/metrics and receivers while
    // making their entire composite read share the unified execution arbiter.
    class RuntimeAccess {
    public:
        using PollingGate = std::function<std::shared_ptr<void>(runtime::CancellationToken)>;
        static void set_polling_gate(runtime::DeviceRuntime &runtime, PollingGate gate);
        using SendObserver = std::function<void(const transport::SendResult &, bool)>;
        using FinalSend = std::function<bool(const runtime::TransactionPlan &)>;
        // Scope only this caller's native transactions. Its absolute budget is
        // inherited by nested operations; receipt callbacks run under the
        // runtime lock and must never call back into runtime/facade state.
        // prepare_send runs before request accounting and physical admission;
        // failures reject that batch without I/O. Reserve observer storage here.
        // accepted_send runs only after the receipt and terminal observer are installed.
        static void execute_command(runtime::DeviceRuntime &runtime, runtime::Deadline deadline, bool send_only,
                                    SendObserver observer, FinalSend final_send,
                                    runtime::CancellationToken cancellation, const std::function<void()> &execute,
                                    std::function<void()> prepare_send = {}, std::function<void()> accepted_send = {});
    };
}
