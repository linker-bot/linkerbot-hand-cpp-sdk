#pragma once

#include <linkerbot/hand/runtime/device_runtime.hpp>

namespace linkerbot::hand::unified_detail {
    // Internal hook: preserve native polling plans/metrics and receivers while
    // making their entire composite read share the unified execution arbiter.
    class RuntimeAccess {
    public:
        using PollingGate = std::function<std::shared_ptr<void>(runtime::CancellationToken)>;
        static void set_polling_gate(runtime::DeviceRuntime &runtime, PollingGate gate);
        static void wait_endpoint_quiet(runtime::DeviceRuntime &runtime, std::chrono::milliseconds quiet,
                                        runtime::Deadline deadline, const std::function<bool()> &cancelled);
        using PlanTransform = std::function<runtime::TransactionPlan(runtime::TransactionPlan)>;
        static void with_response_windows(runtime::DeviceRuntime &runtime, PlanTransform transform,
                                          const std::function<void()> &execute,
                                          runtime::Deadline execution_deadline = runtime::Deadline::max(),
                                          std::function<bool()> generation_valid = {});
        using SendObserver = std::function<void(const transport::SendResult &, bool)>;
        using Submission = std::function<transport::SendReceipt()>;
        using AdmitSend = std::function<transport::SendReceipt(const Submission &, bool)>;
        using ReceiptAccepted = std::function<void(const transport::SendReceipt &, bool)>;
        static void retire_closed_profiles(const std::shared_ptr<transport::Session> &session);
        static void stop_endpoint(runtime::DeviceRuntime &runtime);
        static void resume_endpoint(runtime::DeviceRuntime &runtime);
        static void flush_sends(runtime::DeviceRuntime &runtime);
        static void drain_sends(runtime::DeviceRuntime &runtime, runtime::Deadline deadline);
        static bool uses_response_windows(const runtime::DeviceRuntime &runtime);
        using FinalSend = std::function<bool(const runtime::TransactionPlan &)>;
        // Scope only this caller's native transactions. Its absolute budget is
        // inherited by nested operations; receipt callbacks run under the
        // runtime lock and must never call back into runtime/facade state.
        // prepare_send runs before request accounting and physical admission;
        // failures reject that batch without I/O. Reserve observer storage here.
        // accepted_send runs only after the receipt and terminal observer are installed.
        // admit_send wraps one synchronous Session admission. It must not call
        // runtime/facade state or retain the borrowed submission callback.
        static void execute_command(runtime::DeviceRuntime &runtime, runtime::Deadline deadline, bool send_only,
                                    SendObserver observer, FinalSend final_send,
                                    runtime::CancellationToken cancellation, const std::function<void()> &execute,
                                    std::function<void()> prepare_send = {}, std::function<void()> accepted_send = {},
                                    ReceiptAccepted receipt_accepted = {}, AdmitSend admit_send = {});
    };
}
