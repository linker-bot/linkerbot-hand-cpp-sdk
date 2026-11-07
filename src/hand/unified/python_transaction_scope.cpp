#include "detail/python_transactions.hpp"
#include "detail/runtime_access.hpp"

namespace linkerbot::hand::unified_detail {
    void execute_python_transactions(runtime::DeviceRuntime &device, const HandDeviceConfig &config,
                                     const std::function<void()> &execute, runtime::Deadline deadline,
                                     const std::function<bool()> &valid, bool identity) {
        RuntimeAccess::with_response_windows(
            device,
            [&config, identity](runtime::TransactionPlan plan) {
                plan = python_transaction(std::move(plan), config);
                if (identity && plan.consume) {
                    plan.response_window = config.identity_timeout;
                }
                return plan;
            },
            execute, deadline, valid);
    }
}
