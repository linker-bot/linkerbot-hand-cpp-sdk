#pragma once
#include <linkerbot/hand/api/events.hpp>
#include <map>

namespace linkerbot::hand::unified_detail {
    struct PythonPollingPlan {
        PollingPlan queries;
        std::map<std::string, std::chrono::milliseconds> fields;
    };
    DeviceCapabilities python_force_capabilities(DeviceCapabilities capabilities);
    std::string python_health_channel(TelemetrySemantic semantic);
    PythonPollingPlan resolve_python_polling(const PollingPlan &plan, const DeviceCapabilities &capabilities,
                                             bool strict = true);
}
