#pragma once
#include "model_adapters.hpp"
namespace linkerbot::hand::unified_detail {
    CommandOutcome submit_v25(o30::O30 &device, const CommandRequest &request);
    TelemetrySample read_v25(o30::O30 &device, TelemetrySemantic semantic, std::chrono::milliseconds timeout,
                             const DeviceCapabilities &capabilities);
    void project_v25(const o30::O30 &device, HandSnapshot &snapshot, const DeviceCapabilities &capabilities);
}
