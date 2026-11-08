#pragma once

#include <linkerbot/hand/api/readings.hpp>
#include <linkerbot/hand/l20lite/tip_force.hpp>
#include <linkerbot/hand/o30/sensor.hpp>

namespace linkerbot::hand::unified_detail {
    void validate_force_field(std::string_view name);
    void merge_force_samples(ForceSample &target, const ForceSample &source);
    void merge_python_force_samples(ForceSample &target, const ForceSample &source);
    ForceSample tip_force_sample(const l20lite::TipForceData &data, SampleMeta meta,
                                 const DeviceCapabilities &capabilities);
    ForceSample descriptor_force_sample(const o30::SensorData &data, SampleMeta meta);
    NormalForceResultantSample normal_force_resultant(const ForceSample &sample);
    void complete_force_meta(ForceSample &sample, uint64_t generation, const DeviceCapabilities &capabilities);
}
