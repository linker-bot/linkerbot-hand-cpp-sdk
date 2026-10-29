#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/hand/fault_status.hpp>
#include <linkerbot/hand/o30/sensor.hpp>

namespace linkerbot::hand {
    enum class TelemetryValueOrigin : uint8_t {
        kNone,
        kUnknown,
        kDeviceMeasured,
        kDeviceConfigured,
        kCommandCache,
        kDerived
    };

    enum class TelemetryDelivery : uint8_t {
        kUnknown,
        kQueryResponse,
        kPeriodicReport,
        kQueryOrReport,
        kCommandResponse,
        kCommandCache
    };

    struct SampleMeta {
        uint64_t sequence = 0;
        std::optional<std::chrono::steady_clock::time_point> collected_at;
        std::optional<std::chrono::steady_clock::duration> age;
        bool valid = false;
        bool stale = true;
        uint64_t connection_generation = 0;
        TelemetryValueOrigin origin = TelemetryValueOrigin::kNone;
        TelemetryDelivery delivery = TelemetryDelivery::kUnknown;
        bool within_declared_range = true;

        [[nodiscard]] static SampleMeta unavailable(uint64_t connection_generation = 0) noexcept {
            SampleMeta result;
            result.connection_generation = connection_generation;
            return result;
        }
    };

    struct AngleSample {
        std::vector<double> raw;
        std::vector<std::optional<double>> percent;
        std::vector<std::optional<double>> physical_rad;
        SampleMeta meta;
    };

    struct RawVectorSample {
        std::vector<double> raw;
        SampleMeta meta;
    };

#define LINKERBOT_HAND_RAW_VECTOR_SAMPLE(Name)                                                                         \
    struct Name : RawVectorSample {}

    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(MeasuredSpeedSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(ConfiguredSpeedSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(ConfiguredAccelerationSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(CurrentSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(CurrentLimitSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(TorqueSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(TemperatureSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(VoltageSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(StallDetectionTimeSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(StallThresholdSample);
    LINKERBOT_HAND_RAW_VECTOR_SAMPLE(StallHoldCurrentSample);

#undef LINKERBOT_HAND_RAW_VECTOR_SAMPLE

    struct FaultSample {
        std::vector<uint8_t> raw;
        std::vector<FaultStatus> statuses;
        SampleMeta meta;
    };

    struct TactileMatrix {
        std::string name;
        size_t rows = 0;
        size_t columns = 0;
        std::vector<std::vector<double>> raw;
        std::optional<bool> online;
    };

    struct ForceFieldSample {
        std::string name;
        std::vector<std::string> sensor_names;
        // One vector per finger, including scalars. nullopt means absent.
        std::vector<std::optional<std::vector<double>>> raw;
        std::vector<std::optional<std::string>> units;
        std::vector<std::optional<std::string>> encodings;
        std::vector<std::optional<std::vector<bool>>> valid_values;
        SampleMeta meta;
        TelemetrySemantic query_channel = TelemetrySemantic::kForce;
        std::optional<std::string> scale_note;
    };

    using TactileFieldSample = o30::SensorField;

    struct TactileSensorSample {
        std::string sensor_name;
        uint8_t selected_sensor = 0;
        std::string sensor_type;
        std::string data_unit;
        std::string descriptor;
        size_t rows = 0;
        size_t columns = 0;
        bool available = false;
        std::vector<uint8_t> raw_data;
        std::vector<TactileFieldSample> fields;
    };

    struct ForceSample {
        // Flattened finger-major protocol payload. shape describes dimensions.
        std::vector<uint8_t> raw;
        std::vector<size_t> shape;
        std::vector<std::string> requested_fields;
        std::vector<std::string> unsupported_fields;
        SampleMeta meta;
        std::vector<TactileMatrix> tactile;
        std::vector<TactileSensorSample> sensors;
        std::vector<ForceFieldSample> fields;
        [[nodiscard]] const ForceFieldSample *field(std::string_view name) const;
        [[nodiscard]] std::string_view status(std::string_view name) const;
    };

    struct NormalForceResultantSample {
        std::vector<double> raw;
        SampleMeta meta;
        std::vector<std::string> sensor_names;
        std::vector<std::optional<std::string>> units;
    };
} // namespace linkerbot::hand
