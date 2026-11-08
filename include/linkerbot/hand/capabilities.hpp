#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace linkerbot::hand {
    enum class Model : uint8_t { kUnknown, kL6, kO6, kL20Lite, kL20, kL30, kO30 };

    inline constexpr std::array<Model, 6> kSupportedModels{Model::kL6,  Model::kO6,  Model::kL20Lite,
                                                           Model::kL20, Model::kL30, Model::kO30};

    [[nodiscard]] constexpr const std::array<Model, 6> &supported_models() noexcept { return kSupportedModels; }

    enum class HandSide : uint8_t { kUnknown, kLeft, kRight };

    enum class AngleFeedbackPolicy : uint8_t { kQueryOnly, kCommandFeedbackOnly, kAdaptive };

    enum class TransportKind : uint8_t { kCan, kCanFd };

    enum class Confidence : uint8_t { kMeasured, kDocumented, kApproximate, kConflicting, kUnknown };

    enum class Quantization : uint8_t { kNearestTiesAwayFromZero, kFloor, kCeil };

    enum class JointMotionType : uint8_t { kUnknown, kFlexion, kAbduction, kRoll, kRotation, kWrist };

    enum class ReadMode : uint8_t { kQuery, kReport, kQueryAndReport, kCommandCache, kCommandResponse };

    enum class AckSupport : uint8_t { kUnsupported, kOptional, kRequired };

    enum class ByteOrder : uint8_t { kNotApplicable, kLittleEndian, kBigEndian };

    enum class Signedness : uint8_t { kNotApplicable, kUnsigned, kSigned };

    enum class Packing : uint8_t { kScalar, kArray, kBitField, kObject, kSparseObject };

    enum class WriteMode : uint8_t { kNotApplicable, kSparse, kFull, kSparseOrFull };

    enum class ResponseSemantics : uint8_t { kNone, kAck, kCurrentState, kReadback, kError };

    enum class Idempotency : uint8_t { kUnknown, kIdempotent, kNonIdempotent };

    enum class TelemetrySource : uint8_t { kDevice, kSdkCache, kCommandResponse, kDerived };

    struct CapabilitySchemaVersion {
        uint16_t major = 1;
        uint16_t minor = 0;
    };

    inline constexpr CapabilitySchemaVersion kCapabilitySchemaVersion{1, 1};

    [[nodiscard]] bool schema_compatible(CapabilitySchemaVersion consumer, CapabilitySchemaVersion producer) noexcept;
    [[nodiscard]] bool is_schema_compatible(CapabilitySchemaVersion consumer,
                                            CapabilitySchemaVersion producer) noexcept;

    enum class CommandSemantic : uint8_t {
        kAngles,
        kSpeeds,
        kAccelerations,
        kCurrentLimits,
        kPositionKp = 4,
        kPositionKd,
        kTorqueKd,
        kTorqueLimits,
        kTorques,
        kMotionTimes,
        kStallDetectionTimes = 10,
        kStallThresholds,
        kStallHoldCurrents,
        kLimitCompensations
    };

    enum class TelemetrySemantic : uint8_t {
        kAngles,
        kMeasuredSpeeds,
        kConfiguredSpeeds,
        kConfiguredAccelerations,
        kCurrents,
        kPositionKp = 5,
        kPositionKd,
        kTorqueKd,
        kCurrentLimits = 8,
        kTorqueLimits = 9,
        kTorques = 10,
        kTemperatures,
        kForce,
        kForceFns,
        kForceFt,
        kForceFts,
        kForceFd,
        kForceFds,
        kForcePx,
        kFaults,
        kVoltages,
        kMotionTimes = 21,
        kStallDetectionTimes = 22,
        kStallThresholds,
        kStallHoldCurrents,
        kLimitCompensations
    };

    enum class DeviceOperation : uint8_t {
        kReadDeviceInfo,
        kClearFaults,
        kEnable,
        kDisable,
        kCalibrate = 4,
        kSaveParameters = 5,
        kSaveParametersConfirmed
    };

    struct NumericRange {
        double minimum = 0.0;
        double maximum = 0.0;
        bool integer_only = false;
        std::optional<double> step;

        [[nodiscard]] bool contains(double value) const noexcept;
    };

    struct PercentMapping {
        double raw_at_zero = 0.0;
        double raw_at_hundred = 0.0;
        std::string_view zero_label;
        std::string_view hundred_label;
        Confidence confidence = Confidence::kUnknown;
        Quantization quantization = Quantization::kNearestTiesAwayFromZero;
    };

    struct PhysicalAngleMapping {
        double raw_at_first = 0.0;
        double raw_at_second = 0.0;
        double radians_at_first = 0.0;
        double radians_at_second = 0.0;
        Confidence confidence = Confidence::kUnknown;
    };

    struct JointCapability {
        size_t index = 0;
        std::string_view name;
        JointMotionType motion_type = JointMotionType::kUnknown;
        NumericRange raw_angle_range;
        std::optional<PercentMapping> percent_mapping;
        std::vector<std::string_view> coupled_with;
        std::optional<PhysicalAngleMapping> physical_angle_mapping;
    };

    struct CommandCapability {
        CommandSemantic semantic = CommandSemantic::kAngles;
        std::vector<std::string_view> channels;
        // One range applies to every channel, or one range is supplied per channel.
        std::vector<NumericRange> raw_ranges;
        std::string_view raw_encoding;
        std::optional<std::string_view> raw_unit;
        std::optional<std::string_view> raw_scale_note;
        Confidence unit_confidence = Confidence::kUnknown;
        AckSupport ack_support = AckSupport::kUnsupported;
        bool returns_current_state = false;

        std::vector<NumericRange> normalized_ranges;
        std::optional<std::string_view> normalized_unit;
        ByteOrder byte_order = ByteOrder::kNotApplicable;
        Signedness signedness = Signedness::kNotApplicable;
        Packing packing = Packing::kArray;
        WriteMode write_mode = WriteMode::kSparseOrFull;
        uint32_t timeout_ms = 100;
        ResponseSemantics response = ResponseSemantics::kNone;
        Idempotency idempotency = Idempotency::kUnknown;
        std::vector<std::string_view> preconditions;
        std::vector<std::optional<PercentMapping>> percent_mappings;

        [[nodiscard]] const NumericRange *range_for(size_t channel_index) const noexcept;
        [[nodiscard]] const NumericRange *raw_range_for(size_t channel_index) const noexcept {
            return range_for(channel_index);
        }
        [[nodiscard]] const NumericRange *normalized_range_for(size_t channel_index) const noexcept;
    };

    struct TelemetryCapability {
        TelemetrySemantic semantic = TelemetrySemantic::kAngles;
        std::vector<std::string_view> channels;
        ReadMode read_mode = ReadMode::kQuery;
        std::string_view raw_encoding;
        std::vector<size_t> shape;
        std::optional<NumericRange> raw_range;
        std::optional<std::string_view> raw_unit;
        std::optional<std::string_view> raw_scale_note;
        Confidence unit_confidence = Confidence::kUnknown;
        bool device_measured = true;
        bool device_queried = true;

        TelemetrySource source = TelemetrySource::kDevice;
        bool can_query = true;
        bool can_report = false;
        bool can_stream = false;
        std::optional<uint32_t> recommended_period_ms;
        std::optional<uint32_t> minimum_period_ms;
        // Milliseconds, including fractional periods (e.g. 2 / 60 Hz).
        std::optional<double> freshness_ms;
    };

    struct TransportCapability {
        TransportKind kind = TransportKind::kCan;
        std::optional<uint32_t> nominal_bitrate;
        std::optional<uint32_t> data_bitrate;
        bool extended_frame = false;
        std::string_view addressing_note;

        std::optional<uint32_t> request_id;
        std::optional<uint32_t> response_id;
        uint32_t id_mask = 0x7FFU;
        std::optional<uint8_t> node_id;
        std::optional<uint8_t> host_id;
        bool bit_rate_switch = false;
        uint16_t mtu = 8;
    };

    struct OperationCapabilities {
        bool read_device_info = false;
        bool clear_faults = false;
        bool enable = false;
        bool disable = false;
        bool save_parameters = false;
        std::vector<DeviceOperation> acknowledged_operations;
        bool calibrate = false;
        bool save_parameters_confirmed = false;

        [[nodiscard]] bool supports(DeviceOperation operation) const noexcept;
        [[nodiscard]] bool acknowledged(DeviceOperation operation) const noexcept;
    };

    inline constexpr std::array<std::string_view, 7> kForceFields{"fn", "fns", "ft", "fts", "fd", "fds", "px"};

    struct ForceFieldCapability {
        std::string_view name;
        // nullopt means support awaits O30 sensor descriptors.
        std::optional<bool> supported = false;
        std::optional<TelemetrySemantic> query_channel;
        std::vector<std::string_view> shared_fields;
        std::vector<std::string_view> sensor_names;
        std::optional<std::string_view> raw_unit;
        std::optional<std::string_view> raw_encoding;
        std::optional<std::string_view> raw_scale_note;
        bool firmware_unsupported = false;
    };

    // Capability strings are non-owning views into SDK-owned static profile data.
    // A DeviceCapabilities value must therefore not be assembled from temporary strings.
    struct DeviceCapabilities {
        Model model = Model::kUnknown;
        HandSide side = HandSide::kUnknown;
        TransportCapability transport;
        std::vector<JointCapability> joints;
        std::vector<CommandCapability> commands;
        std::vector<TelemetryCapability> telemetry;
        OperationCapabilities operations;
        CapabilitySchemaVersion schema_version = kCapabilitySchemaVersion;
        // Runtime descriptor observations; views still reference static profile names.
        std::vector<ForceFieldCapability> observed_force_fields;
        // Observed only in the current verified connection generation.
        std::vector<TelemetrySemantic> firmware_unsupported_telemetry;

        [[nodiscard]] bool supports(CommandSemantic semantic) const noexcept;
        [[nodiscard]] bool supports(TelemetrySemantic semantic) const noexcept;
        [[nodiscard]] bool supports(DeviceOperation operation) const noexcept;

        [[nodiscard]] const CommandCapability *command(CommandSemantic semantic) const noexcept;
        [[nodiscard]] const TelemetryCapability *telemetry_channel(TelemetrySemantic semantic) const noexcept;
        [[nodiscard]] bool compatible_with(CapabilitySchemaVersion consumer) const noexcept;
        [[nodiscard]] bool validate() const noexcept;
        [[nodiscard]] std::string serialize() const;
        [[nodiscard]] std::vector<ForceFieldCapability> force_fields() const;
        [[nodiscard]] ForceFieldCapability force_field(std::string_view name) const;
    };

    struct HandDeviceConfig;
    // Resolves protocol-specific ranges and the configured request/response IDs.
    [[nodiscard]] DeviceCapabilities declared_capabilities(const HandDeviceConfig &config);

    [[nodiscard]] std::string serialize_capabilities(const DeviceCapabilities &capabilities);
    [[nodiscard]] std::string to_json(const DeviceCapabilities &capabilities);

    // Returns an SDK-owned immutable profile without performing device I/O.
    // Throws std::invalid_argument for an unknown model or invalid side value.
    [[nodiscard]] const DeviceCapabilities &declared_capabilities(Model model, HandSide side = HandSide::kUnknown);

} // namespace linkerbot::hand
