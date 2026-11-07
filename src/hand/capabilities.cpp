#include <linkerbot/hand/capabilities.hpp>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace linkerbot::hand {
    bool NumericRange::contains(double value) const noexcept {
        if (!std::isfinite(value) || !std::isfinite(minimum) || !std::isfinite(maximum) || minimum > maximum) {
            return false;
        }
        if (integer_only && std::trunc(value) != value) {
            return false;
        }
        return value >= minimum && value <= maximum;
    }

    bool schema_compatible(CapabilitySchemaVersion consumer, CapabilitySchemaVersion producer) noexcept {
        return consumer.major == producer.major && producer.minor >= consumer.minor;
    }

    bool is_schema_compatible(CapabilitySchemaVersion consumer, CapabilitySchemaVersion producer) noexcept {
        return schema_compatible(consumer, producer);
    }

    const NumericRange *CommandCapability::range_for(size_t channel_index) const noexcept {
        if (channel_index >= channels.size() || raw_ranges.empty()) {
            return nullptr;
        }
        if (raw_ranges.size() == 1) {
            return &raw_ranges.front();
        }
        if (raw_ranges.size() != channels.size()) {
            return nullptr;
        }
        return &raw_ranges[channel_index];
    }

    const NumericRange *CommandCapability::normalized_range_for(size_t channel_index) const noexcept {
        if (channel_index >= channels.size() || normalized_ranges.empty()) {
            return nullptr;
        }
        if (normalized_ranges.size() == 1) {
            return &normalized_ranges.front();
        }
        if (normalized_ranges.size() != channels.size()) {
            return nullptr;
        }
        return &normalized_ranges[channel_index];
    }

    bool DeviceCapabilities::supports(CommandSemantic semantic) const noexcept { return command(semantic) != nullptr; }

    bool DeviceCapabilities::supports(TelemetrySemantic semantic) const noexcept {
        return telemetry_channel(semantic) != nullptr;
    }

    bool OperationCapabilities::supports(DeviceOperation operation) const noexcept {
        switch (operation) {
        case DeviceOperation::kReadDeviceInfo:
            return read_device_info;
        case DeviceOperation::kClearFaults:
            return clear_faults;
        case DeviceOperation::kEnable:
            return enable;
        case DeviceOperation::kDisable:
            return disable;
        case DeviceOperation::kSaveParameters:
            return save_parameters;
        case DeviceOperation::kSaveParametersConfirmed:
            return save_parameters_confirmed;
        case DeviceOperation::kCalibrate:
            return calibrate;
        }
        return false;
    }

    bool OperationCapabilities::acknowledged(DeviceOperation operation) const noexcept {
        for (const auto candidate : acknowledged_operations) {
            if (candidate == operation) {
                return true;
            }
        }
        return false;
    }

    bool DeviceCapabilities::supports(DeviceOperation operation) const noexcept {
        return operations.supports(operation);
    }

    const CommandCapability *DeviceCapabilities::command(CommandSemantic semantic) const noexcept {
        for (const auto &capability : commands) {
            if (capability.semantic == semantic) {
                return &capability;
            }
        }
        return nullptr;
    }

    const TelemetryCapability *DeviceCapabilities::telemetry_channel(TelemetrySemantic semantic) const noexcept {
        for (const auto &capability : telemetry) {
            if (capability.semantic == semantic) {
                return &capability;
            }
        }
        return nullptr;
    }

    bool DeviceCapabilities::compatible_with(CapabilitySchemaVersion consumer) const noexcept {
        return schema_compatible(consumer, schema_version) && validate();
    }

    bool DeviceCapabilities::validate() const noexcept {
        const auto valid_range = [](const NumericRange &range) noexcept {
            if (!range.contains(range.minimum) || !range.contains(range.maximum)) {
                return false;
            }
            return !range.step.has_value() || (std::isfinite(*range.step) && *range.step > 0.0);
        };
        const auto valid_percent_mapping = [](const PercentMapping &mapping, const NumericRange &range) noexcept {
            return std::isfinite(mapping.raw_at_zero) && std::isfinite(mapping.raw_at_hundred) &&
                   mapping.raw_at_zero != mapping.raw_at_hundred && range.contains(mapping.raw_at_zero) &&
                   range.contains(mapping.raw_at_hundred);
        };

        if (model == Model::kUnknown || schema_version.major == 0 || transport.mtu == 0 || joints.empty() ||
            (transport.request_id.has_value() && *transport.request_id > transport.id_mask) ||
            (transport.response_id.has_value() && *transport.response_id > transport.id_mask)) {
            return false;
        }
        for (size_t index = 0; index < joints.size(); ++index) {
            const auto &joint = joints[index];
            if (joint.index != index || joint.name.empty() || !valid_range(joint.raw_angle_range)) {
                return false;
            }
            if (joint.percent_mapping && !valid_percent_mapping(*joint.percent_mapping, joint.raw_angle_range)) {
                return false;
            }
            if (joint.physical_angle_mapping) {
                const auto &mapping = *joint.physical_angle_mapping;
                if (!std::isfinite(mapping.raw_at_first) || !std::isfinite(mapping.raw_at_second) ||
                    !std::isfinite(mapping.radians_at_first) || !std::isfinite(mapping.radians_at_second) ||
                    mapping.raw_at_first == mapping.raw_at_second ||
                    !joint.raw_angle_range.contains(mapping.raw_at_first) ||
                    !joint.raw_angle_range.contains(mapping.raw_at_second)) {
                    return false;
                }
            }
        }
        for (size_t index = 0; index < operations.acknowledged_operations.size(); ++index) {
            const auto operation = operations.acknowledged_operations[index];
            if (!operations.supports(operation)) {
                return false;
            }
            for (size_t previous = 0; previous < index; ++previous) {
                if (operations.acknowledged_operations[previous] == operation) {
                    return false;
                }
            }
        }
        for (size_t command_index = 0; command_index < commands.size(); ++command_index) {
            const auto &command_capability = commands[command_index];
            if (command_capability.channels.empty() || command_capability.raw_ranges.empty() ||
                (command_capability.raw_ranges.size() != 1 &&
                 command_capability.raw_ranges.size() != command_capability.channels.size()) ||
                (command_capability.normalized_ranges.size() > 1 &&
                 command_capability.normalized_ranges.size() != command_capability.channels.size()) ||
                command_capability.timeout_ms == 0) {
                return false;
            }
            for (size_t previous = 0; previous < command_index; ++previous) {
                if (commands[previous].semantic == command_capability.semantic) {
                    return false;
                }
            }
            for (const auto &range : command_capability.raw_ranges) {
                if (!valid_range(range)) {
                    return false;
                }
            }
            for (const auto &range : command_capability.normalized_ranges) {
                if (!valid_range(range)) {
                    return false;
                }
            }
            if (!command_capability.percent_mappings.empty() &&
                command_capability.percent_mappings.size() != command_capability.channels.size()) {
                return false;
            }
            for (size_t index = 0; index < command_capability.percent_mappings.size(); ++index) {
                const auto &mapping = command_capability.percent_mappings[index];
                const auto *range = command_capability.range_for(index);
                if (mapping && (!range || !valid_percent_mapping(*mapping, *range))) {
                    return false;
                }
            }
        }
        for (size_t telemetry_index = 0; telemetry_index < telemetry.size(); ++telemetry_index) {
            const auto &telemetry_capability = telemetry[telemetry_index];
            if (telemetry_capability.channels.empty() || telemetry_capability.shape.empty()) {
                return false;
            }
            for (size_t previous = 0; previous < telemetry_index; ++previous) {
                if (telemetry[previous].semantic == telemetry_capability.semantic) {
                    return false;
                }
            }
            for (const auto dimension : telemetry_capability.shape) {
                if (dimension == 0) {
                    return false;
                }
            }
            if (telemetry_capability.raw_range && !valid_range(*telemetry_capability.raw_range)) {
                return false;
            }
            if (telemetry_capability.freshness_ms &&
                (!std::isfinite(*telemetry_capability.freshness_ms) || *telemetry_capability.freshness_ms <= 0.0)) {
                return false;
            }
            if (telemetry_capability.minimum_period_ms.has_value() && *telemetry_capability.minimum_period_ms == 0) {
                return false;
            }
            if (telemetry_capability.recommended_period_ms.has_value() &&
                *telemetry_capability.recommended_period_ms == 0) {
                return false;
            }
            if (telemetry_capability.minimum_period_ms.has_value() &&
                telemetry_capability.recommended_period_ms.has_value() &&
                *telemetry_capability.recommended_period_ms < *telemetry_capability.minimum_period_ms) {
                return false;
            }
        }
        return true;
    }

    namespace {
        std::string quote(std::string_view value) {
            std::string result{"\""};
            for (const auto character : value) {
                if (character == '"') {
                    result += "\\\"";
                } else if (character == '\\') {
                    result += "\\\\";
                } else if (character == '\n') {
                    result += "\\n";
                } else {
                    result.push_back(character);
                }
            }
            result.push_back('"');
            return result;
        }

        std::string_view model_name(Model model) {
            switch (model) {
            case Model::kL6:
                return "L6";
            case Model::kO6:
                return "O6";
            case Model::kL20Lite:
                return "L20Lite";
            case Model::kL20:
                return "L20";
            case Model::kL30:
                return "L30";
            case Model::kO30:
                return "O30";
            case Model::kUnknown:
                break;
            }
            return "unknown";
        }

        std::string_view side_name(HandSide side) {
            if (side == HandSide::kLeft) {
                return "left";
            }
            if (side == HandSide::kRight) {
                return "right";
            }
            return "unknown";
        }

        void append_strings(std::ostringstream &out, const std::vector<std::string_view> &values) {
            out << '[';
            for (size_t index = 0; index < values.size(); ++index) {
                if (index != 0) {
                    out << ',';
                }
                out << quote(values[index]);
            }
            out << ']';
        }

        void append_ranges(std::ostringstream &out, const std::vector<NumericRange> &ranges) {
            out << '[';
            for (size_t index = 0; index < ranges.size(); ++index) {
                if (index != 0) {
                    out << ',';
                }
                const auto &range = ranges[index];
                out << "{\"minimum\":" << range.minimum << ",\"maximum\":" << range.maximum
                    << ",\"integer_only\":" << (range.integer_only ? "true" : "false") << '}';
            }
            out << ']';
        }
    }

    std::string DeviceCapabilities::serialize() const { return serialize_capabilities(*this); }

    std::string to_json(const DeviceCapabilities &capabilities) { return serialize_capabilities(capabilities); }

    std::string serialize_capabilities(const DeviceCapabilities &capabilities) {
        std::ostringstream out;
        out << std::setprecision(17) << "{\"schema_version\":{\"major\":" << capabilities.schema_version.major
            << ",\"minor\":" << capabilities.schema_version.minor
            << "},\"model\":" << quote(model_name(capabilities.model))
            << ",\"side\":" << quote(side_name(capabilities.side))
            << ",\"transport\":{\"kind\":" << static_cast<unsigned>(capabilities.transport.kind) << ",\"request_id\":";
        if (capabilities.transport.request_id) {
            out << *capabilities.transport.request_id;
        } else {
            out << "null";
        }
        out << ",\"response_id\":";
        if (capabilities.transport.response_id) {
            out << *capabilities.transport.response_id;
        } else {
            out << "null";
        }
        out << ",\"id_mask\":" << capabilities.transport.id_mask << ",\"mtu\":" << capabilities.transport.mtu
            << ",\"extended_frame\":" << (capabilities.transport.extended_frame ? "true" : "false")
            << ",\"bit_rate_switch\":" << (capabilities.transport.bit_rate_switch ? "true" : "false")
            << ",\"addressing_note\":" << quote(capabilities.transport.addressing_note) << "},\"commands\":[";
        for (size_t index = 0; index < capabilities.commands.size(); ++index) {
            if (index != 0) {
                out << ',';
            }
            const auto &command_capability = capabilities.commands[index];
            out << "{\"semantic\":" << static_cast<unsigned>(command_capability.semantic) << ",\"channels\":";
            append_strings(out, command_capability.channels);
            out << ",\"raw_encoding\":" << quote(command_capability.raw_encoding) << ",\"raw_ranges\":";
            append_ranges(out, command_capability.raw_ranges);
            out << ",\"normalized_ranges\":";
            append_ranges(out, command_capability.normalized_ranges);
            out << ",\"byte_order\":" << static_cast<unsigned>(command_capability.byte_order)
                << ",\"signedness\":" << static_cast<unsigned>(command_capability.signedness)
                << ",\"packing\":" << static_cast<unsigned>(command_capability.packing)
                << ",\"write_mode\":" << static_cast<unsigned>(command_capability.write_mode)
                << ",\"timeout_ms\":" << command_capability.timeout_ms
                << ",\"ack_type\":" << static_cast<unsigned>(command_capability.ack_support)
                << ",\"response\":" << static_cast<unsigned>(command_capability.response)
                << ",\"idempotency\":" << static_cast<unsigned>(command_capability.idempotency) << '}';
        }
        out << "],\"telemetry\":[";
        for (size_t index = 0; index < capabilities.telemetry.size(); ++index) {
            if (index != 0) {
                out << ',';
            }
            const auto &telemetry_capability = capabilities.telemetry[index];
            out << "{\"semantic\":" << static_cast<unsigned>(telemetry_capability.semantic) << ",\"channels\":";
            append_strings(out, telemetry_capability.channels);
            out << ",\"read_mode\":" << static_cast<unsigned>(telemetry_capability.read_mode)
                << ",\"source\":" << static_cast<unsigned>(telemetry_capability.source)
                << ",\"can_query\":" << (telemetry_capability.can_query ? "true" : "false")
                << ",\"can_report\":" << (telemetry_capability.can_report ? "true" : "false")
                << ",\"can_stream\":" << (telemetry_capability.can_stream ? "true" : "false")
                << ",\"recommended_period_ms\":";
            if (telemetry_capability.recommended_period_ms) {
                out << *telemetry_capability.recommended_period_ms;
            } else {
                out << "null";
            }
            out << ",\"minimum_period_ms\":";
            if (telemetry_capability.minimum_period_ms) {
                out << *telemetry_capability.minimum_period_ms;
            } else {
                out << "null";
            }
            out << ",\"freshness_ms\":";
            if (telemetry_capability.freshness_ms) {
                out << *telemetry_capability.freshness_ms;
            } else {
                out << "null";
            }
            out << ",\"shape\":[";
            for (size_t dimension = 0; dimension < telemetry_capability.shape.size(); ++dimension) {
                if (dimension != 0) {
                    out << ',';
                }
                out << telemetry_capability.shape[dimension];
            }
            out << "]}";
        }
        out << "],\"force_fields\":[";
        const auto force_fields = capabilities.force_fields();
        for (size_t index = 0; index < force_fields.size(); ++index) {
            if (index) {
                out << ',';
            }
            const auto &field = force_fields[index];
            out << "{\"name\":";
            out << quote(field.name);
            out << ",\"supported\":";
            if (field.supported) {
                out << (*field.supported ? "true" : "false");
            } else {
                out << "null";
            }
            out << ",\"query_channel\":";
            if (field.query_channel) {
                out << static_cast<unsigned>(*field.query_channel);
            } else {
                out << "null";
            }
            out << ",\"shared_fields\":";
            append_strings(out, field.shared_fields);
            out << ",\"sensor_names\":";
            append_strings(out, field.sensor_names);
            out << '}';
        }
        out << "],\"operations\":{"
            << "\"read_device_info\":" << (capabilities.operations.read_device_info ? "true" : "false")
            << ",\"clear_faults\":" << (capabilities.operations.clear_faults ? "true" : "false")
            << ",\"enable\":" << (capabilities.operations.enable ? "true" : "false")
            << ",\"disable\":" << (capabilities.operations.disable ? "true" : "false")
            << ",\"save_parameters\":" << (capabilities.operations.save_parameters ? "true" : "false")
            << ",\"save_parameters_confirmed\":"
            << (capabilities.operations.save_parameters_confirmed ? "true" : "false")
            << ",\"calibrate\":" << (capabilities.operations.calibrate ? "true" : "false") << ",\"acknowledged\":[";
        for (size_t index = 0; index < capabilities.operations.acknowledged_operations.size(); ++index) {
            if (index != 0) {
                out << ',';
            }
            out << static_cast<unsigned>(capabilities.operations.acknowledged_operations[index]);
        }
        out << "]}}";
        return out.str();
    }

} // namespace linkerbot::hand
