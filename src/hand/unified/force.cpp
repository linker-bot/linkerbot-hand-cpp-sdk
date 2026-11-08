#include "detail/force.hpp"
#include "detail/freshness.hpp"
#include <linkerbot/hand/protocol/tactile.hpp>

#include <algorithm>

#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/o30/protocol.hpp>

namespace linkerbot::hand {
    std::vector<ForceFieldCapability> DeviceCapabilities::force_fields() const {
        if (!observed_force_fields.empty()) {
            return observed_force_fields;
        }
        std::vector<ForceFieldCapability> result;
        for (const auto name : kForceFields) {
            ForceFieldCapability field;
            field.name = name;
            const auto semantic = name == "fn"    ? TelemetrySemantic::kForce
                                  : name == "fns" ? TelemetrySemantic::kForceFns
                                  : name == "ft"  ? TelemetrySemantic::kForceFt
                                  : name == "fts" ? TelemetrySemantic::kForceFts
                                  : name == "fd"  ? TelemetrySemantic::kForceFd
                                  : name == "fds" ? TelemetrySemantic::kForceFds
                                                  : TelemetrySemantic::kForcePx;
            const auto *channel = telemetry_channel(semantic);
            const auto *native = telemetry_channel(TelemetrySemantic::kForce);
            if (native && (native->raw_encoding == "o30_tactile_descriptor" ||
                           native->raw_encoding == "hop_tactile_descriptor")) {
                field.supported = std::nullopt;
                field.query_channel = TelemetrySemantic::kForce;
                field.shared_fields.assign(kForceFields.begin(), kForceFields.end());
                channel = native;
            } else if (channel) {
                field.supported = true;
                field.query_channel = semantic;
                field.shared_fields = {name};
                // The current L20Lite backend receives all four scalar fields
                // in one native transaction; don't repeat that read per field.
                if (model == Model::kL20Lite && name != "fn") {
                    field.query_channel = TelemetrySemantic::kForceFns;
                    field.shared_fields = {"fns", "fts", "fds", "px"};
                }
            }
            if (channel) {
                field.sensor_names = channel->channels;
                field.raw_unit = channel->raw_unit;
                field.raw_encoding = channel->raw_encoding;
                field.raw_scale_note = channel->raw_scale_note;
            }
            result.push_back(std::move(field));
        }
        return result;
    }

    ForceFieldCapability DeviceCapabilities::force_field(std::string_view name) const {
        unified_detail::validate_force_field(name);
        for (auto &field : force_fields()) {
            if (field.name == name) {
                auto result = std::move(field);
                return result;
            }
        }
        throw ValidationError("unknown force field");
    }

    const ForceFieldSample *ForceSample::field(std::string_view name) const {
        unified_detail::validate_force_field(name);
        for (const auto &value : fields) {
            if (value.name == name) {
                return &value;
            }
        }
        return nullptr;
    }

    std::string_view ForceSample::status(std::string_view name) const {
        if (const auto *value = field(name)) {
            return value->meta.valid ? "available" : "invalidated";
        }
        return std::find(unsupported_fields.begin(), unsupported_fields.end(), name) != unsupported_fields.end()
                   ? "unsupported"
                   : "not_read";
    }
}

namespace linkerbot::hand::unified_detail {
    namespace {
        constexpr const char *kSensors[]{"thumb", "index", "middle", "ring", "pinky"};
    }

    void validate_force_field(std::string_view name) {
        if (std::find(kForceFields.begin(), kForceFields.end(), name) == kForceFields.end()) {
            throw ValidationError("force fields must be lowercase fn, fns, ft, fts, fd, fds or px");
        }
    }

    void merge_force_samples(ForceSample &target, const ForceSample &source) {
        if (target.fields.empty()) {
            target = source;
            return;
        }
        if (!target.meta.collected_at ||
            (source.meta.collected_at && *source.meta.collected_at > *target.meta.collected_at)) {
            target.meta = source.meta;
        }
        if (!source.tactile.empty() || !source.sensors.empty()) {
            target.raw = source.raw;
            target.shape = source.shape;
            target.tactile = source.tactile;
            target.sensors = source.sensors;
        }
        for (const auto &field : source.fields) {
            auto found = std::find_if(target.fields.begin(), target.fields.end(),
                                      [&](const auto &old) { return old.name == field.name; });
            if (found == target.fields.end()) {
                target.fields.push_back(field);
            } else {
                *found = field;
            }
        }
        target.unsupported_fields = source.unsupported_fields;
        target.unsupported_fields.erase(std::remove_if(target.unsupported_fields.begin(),
                                                       target.unsupported_fields.end(),
                                                       [&](const auto &name) { return target.field(name); }),
                                        target.unsupported_fields.end());
    }

    void merge_python_force_samples(ForceSample &target, const ForceSample &source) {
        if (target.fields.empty()) {
            target = source;
            return;
        }
        if (!target.meta.collected_at ||
            (source.meta.collected_at && *source.meta.collected_at > *target.meta.collected_at)) {
            target.meta = source.meta;
        }
        if (!source.tactile.empty() || !source.sensors.empty() || (target.tactile.empty() && target.sensors.empty())) {
            target.raw = source.raw;
            target.shape = source.shape;
            target.tactile = source.tactile;
            target.sensors = source.sensors;
        }
        std::vector<TelemetrySemantic> queries;
        for (const auto &field : source.fields) {
            queries.push_back(field.query_channel);
        }
        // Descriptor responses can stop exposing a previously present quantity.
        // All descriptor fields belong to the same physical force query.
        if (!source.sensors.empty() || !source.tactile.empty()) {
            queries.push_back(TelemetrySemantic::kForce);
        }
        target.fields.erase(std::remove_if(target.fields.begin(), target.fields.end(),
                                           [&](const auto &field) {
                                               return std::find(queries.begin(), queries.end(), field.query_channel) !=
                                                      queries.end();
                                           }),
                            target.fields.end());
        for (const auto &field : source.fields) {
            auto found = std::find_if(target.fields.begin(), target.fields.end(),
                                      [&](const auto &old) { return old.name == field.name; });
            if (found == target.fields.end()) {
                target.fields.push_back(field);
            } else {
                *found = field;
            }
        }
        for (const auto &name : source.unsupported_fields) {
            if (std::find(target.unsupported_fields.begin(), target.unsupported_fields.end(), name) ==
                target.unsupported_fields.end()) {
                target.unsupported_fields.push_back(name);
            }
        }
        target.unsupported_fields.erase(std::remove_if(target.unsupported_fields.begin(),
                                                       target.unsupported_fields.end(),
                                                       [&](const auto &name) { return target.field(name); }),
                                        target.unsupported_fields.end());
    }

    ForceSample tip_force_sample(const l20lite::TipForceData &data, SampleMeta meta,
                                 const DeviceCapabilities &capabilities) {
        ForceSample result;
        result.meta = meta;
        result.shape = {5, 4};
        const auto fingers = data.tip_force.to_array();
        for (const auto name : {"fns", "fts", "fds", "px"}) {
            ForceFieldSample field;
            field.name = name;
            field.meta = meta;
            field.query_channel = TelemetrySemantic::kForceFns;
            const auto capability = capabilities.force_field(name);
            if (capability.raw_scale_note) {
                field.scale_note = std::string{*capability.raw_scale_note};
            }
            for (size_t i = 0; i < fingers.size(); ++i) {
                const auto &finger = fingers[i];
                const uint8_t raw = field.name == "fns"   ? finger.normal
                                    : field.name == "fts" ? finger.tangential
                                    : field.name == "fds" ? finger.direction_raw
                                                          : finger.proximity;
                field.sensor_names.emplace_back(kSensors[i]);
                field.raw.push_back(std::vector<double>{static_cast<double>(raw)});
                field.valid_values.push_back(std::vector<bool>{field.name != "fds" || raw <= 127});
                field.units.push_back(capability.raw_unit ? std::optional<std::string>{*capability.raw_unit}
                                                          : std::nullopt);
                field.encodings.emplace_back("u8");
            }
            result.fields.push_back(std::move(field));
        }
        for (const auto &finger : fingers) {
            result.raw.insert(result.raw.end(),
                              {finger.normal, finger.tangential, finger.direction_raw, finger.proximity});
        }
        result.unsupported_fields = {"ft", "fd"};
        return result;
    }

    ForceSample descriptor_force_sample(const o30::SensorData &data, SampleMeta meta) {
        try {
            return hand::protocol::hop::descriptor_force_sample(data, std::move(meta));
        } catch (const hand::ProtocolError &error) {
            throw o30::protocol::ProtocolError(error.what());
        }
    }

    NormalForceResultantSample normal_force_resultant(const ForceSample &sample) {
        const auto *field = sample.field("fns");
        if (!field) {
            throw UnsupportedOperationError("device sensors do not expose fns");
        }
        NormalForceResultantSample result;
        result.meta = field->meta;
        result.sensor_names = field->sensor_names;
        result.units = field->units;
        for (const auto &finger : field->raw) {
            if (!finger || finger->size() != 1) {
                throw UnsupportedOperationError("normal force resultant requires one fns value per finger");
            }
            result.raw.push_back(finger->front());
        }
        return result;
    }

    void complete_force_meta(ForceSample &sample, uint64_t generation, const DeviceCapabilities &capabilities) {
        const auto now = std::chrono::steady_clock::now();
        for (auto &field : sample.fields) {
            field.meta.connection_generation = generation;
            if (field.meta.sequence == 0) {
                field.meta.sequence = sample.meta.sequence;
            }
            complete_freshness(field.meta, capabilities.telemetry_channel(field.query_channel), now);
        }
    }
}
