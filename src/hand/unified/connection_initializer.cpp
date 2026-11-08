#include "detail/connection_initializer.hpp"

#include "detail/common.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include <linkerbot/hand/errors.hpp>

namespace linkerbot::hand::unified_detail {
    namespace {
        template <typename T>
        [[nodiscard]] bool known_identity_field_changed(const std::optional<T> &expected,
                                                        const std::optional<T> &current) {
            return expected && (!current || *expected != *current);
        }

        [[nodiscard]] bool physical_identity_changed(const DeviceIdentity &expected, const DeviceIdentity &current) {
            return known_identity_field_changed(expected.product_code, current.product_code) ||
                   known_identity_field_changed(expected.serial_number, current.serial_number) ||
                   known_identity_field_changed(expected.device_uid, current.device_uid);
        }
    } // namespace

    std::vector<TelemetrySemantic> baseline_semantics(Model model) {
        using Semantic = TelemetrySemantic;
        switch (model) {
        case Model::kL6:
            return {Semantic::kAngles, Semantic::kFaults, Semantic::kCurrentLimits};
        case Model::kO6:
        case Model::kL20Lite:
        case Model::kL20:
            return {Semantic::kAngles, Semantic::kFaults, Semantic::kCurrents};
        case Model::kL30:
            return {Semantic::kAngles, Semantic::kFaults};
        case Model::kO30:
            return {Semantic::kAngles, Semantic::kFaults, Semantic::kStallDetectionTimes, Semantic::kStallThresholds,
                    Semantic::kStallHoldCurrents};
        case Model::kUnknown:
            break;
        }
        throw ValidationError("cannot build an initialization baseline for an unknown model",
                              {"open", model_name(model)});
    }

    DeviceCapabilities capabilities_for_identity(const DeviceIdentity &identity, const HandDeviceConfig *config) {
        auto resolved = config ? *config : HandDeviceConfig{};
        resolved.model = identity.model;
        resolved.side = identity.side;
        auto capabilities = declared_capabilities(resolved);
        // Verified V4.6.10 supplies the five tactile matrices, but does not
        // implement category queries 0x20-0x23. Preserve fn and reject the
        // unavailable scalar APIs instead of timing out a working fn read.
        if (identity.model == Model::kL20Lite && identity.firmware_version == "V4.6.10") {
            auto &channels = capabilities.telemetry;
            channels.erase(std::remove_if(channels.begin(), channels.end(),
                                          [](const TelemetryCapability &channel) {
                                              return channel.semantic == TelemetrySemantic::kForceFns ||
                                                     channel.semantic == TelemetrySemantic::kForceFts ||
                                                     channel.semantic == TelemetrySemantic::kForceFds ||
                                                     channel.semantic == TelemetrySemantic::kForcePx;
                                          }),
                           channels.end());
        }
        return capabilities;
    }

    void validate_connection_identity(const DeviceIdentity &identity, const HandDeviceConfig &config,
                                      const DeviceIdentity *expected_identity, std::string_view operation) {
        const ErrorContext context{std::string(operation), model_name(config.model)};
        if (identity.model != config.model || (config.side != HandSide::kUnknown && identity.side != config.side)) {
            throw IdentityMismatchError("device identity does not match the HandDevice configuration", context);
        }
        if (expected_identity &&
            (expected_identity->model != identity.model || expected_identity->side != identity.side ||
             physical_identity_changed(*expected_identity, identity))) {
            throw IdentityMismatchError("connected physical device identity changed", context);
        }
    }

    ConnectionInitializationResult initialize_connection(DeviceBackend &backend, const HandDeviceConfig &config,
                                                         const DeviceCapabilities &declared,
                                                         const DeviceIdentity *expected_identity,
                                                         std::string_view operation) {
        auto identity = backend.initialize_identity(config.side, config.identity_timeout);
        validate_connection_identity(identity, config, expected_identity, operation);

        const ErrorContext context{std::string(operation), model_name(config.model)};

        auto verified = capabilities_for_identity(identity, &config);
        if (verified.model != declared.model) {
            throw ProtocolError("verified capabilities do not match declared capabilities", context);
        }

        const auto baseline = baseline_semantics(config.model);
        if (baseline.empty() || baseline.front() != TelemetrySemantic::kAngles) {
            throw ProtocolError("connection baseline must begin with angle telemetry", context);
        }

        std::set<TelemetrySemantic> unique;
        for (const auto semantic : baseline) {
            if (!unique.insert(semantic).second) {
                throw ProtocolError("connection baseline contains duplicate telemetry", context);
            }
            const auto *channel = verified.telemetry_channel(semantic);
            if (!channel || channel->read_mode == ReadMode::kReport || channel->read_mode == ReadMode::kCommandCache) {
                throw ProtocolError("connection baseline contains non-queryable telemetry", context);
            }

            const auto sample = backend.initialize_read(semantic, config.transaction_timeout);
            const auto valid = std::visit([](const auto &value) { return value.meta.valid; }, sample);
            if (!valid) {
                throw ProtocolError("connection baseline returned an invalid sample", context);
            }
        }

        backend.finish_initialization();
        return {std::move(identity), std::move(verified)};
    }
} // namespace linkerbot::hand::unified_detail
