#include "detail/common.hpp"

#include <cmath>
#include <limits>

#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/l30/l30.hpp>
#include <linkerbot/hand/o30/o30.hpp>

namespace linkerbot::hand::unified_detail {
    namespace {
        constexpr uint32_t kLeftClassicCanId = 0x28;
        constexpr uint32_t kRightClassicCanId = 0x27;

        [[nodiscard]] bool is_classic_model(Model model) noexcept {
            return model == Model::kL6 || model == Model::kO6 || model == Model::kL20Lite || model == Model::kL20;
        }
    } // namespace

    [[nodiscard]] const char *model_name(Model model) noexcept {
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
            return "Unknown";
        }
        return "Unknown";
    }

    [[nodiscard]] uint32_t classic_address(HandSide side) {
        switch (side) {
        case HandSide::kLeft:
            return kLeftClassicCanId;
        case HandSide::kRight:
            return kRightClassicCanId;
        case HandSide::kUnknown:
            break;
        }
        throw ValidationError("classic CAN hand models require side Left or Right");
    }

    void validate_config(const HandDeviceConfig &config) {
        if (config.model == Model::kUnknown) {
            throw ValidationError("HandDevice model must not be Unknown");
        }
        if (config.interface_name.empty()) {
            throw ValidationError("HandDevice interface_name must not be empty");
        }
        if (config.polling_interval.count() <= 0) {
            throw ValidationError("HandDevice polling_interval must be positive");
        }
        const long double interval_ticks =
            static_cast<long double>(HealthClock::period::den) / HealthClock::period::num / config.command_rate_hz;
        if (!std::isfinite(config.command_rate_hz) || config.command_rate_hz <= 0 ||
            interval_ticks >= static_cast<long double>(std::numeric_limits<HealthClock::rep>::max())) {
            throw ValidationError("HandDevice command_rate_hz must have a representable positive interval");
        }
        if (config.discrete_queue_capacity == 0) {
            throw ValidationError("HandDevice discrete_queue_capacity must be positive");
        }
        if (config.command_timeout.count() <= 0) {
            throw ValidationError("HandDevice command_timeout must be positive");
        }
        if (config.identity_timeout.count() <= 0) {
            throw ValidationError("HandDevice identity_timeout must be positive");
        }
        if (config.transaction_timeout.count() <= 0) {
            throw ValidationError("HandDevice transaction_timeout must be positive");
        }
        if (config.shutdown_timeout.count() <= 0) {
            throw ValidationError("HandDevice shutdown_timeout must be positive");
        }
        if (config.recovery_attempts == 0) {
            throw ValidationError("HandDevice recovery_attempts must be positive");
        }
        if (config.recovery_backoff.count() < 0) {
            throw ValidationError("HandDevice recovery_backoff must not be negative");
        }

        if (config.o30_layout != O30Layout::kMountedV36 && config.o30_layout != O30Layout::kPythonV25) {
            throw ValidationError("unknown O30 layout");
        }
        if (config.o30_layout == O30Layout::kPythonV25 && config.address.value_or(1) > 0x3FE) {
            throw ValidationError("O30 V25 request address must be 0..0x3FE");
        }
        if (config.o30_layout == O30Layout::kPythonV25 && config.model != Model::kO30) {
            throw ValidationError("V25 layout is only valid for O30");
        }

        // Also validates that enum values are known without performing I/O.
        static_cast<void>(declared_capabilities(config));

        if (config.l6_protocol == L6Protocol::kHopV004) {
            (void)classic_address(config.side); // L6 still requires an explicit side.
            if (config.host_address || config.auto_start_periodic_reports || config.angle_mapping_path) {
                throw ValidationError("L6 HOP does not support host_address, periodic reports or 8-bit angle mappings");
            }
            return;
        }

        if (is_classic_model(config.model)) {
            const auto expected = classic_address(config.side);
            if (config.address && *config.address != expected) {
                throw ValidationError("classic CAN address does not match the configured side");
            }
            if (config.host_address) {
                throw ValidationError("host_address is only valid for L30");
            }
            if (config.auto_start_periodic_reports) {
                throw ValidationError("auto_start_periodic_reports is only valid for L30");
            }
            return;
        }

        if (config.angle_mapping_path) {
            throw ValidationError("angle_mapping_path is only valid for classic CAN models");
        }

        if (config.model == Model::kL30) {
            const auto node_id = config.address.value_or(l30::kDefaultNodeId);
            const auto host_id = config.host_address.value_or(l30::kDefaultHostId);
            if (node_id < 1 || node_id > 31 || host_id > 31) {
                throw ValidationError("L30 address must be 1..31 and host_address must be 0..31");
            }
            return;
        }

        if (config.host_address) {
            throw ValidationError("host_address is only valid for L30");
        }
        if (config.auto_start_periodic_reports) {
            throw ValidationError("auto_start_periodic_reports is only valid for L30");
        }
        if (config.address.value_or(o30::protocol::kDefaultRequestId) > 0x3FF) {
            throw ValidationError("O30 request address must be in range 0..0x3FF");
        }
    }

    [[nodiscard]] bool polling_supported(Model model, TelemetrySemantic semantic) {
        switch (semantic) {
        case TelemetrySemantic::kAngles:
        case TelemetrySemantic::kCurrents:
        case TelemetrySemantic::kTemperatures:
        case TelemetrySemantic::kFaults:
        case TelemetrySemantic::kForce:
            return true;
        case TelemetrySemantic::kForceFns:
        case TelemetrySemantic::kForceFts:
        case TelemetrySemantic::kForceFds:
        case TelemetrySemantic::kForcePx:
            return model == Model::kL20Lite;
        case TelemetrySemantic::kCurrentLimits:
        case TelemetrySemantic::kTorques:
            return model == Model::kL6 || model == Model::kO30;
        default:
            return false;
        }
    }
} // namespace linkerbot::hand::unified_detail
