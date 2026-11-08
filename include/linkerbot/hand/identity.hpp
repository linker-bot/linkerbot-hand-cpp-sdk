#pragma once

#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <linkerbot/hand/api/readings.hpp>
#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/hand/errors.hpp>

namespace linkerbot::hand {
    enum class IdentitySide { kLeft, kRight };

    struct FactoryIdentity {
        std::string product_code;
        std::string model;
        std::string serial_number;
        IdentitySide side;
    };

    // Normalized identity reported by a device and accepted by the SDK's
    // model/side/address validation. When a protocol reports no physical side,
    // an explicitly supplied side may resolve it; optional device identifiers
    // stay empty when the protocol does not report them.
    struct DeviceIdentity {
        Model model = Model::kUnknown;
        HandSide side = HandSide::kUnknown;
        std::optional<uint32_t> address;
        std::optional<std::string> product_code;
        std::optional<std::string> serial_number;
        std::optional<std::string> device_uid;
        std::optional<std::string> firmware_version;
        std::optional<std::string> protocol_name;
        std::optional<std::string> protocol_version;
        std::optional<std::string> hardware_version;
        std::optional<std::string> mechanical_version;
        std::chrono::steady_clock::time_point timestamp;
        // Populated by the explicit Python runtime policy; timestamp is monotonic.
        SampleMeta meta;
    };

    // Python classic_can.parse_factory_identity contract. No protocol I/O.
    [[nodiscard]] FactoryIdentity parse_factory_identity(std::string_view product_code,
                                                         std::initializer_list<std::string_view> expected_models,
                                                         IdentitySide expected_side);
    [[nodiscard]] FactoryIdentity parse_factory_identity(std::string_view product_code,
                                                         const std::vector<std::string_view> &expected_models,
                                                         IdentitySide expected_side);

    namespace identity_detail {
        std::string upper(std::string value);
        std::vector<std::string> split_product(std::string_view value);
        std::optional<IdentitySide> parse_side(std::string_view value);
    }
}
