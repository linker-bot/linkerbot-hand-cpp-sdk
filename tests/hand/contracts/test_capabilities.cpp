#include "hand/support/optional_value.hpp"
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include <linkerbot/can/fake_transport.hpp>
#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/hand/l20/l20.hpp>
#include <linkerbot/hand/l20lite/l20lite.hpp>
#include <linkerbot/hand/l30/l30.hpp>
#include <linkerbot/hand/l6/joints.hpp>
#include <linkerbot/hand/l6/l6.hpp>
#include <linkerbot/hand/o30/o30.hpp>
#include <linkerbot/hand/o6/joints.hpp>
#include <linkerbot/hand/o6/o6.hpp>

#include "hand/support/temp_path.hpp"

using namespace linkerbot::hand;

namespace {
    constexpr std::array<Model, 6> kModels = {Model::kL6,  Model::kO6,  Model::kL20Lite,
                                              Model::kL20, Model::kL30, Model::kO30};

    std::vector<std::string_view> expected_joint_names(Model model) {
        switch (model) {
        case Model::kL6:
        case Model::kO6:
            return {"thumb_flex", "thumb_abd", "index", "middle", "ring", "pinky"};
        case Model::kL20Lite:
            return {"thumb_flex", "thumb_roll", "index_flex", "middle_flex", "ring_flex",
                    "pinky_flex", "index_abd",  "ring_abd",   "pinky_abd",   "thumb_rotation"};
        case Model::kL20:
            return {"thumb_rotation", "thumb_roll", "thumb_root",  "thumb_tip",  "index_abd", "index_root",
                    "index_tip",      "middle_abd", "middle_root", "middle_tip", "ring_abd",  "ring_root",
                    "ring_tip",       "pinky_abd",  "pinky_root",  "pinky_tip"};
        case Model::kL30:
            return {"thumb_root", "thumb_tip",   "thumb_abd",  "thumb_rotation", "ring_abd",  "ring_tip",
                    "ring_root",  "middle_root", "middle_tip", "pinky_root",     "pinky_tip", "pinky_abd",
                    "middle_abd", "index_abd",   "index_root", "index_tip",      "wrist"};
        case Model::kO30:
            return {"thumb_rotation", "thumb_roll",  "index_abd",    "middle_abd",   "ring_abd",
                    "pinky_abd",      "thumb_root1", "index_root1",  "middle_root1", "ring_root1",
                    "pinky_root1",    "index_root2", "middle_root2", "ring_root2",   "pinky_root2",
                    "thumb_tip",      "index_tip",   "middle_tip",   "ring_tip",     "pinky_tip"};
        case Model::kUnknown:
            break;
        }
        throw std::invalid_argument("unknown model in test");
    }

    void validate_profile(const DeviceCapabilities &profile) {
        REQUIRE(profile.validate());
        REQUIRE(profile.supports(DeviceOperation::kReadDeviceInfo));
        REQUIRE(profile.supports(DeviceOperation::kClearFaults) ==
                (profile.model == Model::kL6 || profile.model == Model::kL20));
        REQUIRE(profile.supports(DeviceOperation::kEnable) == (profile.model == Model::kL30));
        REQUIRE(profile.supports(DeviceOperation::kDisable) == (profile.model == Model::kL30));
        REQUIRE(profile.supports(DeviceOperation::kSaveParameters) ==
                (profile.model == Model::kL6 || profile.model == Model::kO6));

        const auto expected = expected_joint_names(profile.model);
        REQUIRE(profile.joints.size() == expected.size());

        std::set<std::string_view> joint_names;
        for (size_t index = 0; index < profile.joints.size(); ++index) {
            const auto &joint = profile.joints[index];
            REQUIRE(joint.index == index);
            REQUIRE(joint.name == expected[index]);
            REQUIRE(joint_names.insert(joint.name).second);
            REQUIRE(joint.raw_angle_range.contains(joint.raw_angle_range.minimum));
            REQUIRE(joint.raw_angle_range.contains(joint.raw_angle_range.maximum));
            REQUIRE(joint.percent_mapping.has_value());
            REQUIRE(joint.raw_angle_range.contains(test_support::require_value(joint.percent_mapping).raw_at_zero));
            REQUIRE(joint.raw_angle_range.contains(test_support::require_value(joint.percent_mapping).raw_at_hundred));
            REQUIRE(test_support::require_value(joint.percent_mapping).raw_at_zero !=
                    test_support::require_value(joint.percent_mapping).raw_at_hundred);
        }

        std::set<CommandSemantic> command_semantics;
        for (const auto &command : profile.commands) {
            REQUIRE(command_semantics.insert(command.semantic).second);
            REQUIRE_FALSE(command.channels.empty());
            REQUIRE((command.raw_ranges.size() == 1 || command.raw_ranges.size() == command.channels.size()));
            for (const auto &range : command.raw_ranges) {
                REQUIRE(range.contains(range.minimum));
                REQUIRE(range.contains(range.maximum));
            }
            REQUIRE(profile.supports(command.semantic));
            REQUIRE(profile.command(command.semantic) == &command);
        }

        std::set<TelemetrySemantic> telemetry_semantics;
        for (const auto &telemetry : profile.telemetry) {
            REQUIRE(telemetry_semantics.insert(telemetry.semantic).second);
            REQUIRE_FALSE(telemetry.channels.empty());
            REQUIRE_FALSE(telemetry.shape.empty());
            for (const auto dimension : telemetry.shape) {
                REQUIRE(dimension > 0);
            }
            if (telemetry.raw_range.has_value()) {
                REQUIRE(test_support::require_value(telemetry.raw_range)
                            .contains(test_support::require_value(telemetry.raw_range).minimum));
                REQUIRE(test_support::require_value(telemetry.raw_range)
                            .contains(test_support::require_value(telemetry.raw_range).maximum));
            }
            REQUIRE(profile.supports(telemetry.semantic));
            REQUIRE(profile.telemetry_channel(telemetry.semantic) == &telemetry);
        }

        const auto *angles = profile.command(CommandSemantic::kAngles);
        REQUIRE(angles != nullptr);
        REQUIRE(angles->channels == expected);
        REQUIRE(angles->raw_ranges.size() == profile.joints.size());
        REQUIRE(angles->percent_mappings.size() == profile.joints.size());
        for (size_t index = 0; index < profile.joints.size(); ++index) {
            REQUIRE(angles->percent_mappings[index].has_value());
            REQUIRE(test_support::require_value(angles->percent_mappings[index]).raw_at_zero ==
                    test_support::require_value(profile.joints[index].percent_mapping).raw_at_zero);
            REQUIRE(test_support::require_value(angles->percent_mappings[index]).raw_at_hundred ==
                    test_support::require_value(profile.joints[index].percent_mapping).raw_at_hundred);
        }

        const auto *angle_telemetry = profile.telemetry_channel(TelemetrySemantic::kAngles);
        REQUIRE(angle_telemetry != nullptr);
        REQUIRE(angle_telemetry->channels == expected);
        REQUIRE(angle_telemetry->shape == std::vector<size_t>{profile.joints.size()});
    }

    DeviceCapabilities example_capabilities() {
        DeviceCapabilities capabilities;
        capabilities.model = Model::kL6;
        capabilities.side = HandSide::kLeft;
        capabilities.transport = {TransportKind::kCan, 1000000U, std::nullopt, false, "classic CAN endpoint"};
        capabilities.operations.read_device_info = true;
        capabilities.joints = {
            {0,
             "thumb_flex",
             JointMotionType::kFlexion,
             {0.0, 255.0, true, 1.0},
             PercentMapping{0.0, 255.0, "extended", "flexed", Confidence::kDocumented},
             {}},
        };
        capabilities.commands = {
            {CommandSemantic::kAngles,
             {"thumb_flex"},
             {{0.0, 255.0, true, 1.0}},
             "uint8",
             std::string_view{"count"},
             std::string_view{"0..255"},
             Confidence::kDocumented,
             AckSupport::kOptional,
             true},
            {CommandSemantic::kCurrentLimits,
             {"thumb_flex"},
             {{0.0, 100.0, true, 1.0}},
             "uint8",
             std::string_view{"count"},
             std::nullopt,
             Confidence::kDocumented,
             AckSupport::kOptional,
             false},
        };
        capabilities.telemetry = {
            {TelemetrySemantic::kAngles,
             {"thumb_flex"},
             ReadMode::kQuery,
             "uint8",
             {1},
             NumericRange{0.0, 255.0, true, 1.0},
             std::string_view{"count"},
             std::nullopt,
             Confidence::kDocumented,
             true,
             true},
            {TelemetrySemantic::kCurrents,
             {"thumb_flex"},
             ReadMode::kQuery,
             "uint8",
             {1},
             NumericRange{0.0, 255.0, true, 1.0},
             std::string_view{"count"},
             std::nullopt,
             Confidence::kUnknown,
             true,
             true},
        };
        return capabilities;
    }

    template <typename Hand, typename Side> void verify_classic_hand_capabilities(Model model, Side left, Side right) {
        static_assert(noexcept(std::declval<const Hand &>().capabilities()),
                      "concrete hand capability lookup must be noexcept");

        const auto verify_side = [model](Side side, HandSide expected_side) {
            auto transport = std::make_unique<linkerbot::can::FakeTransport>();
            auto *raw_transport = transport.get();
            Hand hand(side, std::move(transport), "fake", test_util::unique_mapping_path());

            REQUIRE(raw_transport->sent().empty());
            const auto &profile = hand.capabilities();
            REQUIRE(profile.model == model);
            REQUIRE(profile.side == expected_side);
            REQUIRE(&profile == &declared_capabilities(model, expected_side));
            REQUIRE(raw_transport->sent().empty());
        };

        struct SidePair {
            Side left;
            Side right;
        };
        const auto sides = SidePair{left, right};
        verify_side(sides.left, HandSide::kLeft);
        verify_side(sides.right, HandSide::kRight);
    }
}

TEST_CASE("capabilities: numeric ranges reject invalid values", "[capabilities]") {
    const NumericRange integers{0.0, 255.0, true, 1.0};
    REQUIRE(integers.contains(0.0));
    REQUIRE(integers.contains(255.0));
    REQUIRE_FALSE(integers.contains(-1.0));
    REQUIRE_FALSE(integers.contains(255.5));
    REQUIRE_FALSE(integers.contains(std::nan("")));

    const NumericRange invalid{2.0, 1.0, false, std::nullopt};
    REQUIRE_FALSE(invalid.contains(1.5));
}

TEST_CASE("capabilities: supported model list matches the registry", "[capabilities]") {
    REQUIRE(supported_models() == kModels);
    for (const auto model : supported_models()) {
        REQUIRE_NOTHROW(declared_capabilities(model));
    }
}

TEST_CASE("capabilities: operations describe the unified facade only", "[capabilities]") {
    auto capabilities = example_capabilities();
    REQUIRE(capabilities.supports(DeviceOperation::kReadDeviceInfo));
    REQUIRE_FALSE(capabilities.supports(DeviceOperation::kEnable));

    capabilities.operations.enable = true;
    capabilities.operations.acknowledged_operations = {DeviceOperation::kEnable};
    REQUIRE(capabilities.supports(DeviceOperation::kEnable));
    REQUIRE(capabilities.operations.acknowledged(DeviceOperation::kEnable));
    REQUIRE(capabilities.validate());

    capabilities.operations.enable = false;
    REQUIRE_FALSE(capabilities.validate());
}

TEST_CASE("capabilities: serialization includes operation contract", "[capabilities]") {
    const auto serialized = serialize_capabilities(declared_capabilities(Model::kL6));
    REQUIRE(serialized.find("\"operations\"") != std::string::npos);
    REQUIRE(serialized.find("\"read_device_info\":true") != std::string::npos);
    REQUIRE(serialized.find("\"ack_type\":0") != std::string::npos); // Angle writes do not require an ACK.
    REQUIRE(serialized.find("\"ack_type\":2") != std::string::npos); // Configuration writes require an ACK.
}

TEST_CASE("capabilities: command lookup keeps semantics separate", "[capabilities]") {
    const auto capabilities = example_capabilities();

    REQUIRE(capabilities.supports(CommandSemantic::kAngles));
    REQUIRE(capabilities.supports(CommandSemantic::kCurrentLimits));

    const auto *current_limits = capabilities.command(CommandSemantic::kCurrentLimits);
    REQUIRE(current_limits != nullptr);
    REQUIRE(current_limits->semantic == CommandSemantic::kCurrentLimits);
    REQUIRE(current_limits->range_for(0) != nullptr);
    REQUIRE(current_limits->range_for(0)->maximum == 100.0);
    REQUIRE(current_limits->range_for(1) == nullptr);
}

TEST_CASE("capabilities: telemetry lookup keeps semantics separate", "[capabilities]") {
    const auto capabilities = example_capabilities();

    REQUIRE(capabilities.supports(TelemetrySemantic::kAngles));
    REQUIRE(capabilities.supports(TelemetrySemantic::kCurrents));
    REQUIRE_FALSE(capabilities.supports(TelemetrySemantic::kCurrentLimits));
    REQUIRE_FALSE(capabilities.supports(TelemetrySemantic::kTorques));
    REQUIRE(capabilities.telemetry_channel(TelemetrySemantic::kTorques) == nullptr);

    const auto *currents = capabilities.telemetry_channel(TelemetrySemantic::kCurrents);
    REQUIRE(currents != nullptr);
    REQUIRE(currents->device_measured);
    REQUIRE(currents->device_queried);
}

TEST_CASE("capabilities: range lookup supports shared and per-channel ranges", "[capabilities]") {
    CommandCapability shared;
    shared.channels = {"first", "second"};
    shared.raw_ranges = {{0.0, 10.0, true, 1.0}};
    REQUIRE(shared.range_for(0) == shared.range_for(1));

    CommandCapability per_channel;
    per_channel.channels = {"first", "second"};
    per_channel.raw_ranges = {
        {0.0, 10.0, true, 1.0},
        {-5.0, 5.0, true, 1.0},
    };
    REQUIRE(per_channel.range_for(0)->minimum == 0.0);
    REQUIRE(per_channel.range_for(1)->minimum == -5.0);

    per_channel.raw_ranges.push_back({0.0, 1.0, true, 1.0});
    REQUIRE(per_channel.range_for(0) == nullptr);
}

TEST_CASE("capabilities: registry returns complete immutable model profiles", "[capabilities]") {
    for (const auto model : kModels) {
        const auto &unknown = declared_capabilities(model);
        const auto &unknown_again = declared_capabilities(model, HandSide::kUnknown);
        REQUIRE(&unknown == &unknown_again);
        REQUIRE(unknown.model == model);
        REQUIRE(unknown.side == HandSide::kUnknown);
        validate_profile(unknown);

        const auto &left = declared_capabilities(model, HandSide::kLeft);
        const auto &right = declared_capabilities(model, HandSide::kRight);
        REQUIRE(left.model == model);
        REQUIRE(left.side == HandSide::kLeft);
        REQUIRE(right.model == model);
        REQUIRE(right.side == HandSide::kRight);
        REQUIRE(&left == &declared_capabilities(model, HandSide::kLeft));
        REQUIRE(&right == &declared_capabilities(model, HandSide::kRight));
    }
}

TEST_CASE("capabilities: registry rejects unknown enum values", "[capabilities]") {
    REQUIRE_THROWS_AS(declared_capabilities(Model::kUnknown), std::invalid_argument);
    // Fixed-underlying-type enum values are valid C++17, including unknown values and bit masks.
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    REQUIRE_THROWS_AS(declared_capabilities(static_cast<Model>(255)), std::invalid_argument);
    // Fixed-underlying-type enum values are valid C++17, including unknown values and bit masks.
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    REQUIRE_THROWS_AS(declared_capabilities(Model::kL6, static_cast<HandSide>(255)), std::invalid_argument);
}

TEST_CASE("capabilities: L6 and O6 profiles reuse public joint metadata", "[capabilities]") {
    const auto verify = [](const DeviceCapabilities &profile, const auto &joint_names, double raw_minimum,
                           double raw_maximum) {
        REQUIRE(profile.joints.size() == joint_names.size());
        for (size_t index = 0; index < joint_names.size(); ++index) {
            const auto &joint = profile.joints[index];
            REQUIRE(joint.name == joint_names[index]);
            REQUIRE(joint.raw_angle_range.minimum == raw_minimum);
            REQUIRE(joint.raw_angle_range.maximum == raw_maximum);
        }
    };

    verify(declared_capabilities(Model::kL6), l6::kJointNames, l6::kRawAngleMinimum, l6::kRawAngleMaximum);
    verify(declared_capabilities(Model::kO6), o6::kJointNames, o6::kRawAngleMinimum, o6::kRawAngleMaximum);
}

TEST_CASE("capabilities: transport profiles match the implemented buses", "[capabilities]") {
    for (const auto model : {Model::kL6, Model::kO6, Model::kL20Lite, Model::kL20}) {
        const auto &transport = declared_capabilities(model).transport;
        REQUIRE(transport.kind == TransportKind::kCan);
        REQUIRE(transport.nominal_bitrate == 1000000U);
        REQUIRE_FALSE(transport.data_bitrate.has_value());
        REQUIRE_FALSE(transport.extended_frame);
    }

    const auto &l30 = declared_capabilities(Model::kL30).transport;
    REQUIRE(l30.kind == TransportKind::kCanFd);
    REQUIRE(l30.nominal_bitrate == 1000000U);
    REQUIRE(l30.data_bitrate == 5000000U);
    REQUIRE(l30.extended_frame);

    const auto &o30 = declared_capabilities(Model::kO30).transport;
    REQUIRE(o30.kind == TransportKind::kCanFd);
    REQUIRE(o30.nominal_bitrate == 1000000U);
    REQUIRE_FALSE(o30.data_bitrate.has_value());
    REQUIRE_FALSE(o30.extended_frame);
}

TEST_CASE("capabilities: new operations remain model scoped", "[comm][capabilities]") {
    for (const auto model : kModels) {
        const auto &profile = declared_capabilities(model, HandSide::kLeft);
        REQUIRE(profile.supports(DeviceOperation::kCalibrate) == (model == Model::kL30));
        REQUIRE(profile.operations.acknowledged(DeviceOperation::kCalibrate) == (model == Model::kL30));
        REQUIRE(profile.supports(DeviceOperation::kSaveParametersConfirmed) ==
                (model == Model::kL6 || model == Model::kO6));
        REQUIRE_FALSE(profile.supports(CommandSemantic::kTorqueLimits));
        REQUIRE_FALSE(profile.supports(CommandSemantic::kMotionTimes));
    }
}

TEST_CASE("capabilities: current and torque semantics remain independent", "[capabilities]") {
    for (const auto model : kModels) {
        const auto &profile = declared_capabilities(model);
        REQUIRE(profile.supports(CommandSemantic::kCurrentLimits));
        REQUIRE(profile.supports(TelemetrySemantic::kCurrents));
        REQUIRE(profile.supports(TelemetrySemantic::kCurrentLimits) == (model != Model::kL20));
    }

    const auto &l6 = declared_capabilities(Model::kL6);
    REQUIRE(l6.supports(TelemetrySemantic::kTorques));

    const auto &o30 = declared_capabilities(Model::kO30);
    REQUIRE(o30.supports(TelemetrySemantic::kTorques));

    for (const auto model : {Model::kO6, Model::kL20Lite, Model::kL20, Model::kL30}) {
        const auto &profile = declared_capabilities(model);
        REQUIRE_FALSE(profile.supports(TelemetrySemantic::kTorques));
    }
}

TEST_CASE("capabilities: cached configuration is not reported as measurement", "[capabilities]") {
    const auto &o6 = declared_capabilities(Model::kO6);
    const auto *o6_limits = o6.telemetry_channel(TelemetrySemantic::kCurrentLimits);
    REQUIRE(o6_limits != nullptr);
    REQUIRE(o6_limits->read_mode == ReadMode::kCommandResponse);
    REQUIRE_FALSE(o6_limits->device_measured);
    REQUIRE_FALSE(o6_limits->device_queried);

    const auto &l30 = declared_capabilities(Model::kL30);
    const auto *l30_limits = l30.telemetry_channel(TelemetrySemantic::kCurrentLimits);
    REQUIRE(l30_limits != nullptr);
    REQUIRE(l30_limits->read_mode == ReadMode::kCommandCache);
    REQUIRE_FALSE(l30_limits->device_measured);
    REQUIRE_FALSE(l30_limits->device_queried);
    const auto *l30_speeds = l30.telemetry_channel(TelemetrySemantic::kConfiguredSpeeds);
    REQUIRE(l30_speeds != nullptr);
    REQUIRE(l30_speeds->read_mode == ReadMode::kCommandCache);
    REQUIRE_FALSE(l30_speeds->device_measured);
    REQUIRE_FALSE(l30_speeds->device_queried);
    REQUIRE_FALSE(l30_speeds->can_query);
    REQUIRE_FALSE(l30_speeds->can_report);
    REQUIRE(test_support::require_value(l30_speeds->raw_range).minimum == 1);
    REQUIRE(test_support::require_value(l30_speeds->raw_range).maximum == 250);
    REQUIRE(l30_speeds->freshness_ms == 1000);
}

TEST_CASE("comm: L30 and O30 public percent directions agree across profiles and managers",
          "[comm][capabilities][percent-direction]") {
    for (const auto model : {Model::kL30, Model::kO30}) {
        for (const auto side : {HandSide::kUnknown, HandSide::kLeft, HandSide::kRight}) {
            const auto &profile = declared_capabilities(model, side);
            for (size_t i = 0; i < profile.joints.size(); ++i) {
                const auto &joint = profile.joints[i];
                const auto reversed =
                    model == Model::kL30 ? i == 0 || i == 1 || (i >= 5 && i <= 10) || i == 14 || i == 15 : i >= 6;
                const auto &mapping = test_support::require_value(joint.percent_mapping);
                REQUIRE(mapping.raw_at_zero ==
                        (reversed ? joint.raw_angle_range.maximum : joint.raw_angle_range.minimum));
                REQUIRE(mapping.raw_at_hundred ==
                        (reversed ? joint.raw_angle_range.minimum : joint.raw_angle_range.maximum));
                for (const float percent : {0.0F, 25.0F, 50.0F, 75.0F, 100.0F}) {
                    const auto expected = std::lround(mapping.raw_at_zero +
                                                      (mapping.raw_at_hundred - mapping.raw_at_zero) * percent / 100);
                    const auto raw = model == Model::kL30 ? l30::percent_to_raw(percent, l30::kJointSpecs[i])
                                                          : o30::joint_percent_to_raw(i, percent);
                    REQUIRE(raw == expected);
                }
            }
        }
    }
}

TEST_CASE("comm: concrete classic CAN hands expose side-specific capabilities", "[comm][capabilities]") {
    verify_classic_hand_capabilities<l6::L6>(Model::kL6, l6::Side::kLeft, l6::Side::kRight);
    verify_classic_hand_capabilities<o6::O6>(Model::kO6, o6::Side::kLeft, o6::Side::kRight);
    verify_classic_hand_capabilities<l20lite::L20lite>(Model::kL20Lite, l20lite::Side::kLeft, l20lite::Side::kRight);
    verify_classic_hand_capabilities<l20::L20>(Model::kL20, l20::Side::kLeft, l20::Side::kRight);
}

TEST_CASE("comm: address-only CAN FD hands do not guess their side", "[comm][capabilities]") {
    static_assert(noexcept(std::declval<const l30::L30 &>().capabilities()));
    static_assert(noexcept(std::declval<const o30::O30 &>().capabilities()));

    SECTION("L30") {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        auto *raw_transport = transport.get();
        l30::L30 hand(std::move(transport), 2, 3, false);

        REQUIRE(raw_transport->sent().empty());
        const auto &profile = hand.capabilities();
        REQUIRE(profile.model == Model::kL30);
        REQUIRE(profile.side == HandSide::kUnknown);
        REQUIRE(&profile == &declared_capabilities(Model::kL30, HandSide::kUnknown));
        REQUIRE(raw_transport->sent().empty());
    }

    SECTION("O30") {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        auto *raw_transport = transport.get();
        o30::O30 hand(std::move(transport), 0x123);

        REQUIRE(raw_transport->sent().empty());
        const auto &profile = hand.capabilities();
        REQUIRE(profile.model == Model::kO30);
        REQUIRE(profile.side == HandSide::kUnknown);
        REQUIRE(&profile == &declared_capabilities(Model::kO30, HandSide::kUnknown));
        REQUIRE(raw_transport->sent().empty());
    }
}
