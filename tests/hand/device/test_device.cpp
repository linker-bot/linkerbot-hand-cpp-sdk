#include "hand/support/optional_value.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <linkerbot/can/fake_transport.hpp>
#include <linkerbot/can/message.hpp>
#include <linkerbot/hand/api/device.hpp>
#include <linkerbot/hand/conversion.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/l30/joints.hpp>
#include <linkerbot/hand/l30/protocol.hpp>
#include <linkerbot/hand/o30/protocol.hpp>
#include <linkerbot/transport/session.hpp>

#include "hand/l20/comm_helpers.hpp"
#include "hand/l30/comm_helpers.hpp"
#include "hand/o30/comm_helpers.hpp"
#include "hardware/hand_device_angle_baseline.hpp"

using namespace linkerbot::hand;

namespace {
    // Explicit migration barrier for tests that inspect protocol results. New
    // admission/scheduler tests below use handles directly without this helper.
    CommandHandle completed(CommandHandle handle) {
        if (!handle.wait_for(std::chrono::seconds{5})) {
            throw std::runtime_error("test command did not reach a terminal stage");
        }
        if (handle.error()) {
            std::rethrow_exception(handle.error());
        }
        if (handle.stage() == CommandStage::kSuperseded || handle.stage() == CommandStage::kCancelled) {
            throw std::runtime_error("test command did not succeed");
        }
        return handle;
    }
}

TEST_CASE("force selectors reject malformed names before opening a device", "[comm][force-fields]") {
    HandDeviceConfig config;
    config.model = Model::kL6;
    config.side = HandSide::kLeft;
    HandDevice device{config};
    REQUIRE_THROWS_AS(device.get_force({"Fn"}), ValidationError);
    REQUIRE_THROWS_AS(device.get_force({"fn"}, std::chrono::milliseconds{0}), ValidationError);
}

TEST_CASE("L6 declares response-only speed capabilities", "[comm][device][l6-controls]") {
    const auto &caps = declared_capabilities(Model::kL6, HandSide::kLeft);
    const auto *speed = caps.telemetry_channel(TelemetrySemantic::kConfiguredSpeeds);
    REQUIRE(speed != nullptr);
    REQUIRE(speed->read_mode == ReadMode::kCommandResponse);
    REQUIRE_FALSE(speed->device_measured);
    REQUIRE_FALSE(speed->device_queried);
}

namespace {
    using linkerbot::can::FakeTransport;
    using linkerbot::can::Message;

    constexpr std::array<Model, 6> kModels = {Model::kL6,  Model::kO6,  Model::kL20Lite,
                                              Model::kL20, Model::kL30, Model::kO30};

    [[nodiscard]] HandDeviceConfig config_for(Model model) {
        HandDeviceConfig config;
        config.model = model;
        config.side = model == Model::kL30 || model == Model::kO30 ? HandSide::kUnknown : HandSide::kLeft;
        config.interface_name = "fake0";
        return config;
    }

    [[nodiscard]] std::vector<Message> classic_angle_reply(Model model, const Message &request) {
        Message reply{};
        reply.id = request.id;
        reply.data[0] = request.data[0];

        size_t payload_size = 6;
        if (model == Model::kL20Lite) {
            if (request.dlc == 7 && request.data[0] >= 0xC5 && request.data[0] <= 0xC7) {
                return {request};
            }
            payload_size = request.data[0] == 0x01 ? 6 : request.data[0] == 0x04 ? 4 : 5;
        }

        reply.dlc = static_cast<uint8_t>(payload_size + 1);
        for (size_t index = 0; index < payload_size; ++index) {
            reply.data[index + 1] = request.dlc > 1 ? request.data[index + 1] : uint8_t{64};
        }
        return {reply};
    }

    void install_fake_telemetry(Model model, FakeTransport *transport, std::shared_ptr<o30_comm::HopEndpoint> &hop) {
        switch (model) {
        case Model::kL6:
        case Model::kO6:
        case Model::kL20Lite:
        case Model::kL20:
            transport->on_request(0x28,
                                  [model](const Message &request) { return classic_angle_reply(model, request); });
            return;
        case Model::kL30:
            transport->on_request(l30_comm::query_id(linkerbot::hand::l30::protocol::kSubPosition),
                                  l30_comm::i16_handler(l30_comm::filled_i16(0)));
            transport->on_request(l30_comm::query_id(linkerbot::hand::l30::protocol::kSubCurrent),
                                  l30_comm::i16_handler(l30_comm::filled_i16(2)));
            transport->on_request(l30_comm::query_id(linkerbot::hand::l30::protocol::kSubTemperature),
                                  l30_comm::u8_handler(l30_comm::filled_u8(25)));
            transport->on_request(l30_comm::query_id(linkerbot::hand::l30::protocol::kSubFault),
                                  l30_comm::u8_handler(l30_comm::filled_u8(0)));
            return;
        case Model::kO30:
            if (!hop) {
                hop = std::make_shared<o30_comm::HopEndpoint>();
            }
            hop->seed_defaults();
            hop->put(linkerbot::hand::o30::protocol::kMiPosition,
                     std::vector<uint8_t>(linkerbot::hand::o30::protocol::kU8ObjectBytes, 64));
            o30_comm::attach_hop(transport, hop);
            return;
        case Model::kUnknown:
            return;
        }
    }

    [[nodiscard]] std::string classic_identity_code(Model model) {
        switch (model) {
        case Model::kL6:
            return "LHL6-1-123-L";
        case Model::kO6:
            return "LHO6-1-123-L";
        case Model::kL20Lite:
            return "LHT10-1-123-L";
        case Model::kL20:
            return "LHT20-1-123-L";
        default:
            return {};
        }
    }

    void install_classic_identity(FakeTransport *transport, Model model, std::string code = {}, uint32_t address = 0x28,
                                  bool serve_telemetry = true,
                                  std::chrono::milliseconds identity_delay = std::chrono::milliseconds{0},
                                  std::array<uint8_t, 3> firmware = {1, 2, 3}) {
        if (code.empty()) {
            code = classic_identity_code(model);
        }
        transport->on_request(address, [model, code, serve_telemetry, identity_delay,
                                        firmware](const Message &request) {
            std::vector<Message> replies;
            if (request.data[0] == 0xC0) {
                if (identity_delay.count() > 0) {
                    std::this_thread::sleep_for(identity_delay);
                }
                const bool uses_offsets = model == Model::kL20Lite;
                const int chunks = uses_offsets ? 3 : 4;
                for (int chunk = 0; chunk < chunks; ++chunk) {
                    Message reply{};
                    reply.id = request.id;
                    reply.dlc = 8;
                    reply.data[0] = 0xC0;
                    reply.data[1] = static_cast<uint8_t>(uses_offsets ? chunk * 6 : chunk);
                    for (size_t index = 0; index < 6; ++index) {
                        const auto offset = static_cast<size_t>(chunk) * 6 + index;
                        reply.data[index + 2] = offset < code.size() ? static_cast<uint8_t>(code[offset]) : uint8_t{0};
                    }
                    replies.push_back(reply);
                }
                return replies;
            }

            if (request.data[0] == 0xC1 || request.data[0] == 0xC2 || request.data[0] == 0xC4) {
                Message reply{};
                reply.id = request.id;
                reply.dlc = static_cast<uint8_t>(model == Model::kL6 && request.data[0] == 0xC1 ? 5 : 4);
                reply.data[0] = request.data[0];
                reply.data[1] = 1;
                reply.data[2] = 2;
                reply.data[3] = 3;
                if (request.data[0] == 0xC2) {
                    std::copy(firmware.begin(), firmware.end(), reply.data.begin() + 1);
                }
                replies.push_back(reply);
                return replies;
            }

            return serve_telemetry ? classic_angle_reply(model, request) : std::vector<Message>{};
        });
    }

    void install_l30_identity(FakeTransport *transport) {
        namespace protocol = linkerbot::hand::l30::protocol;
        const std::vector<uint8_t> info{0x13, 0x00, 0x00, 0x00, 0x7B, 1, 2, 3, 2, 3, 4, 3, 4, 5, 1, 0, 'B', 'A'};
        const std::string product_code = "LHT30-06-123-L-B-3-A";
        transport->on_request(
            l30_comm::request_id(protocol::kParentConfig, protocol::kSubDeviceInfo, protocol::kAccessRead),
            l30_comm::payload_handler(info));
        transport->on_request(
            l30_comm::request_id(protocol::kParentConfig, protocol::kSubProductCode, protocol::kAccessRead),
            l30_comm::payload_handler(std::vector<uint8_t>(product_code.begin(), product_code.end())));
    }

    void install_fake_identity(Model model, FakeTransport *transport, std::shared_ptr<o30_comm::HopEndpoint> &hop) {
        switch (model) {
        case Model::kL6:
        case Model::kO6:
        case Model::kL20Lite:
        case Model::kL20:
            install_classic_identity(transport, model);
            return;
        case Model::kL30:
            install_l30_identity(transport);
            return;
        case Model::kO30:
            if (!hop) {
                hop = std::make_shared<o30_comm::HopEndpoint>();
                hop->seed_defaults();
                o30_comm::attach_hop(transport, hop);
            }
            hop->put_ascii(linkerbot::hand::o30::protocol::kMiProductInfo, 0xD9, 8, "left");
            return;
        case Model::kUnknown:
            return;
        }
    }

    void install_fake_device(Model model, FakeTransport *transport, std::shared_ptr<o30_comm::HopEndpoint> &hop) {
        install_fake_telemetry(model, transport, hop);
        install_fake_identity(model, transport, hop);
    }

    template <typename Predicate>
    [[nodiscard]] bool wait_until(Predicate predicate,
                                  std::chrono::milliseconds timeout = std::chrono::milliseconds{2000}) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (predicate()) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{5});
        }
        return predicate();
    }

    [[nodiscard]] std::vector<int32_t> maximum_raw_angles(const DeviceCapabilities &capabilities) {
        std::vector<int32_t> values;
        values.reserve(capabilities.joints.size());
        for (const auto &joint : capabilities.joints) {
            values.push_back(static_cast<int32_t>(joint.raw_angle_range.maximum));
        }
        return values;
    }

    [[nodiscard]] std::vector<int32_t> percent_raw_angles(const DeviceCapabilities &capabilities, float percent) {
        std::vector<int32_t> values;
        values.reserve(capabilities.joints.size());
        for (const auto &joint : capabilities.joints) {
            const auto &mapping = test_support::require_value(joint.percent_mapping);
            const auto raw = mapping.raw_at_zero +
                             static_cast<double>(percent) / 100.0 * (mapping.raw_at_hundred - mapping.raw_at_zero);
            switch (mapping.quantization) {
            case Quantization::kNearestTiesAwayFromZero:
                values.push_back(static_cast<int32_t>(std::lround(raw)));
                break;
            case Quantization::kFloor:
                values.push_back(static_cast<int32_t>(std::floor(raw)));
                break;
            case Quantization::kCeil:
                values.push_back(static_cast<int32_t>(std::ceil(raw)));
                break;
            }
        }
        return values;
    }

    void require_angle_command_values(Model model, const std::vector<Message> &frames,
                                      const std::vector<int32_t> &expected) {
        switch (model) {
        case Model::kL6:
        case Model::kO6:
            REQUIRE(frames.size() == 1);
            for (size_t index = 0; index < expected.size(); ++index) {
                REQUIRE(frames[0].data[index + 1] == expected[index]);
            }
            return;
        case Model::kL20Lite:
            REQUIRE(frames.size() == 2);
            for (size_t index = 0; index < 6; ++index) {
                REQUIRE(frames[0].data[index + 1] == expected[index]);
            }
            for (size_t index = 0; index < 4; ++index) {
                REQUIRE(frames[1].data[index + 1] == expected[index + 6]);
            }
            return;
        case Model::kL20: {
            REQUIRE(frames.size() == 5);
            constexpr std::array<size_t, 4> first_positions{1, 2, 3, 6};
            constexpr std::array<size_t, 3> other_positions{1, 3, 6};
            size_t joint = 0;
            for (const auto position : first_positions) {
                REQUIRE(frames[0].data[position] == expected[joint++]);
            }
            for (size_t frame = 1; frame < frames.size(); ++frame) {
                for (const auto position : other_positions) {
                    REQUIRE(frames[frame].data[position] == expected[joint++]);
                }
            }
            REQUIRE(joint == expected.size());
            return;
        }
        case Model::kL30:
            REQUIRE(frames.size() == 1);
            for (size_t index = 0; index < expected.size(); ++index) {
                const auto encoded = static_cast<uint16_t>(
                    (static_cast<uint16_t>(frames[0].data[2 + index * 2]) << 8U) | frames[0].data[3 + index * 2]);
                REQUIRE(static_cast<int16_t>(encoded) == expected[index]);
            }
            return;
        case Model::kO30: {
            REQUIRE(frames.size() == 3);
            size_t joint = 0;
            for (const auto &frame : frames) {
                const auto length = frame.data[2];
                for (size_t index = 0; index < length; ++index) {
                    REQUIRE(frame.data[index + 3] == expected[joint++]);
                }
            }
            REQUIRE(joint == expected.size());
            return;
        }
        case Model::kUnknown:
            FAIL("unknown model in HandDevice test");
        }
    }

    void require_full_angle_command(Model model, const std::vector<Message> &frames) {
        switch (model) {
        case Model::kL6:
            REQUIRE(frames.size() == 1);
            REQUIRE(frames[0].data[0] == 0x01);
            REQUIRE(frames[0].dlc == 7);
            for (size_t index = 1; index < 7; ++index) {
                REQUIRE(frames[0].data[index] == 0xFF);
            }
            return;
        case Model::kO6:
            REQUIRE(frames.size() == 1);
            REQUIRE(frames[0].data[0] == 0x01);
            REQUIRE(frames[0].dlc == 7);
            for (size_t index = 1; index < 7; ++index) {
                REQUIRE(frames[0].data[index] == 0xFF);
            }
            return;
        case Model::kL20Lite:
            REQUIRE(frames.size() == 2);
            REQUIRE(frames[0].data[0] == 0x01);
            REQUIRE(frames[0].dlc == 7);
            REQUIRE(frames[1].data[0] == 0x04);
            REQUIRE(frames[1].dlc == 5);
            for (size_t index = 1; index < frames[0].dlc; ++index) {
                REQUIRE(frames[0].data[index] == 0xFF);
            }
            for (size_t index = 1; index < frames[1].dlc; ++index) {
                REQUIRE(frames[1].data[index] == 0xFF);
            }
            return;
        case Model::kL20: {
            REQUIRE(frames.size() == 5);
            constexpr std::array<std::array<uint8_t, 7>, 5> expected{{
                {{0x41, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF}},
                {{0x42, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF}},
                {{0x43, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF}},
                {{0x44, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF}},
                {{0x45, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF}},
            }};
            for (size_t frame = 0; frame < frames.size(); ++frame) {
                REQUIRE(frames[frame].dlc == expected[frame].size());
                for (size_t byte = 0; byte < expected[frame].size(); ++byte) {
                    REQUIRE(frames[frame].data[byte] == expected[frame][byte]);
                }
            }
            return;
        }
        case Model::kL30:
            REQUIRE(frames.size() == 1);
            REQUIRE(frames[0].is_extended);
            REQUIRE(frames[0].is_fd);
            REQUIRE(frames[0].dlc == linkerbot::hand::l30::protocol::kVectorWireBytes);
            REQUIRE(frames[0].data[0] == linkerbot::hand::l30::protocol::kVectorPayloadBytes);
            REQUIRE(frames[0].data[1] == linkerbot::hand::l30::protocol::kSingleFrameTransaction);
            for (size_t index = 0; index < linkerbot::hand::l30::kJointCount; ++index) {
                const auto encoded = static_cast<uint16_t>(linkerbot::hand::l30::kJointSpecs[index].maximum);
                REQUIRE(frames[0].data[2 + index * 2] == static_cast<uint8_t>(encoded >> 8U));
                REQUIRE(frames[0].data[3 + index * 2] == static_cast<uint8_t>(encoded & 0xFFU));
            }
            return;
        case Model::kO30: {
            REQUIRE(frames.size() == 3);
            constexpr std::array<uint8_t, 3> starts{0x00, 0x05, 0x10};
            constexpr std::array<uint8_t, 3> lengths{1, 10, 9};
            size_t joints = 0;
            for (size_t frame = 0; frame < frames.size(); ++frame) {
                REQUIRE(frames[frame].is_fd);
                REQUIRE_FALSE(frames[frame].is_extended);
                REQUIRE(frames[frame].data[0] == linkerbot::hand::o30::protocol::kMiPosition);
                REQUIRE(frames[frame].data[1] == starts[frame]);
                REQUIRE(frames[frame].data[2] == lengths[frame]);
                for (size_t byte = 0; byte < lengths[frame]; ++byte) {
                    REQUIRE(frames[frame].data[3 + byte] == 0xFF);
                    ++joints;
                }
            }
            REQUIRE(joints == 20);
            return;
        }
        case Model::kUnknown:
            FAIL("unknown model in HandDevice test");
        }
    }
}

TEST_CASE("HandDevice validates configuration without opening transport", "[comm][device]") {
    size_t factory_calls = 0;
    HandDevice device(config_for(Model::kL6), [&factory_calls] {
        ++factory_calls;
        return std::make_unique<linkerbot::can::FakeTransport>();
    });

    REQUIRE(factory_calls == 0);
    REQUIRE_FALSE(device.is_open());
    REQUIRE(device.get_capabilities().model == Model::kL6);
    REQUIRE(device.get_health().lifecycle_state == LifecycleState::kClosed);
    REQUIRE(device.get_health().connection_state == ConnectionState::kClosed);
    REQUIRE_FALSE(device.get_identity().has_value());
    REQUIRE_FALSE(device.get_verified_capabilities().has_value());
    REQUIRE(factory_calls == 0);

    auto invalid = config_for(Model::kL6);
    invalid.side = HandSide::kUnknown;
    REQUIRE_THROWS_AS(HandDevice(invalid), ValidationError);

    invalid = config_for(Model::kL30);
    invalid.address = 32;
    REQUIRE_THROWS_AS(HandDevice(invalid), ValidationError);

    invalid = config_for(Model::kO30);
    invalid.address = 0x400;
    REQUIRE_THROWS_AS(HandDevice(invalid), ValidationError);
}

TEST_CASE("HandDevice selects every backend and has idempotent lifecycle", "[comm][device]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            size_t factory_calls = 0;
            linkerbot::can::FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            HandDevice device(config_for(model), [&] {
                ++factory_calls;
                auto next = std::make_unique<linkerbot::can::FakeTransport>();
                transport = next.get();
                hop.reset();
                install_fake_device(model, transport, hop);
                return next;
            });

            const auto declared = device.get_capabilities();
            REQUIRE(factory_calls == 0);
            REQUIRE(declared.model == model);
            device.open();
            const auto sent_after_first_open = transport->sent().size();
            device.open();
            REQUIRE(device.is_open());
            REQUIRE(factory_calls == 1);
            REQUIRE(transport->sent().size() == sent_after_first_open);
            REQUIRE(transport != nullptr);
            const auto initialized = device.get_snapshot();
            const auto verified = device.get_verified_capabilities();
            const auto capabilities = device.get_capabilities();
            REQUIRE(initialized.model == model);
            REQUIRE(verified.has_value());
            REQUIRE(capabilities.model == test_support::require_value(verified).model);
            REQUIRE(capabilities.side == test_support::require_value(verified).side);
            if (model == Model::kL30 || model == Model::kO30) {
                REQUIRE(declared.side == HandSide::kUnknown);
                REQUIRE(capabilities.side == HandSide::kLeft);
            }
            REQUIRE(initialized.side == test_support::require_value(verified).side);
            REQUIRE(initialized.device_info.has_value());
            REQUIRE(test_support::require_value(initialized.device_info).model == model);
            REQUIRE(initialized.connection_generation > 0);
            REQUIRE(initialized.snapshot_sequence > 0);
            REQUIRE(initialized.angles.has_value());
            REQUIRE(initialized.faults.has_value());
            REQUIRE(test_support::require_value(initialized.angles).meta.valid);
            // Earlier baseline channels can age while later multi-frame reads
            // complete. Freshness is tested separately; open() promises validity.
            REQUIRE(test_support::require_value(initialized.angles).meta.collected_at.has_value());
            REQUIRE(test_support::require_value(initialized.angles).meta.age.has_value());
            REQUIRE(test_support::require_value(initialized.angles).meta.connection_generation ==
                    initialized.connection_generation);
            REQUIRE(test_support::require_value(initialized.faults).meta.valid);
            REQUIRE(test_support::require_value(initialized.faults).meta.collected_at.has_value());
            REQUIRE(test_support::require_value(initialized.faults).meta.age.has_value());
            REQUIRE(test_support::require_value(initialized.faults).meta.connection_generation ==
                    initialized.connection_generation);
            switch (model) {
            case Model::kL6:
                REQUIRE(initialized.current_limits.has_value());
                REQUIRE(test_support::require_value(initialized.current_limits).meta.valid);
                break;
            case Model::kO6:
            case Model::kL20Lite:
            case Model::kL20:
                REQUIRE(initialized.currents.has_value());
                REQUIRE(test_support::require_value(initialized.currents).meta.valid);
                break;
            case Model::kL30:
                break;
            case Model::kO30:
                REQUIRE(initialized.stall_detection_times.has_value());
                REQUIRE(initialized.stall_thresholds.has_value());
                REQUIRE(initialized.stall_hold_currents.has_value());
                REQUIRE(test_support::require_value(initialized.stall_detection_times).meta.valid);
                REQUIRE(test_support::require_value(initialized.stall_thresholds).meta.valid);
                REQUIRE(test_support::require_value(initialized.stall_hold_currents).meta.valid);
                break;
            case Model::kUnknown:
                FAIL("unknown model in HandDevice lifecycle test");
            }

            const auto before_health = transport->sent().size();
            const auto health = device.get_health();
            REQUIRE(health.lifecycle_state != LifecycleState::kClosed);
            REQUIRE(health.connection_state != ConnectionState::kClosed);
            REQUIRE(health.identity_verified);
            REQUIRE_FALSE(health.channels.empty());
            REQUIRE(transport->sent().size() == before_health);

            const auto sent_before = transport->sent().size();
            const auto joint_count = capabilities.joints.size();
            REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(joint_count - 1, 0))), ValidationError);
            REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(joint_count + 1, 0))), ValidationError);
            auto over = maximum_raw_angles(capabilities);
            over.front() = static_cast<int32_t>(capabilities.joints.front().raw_angle_range.maximum) + 1;
            REQUIRE_THROWS_AS(completed(device.set_angles(over)), ValidationError);
            auto under = maximum_raw_angles(capabilities);
            under.front() = static_cast<int32_t>(capabilities.joints.front().raw_angle_range.minimum) - 1;
            REQUIRE_THROWS_AS(completed(device.set_angles(under)), ValidationError);
            REQUIRE(transport->sent().size() == sent_before);

            device.close();
            device.close();
            REQUIRE_FALSE(device.is_open());
            REQUIRE(device.get_health().lifecycle_state == LifecycleState::kClosed);

            device.open();
            REQUIRE(device.is_open());
            REQUIRE(factory_calls == 2);
            device.close();
        }
    }
}

TEST_CASE("HandDevice converts dynamic angle commands for every model", "[comm][device]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            auto config = config_for(model);
            // Opening performs paced baseline reads before the command assertions.
            // Give both paths scheduler headroom for multi-frame Windows I/O.
            config.transaction_timeout = std::chrono::seconds{1};
            config.command_timeout = std::chrono::seconds{1};
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });

            REQUIRE_NOTHROW(device.open());
            transport->clear_sent();
            const auto handle = completed(device.set_angles(maximum_raw_angles(device.get_capabilities())));
            REQUIRE(handle.done());
            REQUIRE(handle.stage() == (model == Model::kO30 ? CommandStage::kAcknowledged : CommandStage::kDispatched));
            REQUIRE(handle.ack_expectation() ==
                    (model == Model::kO30 ? AckExpectation::kRequired : AckExpectation::kNone));
            require_full_angle_command(model, transport->sent());
            device.close();
        }
    }
}

TEST_CASE("HandDevice converts percent angle commands for every model", "[comm][device][angles]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            auto config = config_for(model);
            // Opening performs paced baseline reads before the command assertions.
            // Give both paths scheduler headroom for multi-frame Windows I/O.
            config.transaction_timeout = std::chrono::seconds{1};
            config.command_timeout = std::chrono::seconds{1};
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });

            REQUIRE_NOTHROW(device.open());
            transport->clear_sent();
            const auto capabilities = device.get_capabilities();
            const auto joint_count = capabilities.joints.size();
            for (const auto percent : {0.0F, 50.0F, 100.0F}) {
                transport->clear_sent();
                const auto handle = completed(device.set_percent_angles(std::vector<float>(joint_count, percent)));

                REQUIRE(handle.done());
                REQUIRE(handle.family() == CommandFamily::kAngleTarget);
                REQUIRE(handle.stage() ==
                        (model == Model::kO30 ? CommandStage::kAcknowledged : CommandStage::kDispatched));
                REQUIRE(handle.ack_expectation() ==
                        (model == Model::kO30 ? AckExpectation::kRequired : AckExpectation::kNone));
                require_angle_command_values(model, transport->sent(), percent_raw_angles(capabilities, percent));
            }
            device.close();
        }
    }
}

TEST_CASE("HandDevice dispatches every declared non-angle command", "[comm][device][commands]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            auto config = config_for(model);
            // Opening performs paced baseline reads before the command assertions.
            // Give both paths scheduler headroom for multi-frame Windows I/O.
            config.transaction_timeout = std::chrono::seconds{1};
            config.command_timeout = std::chrono::seconds{1};
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });
            REQUIRE_NOTHROW(device.open());

            const auto capabilities = device.get_capabilities();
            for (const auto &command : capabilities.commands) {
                INFO("command semantic: " << static_cast<int>(command.semantic));
                if (command.semantic == CommandSemantic::kAngles) {
                    continue;
                }
                std::vector<double> values;
                values.reserve(command.channels.size());
                for (size_t index = 0; index < command.channels.size(); ++index) {
                    const auto *range = command.range_for(index);
                    REQUIRE(range != nullptr);
                    values.push_back(range->minimum);
                }

                std::optional<CommandHandle> handle;
                switch (command.semantic) {
                case CommandSemantic::kPositionKp:
                    handle = completed(device.set_position_kp(values));
                    break;
                case CommandSemantic::kPositionKd:
                    handle = completed(device.set_position_kd(values));
                    break;
                case CommandSemantic::kTorqueKd:
                    handle = completed(device.set_torque_kd(values));
                    break;
                case CommandSemantic::kTorques:
                    handle = completed(device.set_torques(values));
                    break;
                case CommandSemantic::kTorqueLimits:
                    handle = completed(device.set_torque_limits(values));
                    break;
                case CommandSemantic::kMotionTimes:
                    handle = completed(device.set_motion_times(values));
                    break;
                case CommandSemantic::kLimitCompensations:
                    handle = completed(device.set_limit_compensations(values));
                    break;
                case CommandSemantic::kSpeeds:
                    handle = completed(device.set_speeds(values));
                    break;
                case CommandSemantic::kAccelerations:
                    handle = completed(device.set_accelerations(values));
                    break;
                case CommandSemantic::kCurrentLimits:
                    handle = completed(device.set_current_limits(values));
                    break;
                case CommandSemantic::kStallDetectionTimes:
                    handle = completed(device.set_stall_detection_times(values));
                    break;
                case CommandSemantic::kStallThresholds:
                    handle = completed(device.set_stall_thresholds(values));
                    break;
                case CommandSemantic::kStallHoldCurrents:
                    handle = completed(device.set_stall_hold_currents(values));
                    break;
                case CommandSemantic::kAngles:
                    FAIL("angle command was not filtered");
                }
                REQUIRE(handle.has_value());
                *handle = completed(*handle);
                REQUIRE(handle->done());
                REQUIRE(handle->stage() == (command.ack_support == AckSupport::kRequired ? CommandStage::kAcknowledged
                                                                                         : CommandStage::kDispatched));
                REQUIRE(handle->family() != CommandFamily::kDiscreteOperation);
            }
            device.close();
        }
    }
}

TEST_CASE("HandDevice rejects an undeclared Manager command before I/O", "[comm][device][commands]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kL20), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL20, transport, hop);
        return next;
    });
    device.open();
    const auto sent_before = transport->sent().size();
    REQUIRE_THROWS_AS(completed(device.set_accelerations(std::vector<double>(16, 0.0))), UnsupportedOperationError);
    REQUIRE(transport->sent().size() == sent_before);
    device.close();
}

TEST_CASE("HandDevice preserves protocol raw values across legacy Manager units", "[comm][device][commands]") {
    SECTION("L6 speed raw counts survive the Manager percent bridge") {
        FakeTransport *transport = nullptr;
        std::shared_ptr<o30_comm::HopEndpoint> hop;
        HandDevice device(config_for(Model::kL6), [&] {
            auto next = std::make_unique<FakeTransport>();
            transport = next.get();
            install_fake_device(Model::kL6, transport, hop);
            return next;
        });
        device.open();
        transport->clear_sent();
        const std::vector<double> raw{0, 1, 64, 127, 128, 255};
        static_cast<void>(completed(device.set_speeds(raw)));
        const auto sent = transport->sent();
        REQUIRE(sent.size() == 1);
        for (size_t index = 0; index < raw.size(); ++index) {
            REQUIRE(sent[0].data[index + 1] == raw[index]);
        }
        device.close();
    }

    SECTION("O6 acceleration keeps the special raw zero encoding") {
        FakeTransport *transport = nullptr;
        std::shared_ptr<o30_comm::HopEndpoint> hop;
        HandDevice device(config_for(Model::kO6), [&] {
            auto next = std::make_unique<FakeTransport>();
            transport = next.get();
            install_fake_device(Model::kO6, transport, hop);
            return next;
        });
        device.open();
        transport->clear_sent();
        const std::vector<double> raw{0, 1, 2, 127, 253, 254};
        static_cast<void>(completed(device.set_accelerations(raw)));
        const auto sent = transport->sent();
        REQUIRE(sent.size() == 1);
        for (size_t index = 0; index < raw.size(); ++index) {
            REQUIRE(sent[0].data[index + 1] == raw[index]);
        }
        device.close();
    }
}

TEST_CASE("HandDevice exposes O30 Manager telemetry through typed getters", "[comm][device][readings]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kO30), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kO30, transport, hop);
        return next;
    });
    device.open();

    constexpr auto timeout = std::chrono::milliseconds{500};
    REQUIRE(device.get_speeds(timeout).raw.size() == 20);
    REQUIRE(device.get_configured_speeds(timeout).raw.size() == 20);
    REQUIRE(device.get_configured_accelerations(timeout).raw.size() == 20);
    REQUIRE(device.get_currents(timeout).raw.size() == 20);
    REQUIRE(device.get_current_limits(timeout).raw.size() == 20);
    REQUIRE(device.get_torques(timeout).raw.size() == 20);
    REQUIRE(device.get_temperatures(timeout).raw.size() == 20);
    REQUIRE(device.get_voltages(timeout).raw.size() == 20);
    REQUIRE(device.get_faults(timeout).raw.size() == 20);
    REQUIRE(device.get_stall_detection_times(timeout).raw.size() == 20);
    REQUIRE(device.get_stall_thresholds(timeout).raw.size() == 20);
    REQUIRE(device.get_stall_hold_currents(timeout).raw.size() == 20);
    device.close();
}

TEST_CASE("O30 hardware setpoint reader shares the adapter through open and recovery",
          "[comm][device][hardware-config]") {
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    size_t connections = 0;
    hardware_test::DeviceFixture fixture(config_for(Model::kO30), [&] {
        ++connections;
        auto transport = std::make_unique<FakeTransport>();
        install_fake_device(Model::kO30, transport.get(), hop);
        hop->put(o30::protocol::kMiPosition, std::vector<uint8_t>(o30::protocol::kU8ObjectBytes, 42), true);
        return transport;
    });
    auto *reader = fixture.setpoint_reader();
    const auto targets = hardware_test::o30_setpoint_pose(fixture.config(), reader);
    REQUIRE(targets.has_value());
    REQUIRE(test_support::require_value(targets).size() == 20);
    REQUIRE((test_support::require_value(targets))[0] == 42);
    auto &device = fixture.device();
    device.open();
    REQUIRE(device.get_angles().raw.size() == 20);
    REQUIRE(hardware_test::o30_setpoint_pose(fixture.config(), reader) == targets);
    REQUIRE(connections == 1);
    const auto generation = device.get_health().connection_generation;
    device.recover();
    REQUIRE(device.get_health().connection_generation > generation);
    REQUIRE(hardware_test::o30_setpoint_pose(fixture.config(), reader) == targets);
    REQUIRE(device.get_angles().raw.size() == 20);
    device.close();
}

TEST_CASE("HandDevice L20 current-limit replies are measured currents", "[comm][device][readings]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kL20), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL20, transport, hop);
        return next;
    });
    device.open();
    transport->on_request(l20_comm::kLeftId, l20_comm::five_handler(l20_comm::kLeftId, 0x51, 7));

    REQUIRE_THROWS_AS(device.get_current_limits(std::chrono::milliseconds{100}), UnsupportedOperationError);
    static_cast<void>(completed(device.set_current_limits(std::vector<double>(16, 64.0))));
    REQUIRE_THROWS_AS(device.get_current_limits(std::chrono::milliseconds{100}), UnsupportedOperationError);
    const auto currents = device.get_currents(std::chrono::milliseconds{500});
    REQUIRE(currents.raw == std::vector<double>(16, 7.0));
    REQUIRE(currents.meta.origin == TelemetryValueOrigin::kDeviceMeasured);
    REQUIRE_FALSE(device.get_snapshot().current_limits.has_value());
    device.close();
}

TEST_CASE("HandDevice dispatches declared discrete Manager operations", "[comm][device][operations]") {
    SECTION("classic hands expose only their implemented operations") {
        for (const auto model : {Model::kL6, Model::kO6, Model::kL20}) {
            DYNAMIC_SECTION("model " << static_cast<int>(model)) {
                FakeTransport *transport = nullptr;
                std::shared_ptr<o30_comm::HopEndpoint> hop;
                HandDevice device(config_for(model), [&] {
                    auto next = std::make_unique<FakeTransport>();
                    transport = next.get();
                    install_fake_device(model, transport, hop);
                    return next;
                });
                device.open();
                if (model == Model::kL6 || model == Model::kL20) {
                    REQUIRE(device.clear_faults().stage() == CommandStage::kDispatched);
                }
                if (model == Model::kL6 || model == Model::kO6) {
                    REQUIRE(device.save_parameters().stage() == CommandStage::kDispatched);
                }
                device.close();
            }
        }
    }

    SECTION("L30 acknowledged operations preserve their protocol result") {
        constexpr uint32_t kEnableId = 0x0220E100;
        constexpr uint32_t kDisableId = 0x02210100;
        FakeTransport *transport = nullptr;
        HandDevice device(config_for(Model::kL30), [&] {
            auto next = std::make_unique<FakeTransport>();
            transport = next.get();
            std::shared_ptr<o30_comm::HopEndpoint> unused;
            install_fake_device(Model::kL30, transport, unused);
            transport->on_request(kEnableId, l30_comm::ack_handler());
            transport->on_request(kDisableId, l30_comm::ack_handler());
            return next;
        });
        device.open();
        REQUIRE(device.enable().stage() == CommandStage::kAcknowledged);
        REQUIRE(device.disable().stage() == CommandStage::kAcknowledged);
        device.close();
    }
}

TEST_CASE("HandDevice rejects invalid percent angles before device I/O", "[comm][device][angles]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kL30), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL30, transport, hop);
        return next;
    });

    device.open();
    const auto joint_count = device.get_capabilities().joints.size();
    const auto sent_before = transport->sent().size();
    REQUIRE_THROWS_AS(completed(device.set_percent_angles(std::vector<float>(joint_count - 1, 50.0F))),
                      ValidationError);
    REQUIRE_THROWS_AS(completed(device.set_percent_angles(std::vector<float>(joint_count + 1, 50.0F))),
                      ValidationError);

    auto invalid = std::vector<float>(joint_count, 50.0F);
    invalid.front() = -0.01F;
    REQUIRE_THROWS_AS(completed(device.set_percent_angles(invalid)), ValidationError);
    invalid.front() = 100.01F;
    REQUIRE_THROWS_AS(completed(device.set_percent_angles(invalid)), ValidationError);
    invalid.front() = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(completed(device.set_percent_angles(invalid)), ValidationError);
    REQUIRE(transport->sent().size() == sent_before);
    device.close();
}

TEST_CASE("HandDevice reads unified angle samples for every model", "[comm][device]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            HandDevice device(config_for(model), [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });

            device.open();
            const auto sample = device.get_angles(std::chrono::milliseconds{500});
            REQUIRE(sample.meta.valid);
            REQUIRE(sample.meta.collected_at.has_value());
            REQUIRE(sample.raw.size() == device.get_capabilities().joints.size());
            REQUIRE(sample.percent.size() == sample.raw.size());
            REQUIRE(device.get_snapshot().angles.has_value());
            device.close();
        }
    }
}

TEST_CASE("HandDevice L30 configured speed reads only dispatched command cache", "[comm][device][configured-speed]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> unused;
    HandDevice device(config_for(Model::kL30), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL30, transport, unused);
        transport->on_request(l30_comm::query_id(l30::protocol::kSubSpeed),
                              l30_comm::i16_handler(l30_comm::filled_i16(7)));
        return next;
    });
    device.open();
    const auto sent_before = transport->sent().size();
    const auto empty = device.get_configured_speeds();
    REQUIRE_FALSE(empty.meta.valid);
    REQUIRE(empty.raw.empty());
    REQUIRE(transport->sent().size() == sent_before);

    std::vector<double> targets(17, 123);
    targets.front() = 1;
    targets.back() = 250;
    REQUIRE(completed(device.set_speeds(targets)).stage() == CommandStage::kDispatched);
    const auto sent_after_write = transport->sent().size();
    const auto sample = device.get_configured_speeds();
    REQUIRE(sample.raw == targets);
    REQUIRE(sample.meta.valid);
    REQUIRE_FALSE(sample.meta.stale);
    REQUIRE(sample.meta.origin == TelemetryValueOrigin::kCommandCache);
    REQUIRE(sample.meta.delivery == TelemetryDelivery::kCommandCache);
    REQUIRE(transport->sent().size() == sent_after_write);
    REQUIRE(device.get_speeds(std::chrono::milliseconds{500}).raw == std::vector<double>(17, 7));
    REQUIRE(device.get_configured_speeds().raw == targets);
    const auto snapshot = device.get_snapshot();
    REQUIRE(test_support::require_value(snapshot.configured_speeds).raw == targets);
    REQUIRE(test_support::require_value(snapshot.measured_speeds).raw == std::vector<double>(17, 7));
    REQUIRE(snapshot.joints.front().configured_speed_raw == 1);
    REQUIRE(snapshot.joints.back().configured_speed_raw == 250);
    auto invalid = targets;
    invalid.front() = 0;
    REQUIRE_THROWS_AS(completed(device.set_speeds(invalid)), ValidationError);
    REQUIRE(device.get_configured_speeds().raw == targets);
    device.close();
    device.open();
    REQUIRE_FALSE(device.get_configured_speeds().meta.valid);
    device.close();
}

TEST_CASE("HandDevice L30 and O30 percent endpoints match Python on every axis", "[comm][device][percent-direction]") {
    for (const auto model : {Model::kL30, Model::kO30}) {
        DYNAMIC_SECTION("model=" << static_cast<int>(model)) {
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            HandDevice device(config_for(model), [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });
            device.open();
            const auto count = model == Model::kL30 ? 17U : 20U;
            for (const auto percent : {0.0F, 25.0F, 50.0F, 75.0F, 100.0F}) {
                std::vector<int32_t> expected;
                for (size_t i = 0; i < count; ++i) {
                    const bool reversed =
                        model == Model::kL30 ? i == 0 || i == 1 || (i >= 5 && i <= 10) || i == 14 || i == 15 : i >= 6;
                    const double minimum = model == Model::kL30 ? l30::kJointSpecs[i].minimum : 0;
                    const double maximum = model == Model::kL30 ? l30::kJointSpecs[i].maximum : 255;
                    expected.push_back(static_cast<int32_t>(
                        std::lround(minimum + (maximum - minimum) * (reversed ? 100 - percent : percent) / 100)));
                }
                transport->clear_sent();
                static_cast<void>(completed(device.set_percent_angles(std::vector<float>(count, percent))));
                require_angle_command_values(model, transport->sent(), expected);
                if (model == Model::kL30) {
                    std::array<int16_t, 17> raw{};
                    std::copy(expected.begin(), expected.end(), raw.begin());
                    transport->on_request(l30_comm::query_id(l30::protocol::kSubPosition), l30_comm::i16_handler(raw));
                }
                const auto sample = device.get_angles(std::chrono::milliseconds{500});
                REQUIRE(sample.raw == std::vector<double>(expected.begin(), expected.end()));
                for (const auto actual : sample.percent) {
                    REQUIRE(actual == Catch::Approx(percent).margin(0.2));
                }
                REQUIRE(test_support::require_value(device.get_snapshot().angles).raw == sample.raw);
            }
            device.close();
        }
    }
}

TEST_CASE("HandDevice waits for cached snapshot revisions without device I/O", "[comm][device][snapshot]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kL30), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL30, transport, hop);
        return next;
    });

    device.open();
    const auto previous = device.get_snapshot();
    const auto sent_before = transport->sent().size();
    const auto unchanged = device.wait_for_snapshot(previous, std::chrono::milliseconds{10});
    REQUIRE(unchanged.connection_generation == previous.connection_generation);
    REQUIRE(unchanged.snapshot_sequence == previous.snapshot_sequence);
    REQUIRE(transport->sent().size() == sent_before);
    REQUIRE_THROWS_AS(device.wait_for_snapshot(previous, std::chrono::milliseconds{0}), ValidationError);

    std::exception_ptr read_error;
    std::thread reader([&] {
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
            static_cast<void>(device.get_angles(std::chrono::milliseconds{500}));
        } catch (...) {
            read_error = std::current_exception();
        }
    });
    const auto changed = device.wait_for_snapshot(previous, std::chrono::milliseconds{1000});
    reader.join();

    REQUIRE_FALSE(read_error);
    REQUIRE(changed.snapshot_sequence > previous.snapshot_sequence);
    REQUIRE(changed.angles.has_value());
    device.close();
}

TEST_CASE("HandDevice close wakes snapshot waiters", "[comm][device][snapshot]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kL30), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL30, transport, hop);
        return next;
    });

    device.open();
    const auto previous = device.get_snapshot();
    std::atomic<bool> waiting{false};
    std::exception_ptr wait_error;
    std::thread waiter([&] {
        waiting.store(true, std::memory_order_release);
        try {
            static_cast<void>(device.wait_for_snapshot(previous, std::chrono::seconds{5}));
        } catch (...) {
            wait_error = std::current_exception();
        }
    });

    REQUIRE(wait_until([&] { return waiting.load(std::memory_order_acquire); }));
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
    device.close();
    waiter.join();

    REQUIRE(wait_error);
    REQUIRE_THROWS_AS(std::rethrow_exception(wait_error), DeviceNotReadyError);
}

TEST_CASE("HandDevice marks samples stale using declared freshness", "[comm][device][snapshot]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kL30), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL30, transport, hop);
        return next;
    });

    device.open();
    static_cast<void>(device.get_angles(std::chrono::milliseconds{500}));
    REQUIRE_FALSE(test_support::require_value(device.get_snapshot().angles).meta.stale);
    const auto angle_health = [](const HandHealth &health) {
        const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                          [](const auto &metrics) { return metrics.channel == "angles"; });
        REQUIRE(channel != health.channels.end());
        return *channel;
    };
    REQUIRE_FALSE(angle_health(device.get_health()).data_age.stale);
    const auto sent_before = transport->sent().size();
    std::this_thread::sleep_for(std::chrono::milliseconds{125});
    const auto stale = device.get_snapshot();
    REQUIRE(test_support::require_value(stale.angles).meta.stale);
    REQUIRE(test_support::require_value(stale.angles).meta.age.has_value());
    const auto health = device.get_health();
    const auto channel = angle_health(health);
    REQUIRE(channel.data_age.valid == test_support::require_value(stale.angles).meta.valid);
    REQUIRE(channel.data_age.stale == test_support::require_value(stale.angles).meta.stale);
    REQUIRE(channel.data_age.age >= test_support::require_value(stale.angles).meta.age);
    REQUIRE(channel.data_age.generation == test_support::require_value(stale.angles).meta.connection_generation);
    REQUIRE(transport->sent().size() == sent_before);
    device.close();
}

TEST_CASE("HandDevice health applies angle and configuration freshness consistently", "[comm][device][snapshot]") {
    FakeTransport *transport = nullptr;
    HandDevice device(config_for(Model::kL6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    static_cast<void>(completed(device.set_speeds(std::vector<double>(6, 40))));
    const auto speed_health = [](const HandHealth &health) {
        const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                          [](const auto &metrics) { return metrics.channel == "speeds"; });
        REQUIRE(channel != health.channels.end());
        return *channel;
    };
    REQUIRE_FALSE(test_support::require_value(device.get_snapshot().configured_speeds).meta.stale);
    REQUIRE_FALSE(speed_health(device.get_health()).data_age.stale);
    const auto sent = transport->sent().size();
    std::this_thread::sleep_for(std::chrono::milliseconds{1100});
    const auto snapshot = device.get_snapshot();
    REQUIRE(test_support::require_value(snapshot.configured_speeds).meta.stale);
    REQUIRE(test_support::require_value(snapshot.angles).meta.stale);
    REQUIRE(speed_health(device.get_health()).data_age.stale);
    REQUIRE(transport->sent().size() == sent);
}

TEST_CASE("HandDevice L20Lite stall reads use complete acknowledged write cache", "[comm][device][stall]") {
    FakeTransport *transport = nullptr;
    auto config = config_for(Model::kL20Lite);
    config.command_timeout = std::chrono::milliseconds{30};
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL20Lite);
        return next;
    });
    device.open();
    const auto initial_sent = transport->sent().size();
    REQUIRE_THROWS_AS(device.get_stall_detection_times(), DeviceNotReadyError);
    REQUIRE_THROWS_AS(device.get_stall_thresholds(), DeviceNotReadyError);
    REQUIRE_THROWS_AS(device.get_stall_hold_currents(), DeviceNotReadyError);
    REQUIRE(transport->sent().size() == initial_sent);
    REQUIRE(completed(device.set_stall_detection_times(std::vector<double>(10, 50))).stage() ==
            CommandStage::kAcknowledged);
    REQUIRE(completed(device.set_stall_thresholds(std::vector<double>(10, 128))).stage() ==
            CommandStage::kAcknowledged);
    REQUIRE(completed(device.set_stall_hold_currents(std::vector<double>(10, 255))).stage() ==
            CommandStage::kAcknowledged);
    const auto snapshot = device.get_snapshot();
    const auto sent = transport->sent().size();
    REQUIRE(device.get_stall_detection_times().raw == std::vector<double>(10, 50));
    REQUIRE(device.get_stall_thresholds().raw == std::vector<double>(10, 128));
    REQUIRE(device.get_stall_hold_currents().raw == std::vector<double>(10, 255));
    REQUIRE(device.get_stall_detection_times().meta.collected_at ==
            test_support::require_value(snapshot.stall_detection_times).meta.collected_at);
    REQUIRE(device.get_stall_thresholds().meta.delivery == TelemetryDelivery::kCommandResponse);
    REQUIRE(transport->sent().size() == sent);
    transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
    const auto begin = std::chrono::steady_clock::now();
    REQUIRE_THROWS_AS(completed(device.set_stall_detection_times(std::vector<double>(10, 80))), TimeoutError);
    REQUIRE(std::chrono::steady_clock::now() - begin < std::chrono::milliseconds{300});
    REQUIRE(device.get_stall_detection_times().raw == std::vector<double>(10, 50));
    REQUIRE(device.get_stall_detection_times().meta.collected_at ==
            test_support::require_value(snapshot.stall_detection_times).meta.collected_at);
}

TEST_CASE("HandDevice polling populates snapshots and shuts down safely", "[comm][device]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            auto config = config_for(model);
            // Exercise restart after a complete sample, without cancelling the
            // next multi-frame read (which intentionally poisons its reply key).
            config.polling_interval = std::chrono::seconds{10};
            {
                HandDevice device(config, [&] {
                    auto next = std::make_unique<FakeTransport>();
                    transport = next.get();
                    install_fake_device(model, transport, hop);
                    return next;
                });
                device.open();
                const auto before_snapshot = device.get_snapshot();
                const auto before_polling = transport->sent().size();
                device.start_polling();

                REQUIRE(wait_until([transport, before_polling] { return transport->sent().size() > before_polling; }));
                const auto snapshot = device.wait_for_snapshot(before_snapshot, std::chrono::seconds{1});
                REQUIRE(snapshot.snapshot_sequence != before_snapshot.snapshot_sequence);
                const auto health = device.get_health();
                REQUIRE(snapshot.model == model);
                REQUIRE(snapshot.angles.has_value());
                REQUIRE(test_support::require_value(snapshot.angles).percent.size() ==
                        device.get_capabilities().joints.size());
                REQUIRE(test_support::require_value(snapshot.angles).meta.collected_at.has_value());
                REQUIRE(test_support::require_value(test_support::require_value(snapshot.angles).meta.collected_at) <=
                        snapshot.captured_at);
                for (const auto value : test_support::require_value(snapshot.angles).percent) {
                    REQUIRE(value.has_value());
                    REQUIRE(std::isfinite(test_support::require_value(value)));
                }
                REQUIRE(health.polling_active);
                const auto angles = std::find_if(health.channels.begin(), health.channels.end(),
                                                 [](const auto &channel) { return channel.channel == "angles"; });
                REQUIRE(angles != health.channels.end());
                REQUIRE(angles->data_age.valid);

                device.stop_polling();
                REQUIRE_NOTHROW(device.stop_polling());
                REQUIRE_FALSE(device.get_health().polling_active);

                const auto before_restart = transport->sent().size();
                device.start_polling();
                REQUIRE(wait_until([transport, before_restart] { return transport->sent().size() > before_restart; }));
                // Leave polling active: HandDevice destruction must stop the
                // worker, stream and owned transport without hanging.
            }
        }
    }
}

TEST_CASE("HandDevice reports operations on a closed device", "[comm][device]") {
    HandDevice device(config_for(Model::kL6), [] { return std::make_unique<linkerbot::can::FakeTransport>(); });

    REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(6, 50))), DeviceNotReadyError);
    REQUIRE_THROWS_AS(completed(device.set_percent_angles(std::vector<float>(6, 50.0F))), DeviceNotReadyError);
    REQUIRE_THROWS_AS(device.get_snapshot(), DeviceNotReadyError);
    HandSnapshot previous;
    REQUIRE_THROWS_AS(device.wait_for_snapshot(previous, std::chrono::milliseconds{10}), DeviceNotReadyError);
    REQUIRE_THROWS_AS(device.start_polling(), DeviceNotReadyError);
    REQUIRE_THROWS_AS(device.stop_polling(), DeviceNotReadyError);
    REQUIRE_THROWS_AS(device.verify_identity(), DeviceNotReadyError);
}

TEST_CASE("HandDevice verifies and caches identity for every model", "[comm][device][identity]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            auto config = config_for(model);
            config.side = HandSide::kLeft;
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });

            device.open();
            REQUIRE(device.get_identity().has_value());
            REQUIRE(device.get_verified_capabilities().has_value());
            REQUIRE(device.get_health().identity_verified);

            const auto identity = device.verify_identity(std::chrono::milliseconds{500});
            REQUIRE(identity.model == model);
            REQUIRE(identity.side == HandSide::kLeft);
            REQUIRE(identity.firmware_version.has_value());
            if (model == Model::kO30) {
                REQUIRE(identity.device_uid == std::optional<std::string>{"DEVICE-UID-O30"});
                REQUIRE(identity.protocol_name == std::optional<std::string>{"HOP"});
                REQUIRE(identity.protocol_version == std::optional<std::string>{"0.0.3"});
                REQUIRE_FALSE(identity.serial_number.has_value());
                REQUIRE_FALSE(identity.address.has_value());
            } else {
                REQUIRE(identity.serial_number == std::optional<std::string>{"123"});
                REQUIRE(identity.address.has_value());
                REQUIRE_FALSE(identity.protocol_version.has_value());
            }

            const auto sent_after_verification = transport->sent().size();
            const auto cached = device.get_identity();
            const auto capabilities = device.get_verified_capabilities();
            const auto public_capabilities = device.get_capabilities();
            const auto health = device.get_health();
            const auto snapshot = device.get_snapshot();
            REQUIRE(cached.has_value());
            REQUIRE(test_support::require_value(cached).model == model);
            REQUIRE(capabilities.has_value());
            REQUIRE(test_support::require_value(capabilities).model == model);
            REQUIRE(test_support::require_value(capabilities).side == HandSide::kLeft);
            REQUIRE(public_capabilities.model == test_support::require_value(capabilities).model);
            REQUIRE(public_capabilities.side == test_support::require_value(capabilities).side);
            REQUIRE(health.identity_verified);
            REQUIRE(snapshot.device_info.has_value());
            REQUIRE(test_support::require_value(snapshot.device_info).model == model);
            REQUIRE(transport->sent().size() == sent_after_verification);

            device.close();
            REQUIRE_FALSE(device.get_identity().has_value());
            REQUIRE_FALSE(device.get_verified_capabilities().has_value());
            REQUIRE_FALSE(device.get_health().identity_verified);
        }
    }
}

TEST_CASE("HandDevice faults when active identity refresh fails", "[comm][device][identity]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> unused;
    HandDevice device(config_for(Model::kL6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL6, transport, unused);
        return next;
    });

    device.open();
    REQUIRE(device.is_open());
    REQUIRE(device.get_identity().has_value());
    install_classic_identity(transport, Model::kL6, "LHL6-1-999-L");

    REQUIRE_THROWS_AS(device.verify_identity(std::chrono::milliseconds{500}), IdentityMismatchError);
    REQUIRE_FALSE(device.is_open());
    REQUIRE_FALSE(device.get_identity().has_value());
    REQUIRE_FALSE(device.get_verified_capabilities().has_value());
    REQUIRE(device.get_capabilities().model == Model::kL6);
    const auto health = device.get_health();
    REQUIRE(health.lifecycle_state == LifecycleState::kFaulted);
    REQUIRE(health.connection_state == ConnectionState::kFaulted);
    REQUIRE_FALSE(health.identity_verified);
    REQUIRE_THROWS_AS(device.get_snapshot(), DeviceNotReadyError);
    REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(6, 50))), DeviceNotReadyError);

    device.close();
    REQUIRE(device.get_health().lifecycle_state == LifecycleState::kClosed);
}

TEST_CASE("HandDevice does not cache a mismatched identity", "[comm][device][identity]") {
    FakeTransport *transport = nullptr;
    HandDevice device(config_for(Model::kL6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(next.get(), Model::kL6, "LHL6-1-123-R");
        return next;
    });

    REQUIRE_THROWS_AS(device.open(), IdentityMismatchError);
    REQUIRE_FALSE(device.is_open());
    REQUIRE_FALSE(device.get_identity().has_value());
    REQUIRE_FALSE(device.get_verified_capabilities().has_value());
    REQUIRE_FALSE(device.get_health().identity_verified);
}

TEST_CASE("HandDevice rolls back a failed baseline and can reopen", "[comm][device][identity]") {
    auto config = config_for(Model::kL6);
    config.transaction_timeout = std::chrono::milliseconds{20};
    size_t factory_calls = 0;
    HandDevice device(config, [&] {
        ++factory_calls;
        auto next = std::make_unique<FakeTransport>();
        if (factory_calls == 1) {
            install_classic_identity(next.get(), Model::kL6, {}, 0x28, false);
        } else {
            std::shared_ptr<o30_comm::HopEndpoint> unused;
            install_fake_device(Model::kL6, next.get(), unused);
        }
        return next;
    });

    REQUIRE_THROWS_AS(device.open(), TimeoutError);
    REQUIRE_FALSE(device.is_open());
    REQUIRE_FALSE(device.get_identity().has_value());
    REQUIRE_FALSE(device.get_verified_capabilities().has_value());
    REQUIRE(device.get_health().lifecycle_state == LifecycleState::kClosed);
    REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(6, 50))), DeviceNotReadyError);

    REQUIRE_NOTHROW(device.open());
    REQUIRE(factory_calls == 2);
    REQUIRE(device.is_open());
    REQUIRE(device.get_identity().has_value());
    REQUIRE(device.get_verified_capabilities().has_value());
    REQUIRE(device.get_snapshot().angles.has_value());
    device.close();
}

TEST_CASE("HandDevice keeps its physical identity constraint across close and failed reopen",
          "[comm][device][identity][lifecycle]") {
    size_t factory_calls = 0;
    HandDevice device(config_for(Model::kL6), [&]() {
        ++factory_calls;
        auto next = std::make_unique<FakeTransport>();
        std::shared_ptr<o30_comm::HopEndpoint> unused;
        install_fake_telemetry(Model::kL6, next.get(), unused);
        install_classic_identity(next.get(), Model::kL6, factory_calls == 2 ? "LHL6-1-999-L" : "LHL6-1-123-L");
        return std::unique_ptr<linkerbot::can::CANTransport>(std::move(next));
    });

    device.open();
    REQUIRE(test_support::require_value(device.get_identity()).serial_number == std::optional<std::string>{"123"});
    device.close();

    REQUIRE_THROWS_AS(device.open(), IdentityMismatchError);
    REQUIRE_FALSE(device.is_open());
    REQUIRE(factory_calls == 2);

    REQUIRE_NOTHROW(device.open());
    REQUIRE(factory_calls == 3);
    REQUIRE(test_support::require_value(device.get_identity()).serial_number == std::optional<std::string>{"123"});
    device.close();
}

TEST_CASE("HandDevice remains closed when identity verification times out", "[comm][device][identity]") {
    auto config = config_for(Model::kL6);
    config.identity_timeout = std::chrono::milliseconds{20};
    HandDevice device(config, [] { return std::make_unique<FakeTransport>(); });

    REQUIRE_THROWS_AS(device.open(), TimeoutError);
    REQUIRE_FALSE(device.is_open());
    REQUIRE_FALSE(device.get_identity().has_value());
    REQUIRE_FALSE(device.get_verified_capabilities().has_value());
    REQUIRE_THROWS_AS(device.get_snapshot(), DeviceNotReadyError);
}

TEST_CASE("HandDevice rejects commands while open is initializing", "[comm][device][lifecycle]") {
    std::atomic<FakeTransport *> transport{nullptr};
    HandDevice device(config_for(Model::kL6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport.store(next.get(), std::memory_order_release);
        install_classic_identity(next.get(), Model::kL6, {}, 0x28, true, std::chrono::milliseconds{100});
        return next;
    });

    std::exception_ptr open_error;
    std::thread opening([&] {
        try {
            device.open();
        } catch (...) {
            open_error = std::current_exception();
        }
    });

    REQUIRE(wait_until([&transport] {
        const auto current = transport.load(std::memory_order_acquire);
        return current != nullptr && !current->sent().empty();
    }));
    const auto current = transport.load(std::memory_order_acquire);
    const auto sent_before_cached_queries = current->sent().size();
    const auto opening_health = device.get_health();
    const auto opening_capabilities = device.get_capabilities();
    REQUIRE(opening_health.lifecycle_state == LifecycleState::kOpening);
    REQUIRE(opening_health.connection_state == ConnectionState::kConnecting);
    REQUIRE_FALSE(opening_health.identity_verified);
    REQUIRE_FALSE(device.is_open());
    REQUIRE(opening_capabilities.model == Model::kL6);
    REQUIRE(current->sent().size() == sent_before_cached_queries);
    REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(6, 50))), DeviceNotReadyError);
    opening.join();
    REQUIRE_FALSE(open_error);
    REQUIRE(device.is_open());
    REQUIRE(device.get_health().lifecycle_state == LifecycleState::kOpen);
    REQUIRE(device.get_health().connection_state == ConnectionState::kConnected);
    REQUIRE(device.get_health().identity_verified);
    device.close();
}

TEST_CASE("HandDevice serializes concurrent open close and command", "[comm][device][lifecycle]") {
    std::atomic<FakeTransport *> transport{nullptr};
    HandDevice device(config_for(Model::kL6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport.store(next.get(), std::memory_order_release);
        install_classic_identity(next.get(), Model::kL6, {}, 0x28, true, std::chrono::milliseconds{100});
        return next;
    });

    std::exception_ptr open_error;
    std::exception_ptr close_error;
    std::thread opening([&] {
        try {
            device.open();
        } catch (...) {
            open_error = std::current_exception();
        }
    });
    REQUIRE(wait_until([&] {
        const auto current = transport.load(std::memory_order_acquire);
        return current && !current->sent().empty();
    }));
    REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(6, 50))), DeviceNotReadyError);

    std::thread closing([&] {
        try {
            device.close();
        } catch (...) {
            close_error = std::current_exception();
        }
    });
    opening.join();
    closing.join();

    // close closes admission immediately, including an opening generation.
    // Depending on the linearization order open either succeeds or is interrupted.
    if (open_error) {
        REQUIRE_THROWS_AS(std::rethrow_exception(open_error), ClosedError);
    }
    REQUIRE_FALSE(close_error);
    REQUIRE_FALSE(device.is_open());
    REQUIRE(device.get_health().lifecycle_state == LifecycleState::kClosed);
    REQUIRE_FALSE(device.get_identity().has_value());
    REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(6, 50))), DeviceNotReadyError);
}

TEST_CASE("HandDevice validates timeout and bounded recovery configuration", "[comm][device][recovery]") {
    auto invalid = config_for(Model::kL6);
    invalid.command_timeout = std::chrono::milliseconds{0};
    REQUIRE_THROWS_AS(HandDevice(invalid), ValidationError);

    invalid = config_for(Model::kL6);
    invalid.identity_timeout = std::chrono::milliseconds{0};
    REQUIRE_THROWS_AS(HandDevice(invalid), ValidationError);

    invalid = config_for(Model::kL6);
    invalid.transaction_timeout = std::chrono::milliseconds{0};
    REQUIRE_THROWS_AS(HandDevice(invalid), ValidationError);

    invalid = config_for(Model::kL6);
    invalid.shutdown_timeout = std::chrono::milliseconds{0};
    REQUIRE_THROWS_AS(HandDevice(invalid), ValidationError);

    invalid = config_for(Model::kL6);
    invalid.recovery_attempts = 0;
    REQUIRE_THROWS_AS(HandDevice(invalid), ValidationError);

    invalid = config_for(Model::kL6);
    invalid.recovery_backoff = std::chrono::milliseconds{-1};
    REQUIRE_THROWS_AS(HandDevice(invalid), ValidationError);
}

TEST_CASE("HandDevice polls common telemetry independently for every model", "[comm][device][polling]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            HandDevice device(config_for(model), [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });

            device.open();
            const PollingPlan plan{
                {TelemetrySemantic::kAngles, std::chrono::milliseconds{11}},
                {TelemetrySemantic::kCurrents, std::chrono::milliseconds{13}},
                {TelemetrySemantic::kTemperatures, std::chrono::milliseconds{17}},
                {TelemetrySemantic::kFaults, std::chrono::milliseconds{19}},
            };
            device.start_polling(plan);

            REQUIRE(wait_until([&device] {
                const auto snapshot = device.get_snapshot();
                return snapshot.angles && snapshot.currents && snapshot.temperatures && snapshot.faults;
            }));
            const auto snapshot = device.get_snapshot();
            const auto joint_count = device.get_capabilities().joints.size();
            REQUIRE(test_support::require_value(snapshot.angles).percent.size() == joint_count);
            REQUIRE(test_support::require_value(snapshot.currents).raw.size() == joint_count);
            REQUIRE(test_support::require_value(snapshot.temperatures).raw.size() == joint_count);
            REQUIRE(test_support::require_value(snapshot.faults).raw.size() == joint_count);
            REQUIRE(test_support::require_value(snapshot.currents).meta.origin ==
                    TelemetryValueOrigin::kDeviceMeasured);
            REQUIRE(test_support::require_value(test_support::require_value(snapshot.angles).meta.collected_at) <=
                    snapshot.captured_at);
            REQUIRE(test_support::require_value(test_support::require_value(snapshot.currents).meta.collected_at) <=
                    snapshot.captured_at);
            REQUIRE(test_support::require_value(test_support::require_value(snapshot.temperatures).meta.collected_at) <=
                    snapshot.captured_at);
            REQUIRE(test_support::require_value(test_support::require_value(snapshot.faults).meta.collected_at) <=
                    snapshot.captured_at);

            const auto before_replace =
                test_support::require_value(test_support::require_value(snapshot.angles).meta.collected_at);
            device.start_polling({
                {TelemetrySemantic::kAngles, std::chrono::milliseconds{7}},
            });
            const auto replaced = wait_until([&device, before_replace] {
                const auto current = device.get_snapshot();
                return current.angles &&
                       test_support::require_value(current.angles).meta.collected_at.value() > before_replace;
            });
            const auto replace_health = device.get_health();
            INFO("polling active: " << replace_health.polling_active);
            INFO("recent error: " << (replace_health.recent_error ? replace_health.recent_error->message
                                                                  : std::string{"none"}));
            REQUIRE(replaced);
            device.stop_polling();
        }
    }
}

TEST_CASE("HandDevice rejects invalid polling plans before transport IO", "[comm][device][polling]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            HandDevice device(config_for(model), [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });
            device.open();

            const auto before = transport->sent().size();
            REQUIRE_THROWS_AS(device.start_polling({
                                  {TelemetrySemantic::kAngles, std::chrono::milliseconds{0}},
                              }),
                              PollingPlanError);
            REQUIRE_THROWS_AS(device.start_polling({
                                  {TelemetrySemantic::kAngles, std::chrono::milliseconds{10}},
                                  {TelemetrySemantic::kAngles, std::chrono::milliseconds{20}},
                              }),
                              PollingPlanError);
            REQUIRE(transport->sent().size() == before);

            if (model != Model::kO30) {
                REQUIRE_THROWS_AS(device.start_polling({
                                      {TelemetrySemantic::kVoltages, std::chrono::milliseconds{10}},
                                  }),
                                  UnsupportedOperationError);
                REQUIRE(transport->sent().size() == before);
            }
        }
    }
}

TEST_CASE("HandDevice preserves current and torque telemetry semantics", "[comm][device][polling]") {
    SECTION("L6 current limits and torques are configured device values") {
        FakeTransport *transport = nullptr;
        std::shared_ptr<o30_comm::HopEndpoint> hop;
        HandDevice device(config_for(Model::kL6), [&] {
            auto next = std::make_unique<FakeTransport>();
            transport = next.get();
            install_fake_device(Model::kL6, transport, hop);
            return next;
        });
        device.open();
        transport->clear_sent();
        const auto torque = device.get_torques(std::chrono::milliseconds{500});
        REQUIRE(torque.meta.valid);
        REQUIRE(torque.meta.origin == TelemetryValueOrigin::kDeviceConfigured);
        REQUIRE(torque.meta.delivery == TelemetryDelivery::kQueryResponse);
        REQUIRE(torque.raw.size() == 6);
        REQUIRE(test_support::require_value(device.get_snapshot().torques).raw == torque.raw);
        const auto sent = transport->sent();
        REQUIRE(sent.size() == 1);
        REQUIRE(sent[0].dlc == 1);
        REQUIRE(sent[0].data[0] == 0x03);
        device.start_polling({
            {TelemetrySemantic::kCurrentLimits, std::chrono::milliseconds{10}},
            {TelemetrySemantic::kTorques, std::chrono::milliseconds{13}},
        });
        REQUIRE(wait_until([&device] {
            const auto snapshot = device.get_snapshot();
            return snapshot.current_limits && snapshot.torques;
        }));
        const auto snapshot = device.get_snapshot();
        REQUIRE(test_support::require_value(snapshot.current_limits).meta.origin ==
                TelemetryValueOrigin::kDeviceConfigured);
        REQUIRE(test_support::require_value(snapshot.torques).meta.origin == TelemetryValueOrigin::kDeviceConfigured);
        REQUIRE(test_support::require_value(snapshot.current_limits).raw.size() == 6);
        REQUIRE(test_support::require_value(snapshot.torques).raw.size() == 6);
        device.stop_polling();
    }

    SECTION("O30 preserves measured torque and configured current limits") {
        FakeTransport *transport = nullptr;
        std::shared_ptr<o30_comm::HopEndpoint> hop;
        HandDevice device(config_for(Model::kO30), [&] {
            auto next = std::make_unique<FakeTransport>();
            transport = next.get();
            install_fake_device(Model::kO30, transport, hop);
            return next;
        });
        device.open();
        device.start_polling({
            {TelemetrySemantic::kCurrentLimits, std::chrono::milliseconds{10}},
            {TelemetrySemantic::kTorques, std::chrono::milliseconds{13}},
        });
        REQUIRE(wait_until([&device] {
            const auto snapshot = device.get_snapshot();
            return snapshot.current_limits && snapshot.torques;
        }));
        const auto snapshot = device.get_snapshot();
        REQUIRE(test_support::require_value(snapshot.current_limits).meta.origin ==
                TelemetryValueOrigin::kDeviceConfigured);
        REQUIRE(test_support::require_value(snapshot.torques).meta.origin == TelemetryValueOrigin::kDeviceMeasured);
        REQUIRE(test_support::require_value(snapshot.current_limits).raw.size() == 20);
        REQUIRE(test_support::require_value(snapshot.torques).raw.size() == 20);
        device.stop_polling();
    }
}

TEST_CASE("HandDevice polling isolates a failing telemetry channel", "[comm][device][polling]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kO30), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kO30, transport, hop);
        return next;
    });
    device.open();
    device.start_polling({
        {TelemetrySemantic::kAngles, std::chrono::milliseconds{10}},
        {TelemetrySemantic::kForce, std::chrono::milliseconds{10}},
    });

    REQUIRE(wait_until([&device] { return device.get_snapshot().angles.has_value(); }));
    const auto first =
        test_support::require_value(test_support::require_value(device.get_snapshot().angles).meta.collected_at);
    REQUIRE(wait_until([&device, first] {
        const auto snapshot = device.get_snapshot();
        return snapshot.angles && test_support::require_value(snapshot.angles).meta.collected_at.value() > first;
    }));
    REQUIRE_FALSE(device.get_snapshot().force.has_value());
    REQUIRE(wait_until([&device] { return device.get_health().recent_error.has_value(); }));
    const auto health = device.get_health();
    REQUIRE(health.polling_active);
    REQUIRE(health.recent_error.has_value());
    device.stop_polling();
}

TEST_CASE("HandDevice common event stream publishes cached telemetry", "[comm][device][stream]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kO30), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kO30, transport, hop);
        return next;
    });
    device.open();
    device.start_polling({
        {TelemetrySemantic::kAngles, std::chrono::milliseconds{10}},
        {TelemetrySemantic::kCurrents, std::chrono::milliseconds{10}},
    });
    REQUIRE(wait_until([&device] {
        const auto snapshot = device.get_snapshot();
        return snapshot.angles && snapshot.currents;
    }));
    device.stop_polling();

    const auto sent_before_stream = transport->sent().size();
    const auto stream = device.stream_handle(8);
    const auto event = stream->pop();
    REQUIRE(event.has_value());
    REQUIRE((test_support::require_value(event).semantic == TelemetrySemantic::kAngles ||
             event->semantic == TelemetrySemantic::kCurrents));
    const auto snapshot = device.get_snapshot();
    std::visit(
        [&](const auto &sample) {
            REQUIRE(sample.meta.connection_generation == snapshot.connection_generation);
            REQUIRE(sample.meta.sequence == snapshot.snapshot_sequence);
            REQUIRE(sample.meta.age.has_value());
            REQUIRE(sample.meta.valid);
            REQUIRE_FALSE(sample.meta.stale);
        },
        test_support::require_value(event).sample);
    REQUIRE(transport->sent().size() == sent_before_stream);
    device.stop_stream();
    REQUIRE(stream->closed());
}

TEST_CASE("HandDevice facades can share one physical session", "[comm][device][session]") {
    auto transport = std::make_unique<FakeTransport>();
    auto *raw_transport = transport.get();
    install_classic_identity(transport.get(), Model::kL6, "LHL6-1-123-L", 0x28);
    install_classic_identity(transport.get(), Model::kL6, "LHL6-1-123-R", 0x27);
    auto session = linkerbot::transport::Session::own(std::move(transport));

    auto left_config = config_for(Model::kL6);
    auto right_config = config_for(Model::kL6);
    right_config.side = HandSide::kRight;
    HandDevice left(left_config, session);
    HandDevice right(right_config, session);
    left.open();
    right.open();

    left.close();
    REQUIRE(session->health().open);
    const auto before = raw_transport->sent().size();
    static_cast<void>(completed(right.set_angles(std::vector<int32_t>(6, 50))));
    REQUIRE(raw_transport->sent().size() > before);
    right.close();
    REQUIRE(session->health().open);
    session->close();
}

TEST_CASE("HandDevice recovery retries and restores polling only", "[comm][device][recovery]") {
    auto config = config_for(Model::kL6);
    config.recovery_attempts = 2;
    config.recovery_backoff = std::chrono::milliseconds{0};
    size_t factory_calls = 0;
    FakeTransport *active_transport = nullptr;
    HandDevice device(config, [&]() {
        ++factory_calls;
        if (factory_calls == 2) {
            throw TransportError("simulated first recovery failure");
        }
        auto next = std::make_unique<FakeTransport>();
        active_transport = next.get();
        std::shared_ptr<o30_comm::HopEndpoint> unused;
        install_fake_device(Model::kL6, active_transport, unused);
        return std::unique_ptr<linkerbot::can::CANTransport>(std::move(next));
    });

    device.open();
    static_cast<void>(completed(device.set_angles(std::vector<int32_t>(6, 50))));
    static_cast<void>(completed(device.set_stall_thresholds(std::vector<double>(6, 37.0))));
    device.start_polling({
        {TelemetrySemantic::kAngles, std::chrono::milliseconds{10}},
    });
    REQUIRE(wait_until([&device] { return device.get_snapshot().angles.has_value(); }));
    const auto before = device.get_snapshot();
    const auto before_identity = device.get_identity();

    device.recover();
    REQUIRE(factory_calls == 3);
    REQUIRE(
        wait_until([active_transport] { return active_transport != nullptr && !active_transport->sent().empty(); }));
    const auto frames = active_transport->sent();
    REQUIRE(std::none_of(frames.begin(), frames.end(), [](const Message &frame) {
        const auto replays_angles =
            frame.dlc == 7 && frame.data[0] == 0x01 &&
            std::all_of(frame.data.begin() + 1, frame.data.begin() + 7, [](uint8_t value) { return value == 50; });
        const auto replays_stall =
            frame.dlc == 7 && frame.data[0] == 0xC6 &&
            std::all_of(frame.data.begin() + 1, frame.data.begin() + 7, [](uint8_t value) { return value == 37; });
        return replays_angles || replays_stall;
    }));
    const auto after = device.get_snapshot();
    const auto after_identity = device.get_identity();
    const auto health = device.get_health();
    REQUIRE(after.connection_generation > before.connection_generation);
    REQUIRE(after.angles.has_value());
    REQUIRE(test_support::require_value(after.angles).meta.valid);
    REQUIRE(test_support::require_value(after.angles).meta.connection_generation == after.connection_generation);
    REQUIRE(after.faults.has_value());
    REQUIRE(after.current_limits.has_value());
    REQUIRE(before_identity.has_value());
    REQUIRE(after_identity.has_value());
    REQUIRE(test_support::require_value(after_identity).product_code ==
            test_support::require_value(before_identity).product_code);
    REQUIRE(health.connection_generation == after.connection_generation);
    REQUIRE(health.identity_verified);
    REQUIRE(health.polling_active);
    device.stop_polling();
}

TEST_CASE("HandDevice recovery rejects a replacement physical device", "[comm][device][recovery][identity]") {
    const std::array<std::string, 3> replacements{"LHO6-1-123-L", "LHL6-1-123-R", "LHL6-1-999-L"};
    for (const auto &replacement : replacements) {
        DYNAMIC_SECTION("identity " << replacement) {
            auto config = config_for(Model::kL6);
            config.recovery_attempts = 3;
            config.recovery_backoff = std::chrono::milliseconds{0};
            size_t factory_calls = 0;
            HandDevice device(config, [&]() {
                ++factory_calls;
                auto next = std::make_unique<FakeTransport>();
                std::shared_ptr<o30_comm::HopEndpoint> unused;
                install_fake_telemetry(Model::kL6, next.get(), unused);
                install_classic_identity(next.get(), Model::kL6, factory_calls == 1 ? "LHL6-1-123-L" : replacement);
                return std::unique_ptr<linkerbot::can::CANTransport>(std::move(next));
            });

            device.open();
            const auto previous_generation = device.get_snapshot().connection_generation;
            REQUIRE_THROWS_AS(device.recover(), IdentityMismatchError);
            REQUIRE(factory_calls == 2);
            REQUIRE_FALSE(device.is_open());
            REQUIRE_FALSE(device.get_identity().has_value());
            REQUIRE_FALSE(device.get_verified_capabilities().has_value());
            const auto health = device.get_health();
            REQUIRE(health.lifecycle_state == LifecycleState::kFaulted);
            REQUIRE(health.connection_state == ConnectionState::kFaulted);
            REQUIRE(health.connection_generation > previous_generation);
            REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(6, 50))), DeviceNotReadyError);
            device.close();
        }
    }
}

TEST_CASE("HandDevice recovery faults when baseline initialization fails", "[comm][device][recovery]") {
    auto config = config_for(Model::kL6);
    config.transaction_timeout = std::chrono::milliseconds{20};
    config.recovery_backoff = std::chrono::milliseconds{0};
    size_t factory_calls = 0;
    HandDevice device(config, [&]() {
        ++factory_calls;
        auto next = std::make_unique<FakeTransport>();
        if (factory_calls == 1) {
            std::shared_ptr<o30_comm::HopEndpoint> unused;
            install_fake_device(Model::kL6, next.get(), unused);
        } else {
            install_classic_identity(next.get(), Model::kL6, "LHL6-1-123-L", 0x28, false);
        }
        return std::unique_ptr<linkerbot::can::CANTransport>(std::move(next));
    });

    device.open();
    REQUIRE_THROWS_AS(device.recover(), TimeoutError);
    REQUIRE_FALSE(device.is_open());
    REQUIRE_FALSE(device.get_identity().has_value());
    REQUIRE_FALSE(device.get_verified_capabilities().has_value());
    REQUIRE(device.get_health().lifecycle_state == LifecycleState::kFaulted);
    REQUIRE_THROWS_AS(device.get_snapshot(), DeviceNotReadyError);
    REQUIRE_THROWS_AS(device.start_polling(), DeviceNotReadyError);
    device.close();
}

TEST_CASE("HandDevice rejects work while recovery reinitializes", "[comm][device][recovery]") {
    auto config = config_for(Model::kL6);
    config.recovery_backoff = std::chrono::milliseconds{0};
    std::atomic<FakeTransport *> active_transport{nullptr};
    std::atomic<size_t> factory_calls{0};
    HandDevice device(config, [&]() {
        const auto call = factory_calls.fetch_add(1, std::memory_order_acq_rel) + 1;
        auto next = std::make_unique<FakeTransport>();
        active_transport.store(next.get(), std::memory_order_release);
        std::shared_ptr<o30_comm::HopEndpoint> unused;
        install_fake_telemetry(Model::kL6, next.get(), unused);
        install_classic_identity(next.get(), Model::kL6, "LHL6-1-123-L", 0x28, true,
                                 call == 1 ? std::chrono::milliseconds{0} : std::chrono::milliseconds{100});
        return std::unique_ptr<linkerbot::can::CANTransport>(std::move(next));
    });

    device.open();
    std::exception_ptr recovery_error;
    std::thread recovery([&] {
        try {
            device.recover();
        } catch (...) {
            recovery_error = std::current_exception();
        }
    });
    REQUIRE(wait_until([&] {
        const auto current = active_transport.load(std::memory_order_acquire);
        return factory_calls.load(std::memory_order_acquire) == 2 && current && !current->sent().empty();
    }));

    const auto health = device.get_health();
    REQUIRE(health.lifecycle_state == LifecycleState::kRecovering);
    REQUIRE(health.connection_state == ConnectionState::kReconnecting);
    REQUIRE_FALSE(health.identity_verified);
    REQUIRE_FALSE(device.is_open());
    REQUIRE_FALSE(device.get_identity().has_value());
    REQUIRE_FALSE(device.get_verified_capabilities().has_value());
    REQUIRE_THROWS_AS(completed(device.set_angles(std::vector<int32_t>(6, 50))), DeviceNotReadyError);
    REQUIRE_THROWS_AS(device.start_polling(), DeviceNotReadyError);

    recovery.join();
    REQUIRE_FALSE(recovery_error);
    REQUIRE(device.is_open());
    REQUIRE(device.get_health().identity_verified);
    device.close();
}

TEST_CASE("HandDevice serializes concurrent recovery stop and close", "[comm][device][recovery]") {
    auto config = config_for(Model::kL6);
    config.recovery_backoff = std::chrono::milliseconds{0};
    HandDevice device(config, [] {
        auto next = std::make_unique<FakeTransport>();
        std::shared_ptr<o30_comm::HopEndpoint> unused;
        install_fake_device(Model::kL6, next.get(), unused);
        return next;
    });
    device.open();
    device.start_polling();
    REQUIRE(wait_until([&device] { return device.get_snapshot().angles.has_value(); }));

    std::atomic<size_t> expected_lifecycle_errors{0};
    auto tolerate_close_race = [&device, &expected_lifecycle_errors](const std::function<void()> &operation) {
        try {
            operation();
        } catch (const DeviceNotReadyError &) {
            ++expected_lifecycle_errors;
        } catch (const ClosedError &) // close can now interrupt an entered recovery
        {
            ++expected_lifecycle_errors;
        }
    };
    std::thread recovering([&] { tolerate_close_race([&device] { device.recover(); }); });
    std::thread stopping([&] { tolerate_close_race([&device] { device.stop_polling(); }); });
    std::thread closing([&] { device.close(); });
    recovering.join();
    stopping.join();
    closing.join();

    REQUIRE_FALSE(device.is_open());
    REQUIRE(expected_lifecycle_errors.load() <= 2);
    REQUIRE_NOTHROW(device.close());
}

TEST_CASE("L6 configured speed preserves write replies without issuing a query", "[comm][device][l6-controls]") {
    for (const auto side : {HandSide::kLeft, HandSide::kRight}) {
        DYNAMIC_SECTION("side " << static_cast<int>(side)) {
            auto config = config_for(Model::kL6);
            config.side = side;
            FakeTransport *transport = nullptr;
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_classic_identity(transport, Model::kL6,
                                         side == HandSide::kLeft ? "LHL6-1-123-L" : "LHL6-1-123-R",
                                         side == HandSide::kLeft ? 0x28 : 0x27);
                return next;
            });
            device.open();
            const auto empty_speed = device.get_configured_speeds();
            REQUIRE_FALSE(empty_speed.meta.valid);
            REQUIRE(empty_speed.raw.empty());

            std::array<std::array<uint8_t, 6>, 256> settings{};
            settings[0x05].fill(128);
            transport->on_request(side == HandSide::kLeft ? 0x28 : 0x27, [&settings](const Message &request) {
                Message reply{};
                reply.id = request.id;
                reply.dlc = 7;
                reply.data[0] = request.data[0];
                auto &raw = settings[request.data[0]];
                std::copy(raw.begin(), raw.end(), reply.data.begin() + 1);
                if (request.dlc == 7) {
                    std::copy_n(request.data.begin() + 1, 6, raw.begin());
                }
                return std::vector<Message>{reply};
            });

            const std::vector<double> speed_target{0, 1, 127, 128, 254, 255};
            REQUIRE(completed(device.set_speeds(speed_target)).stage() == CommandStage::kAcknowledged);
            const auto before_speed_read = transport->sent().size();
            const auto speed = device.get_configured_speeds();
            REQUIRE(speed.raw == std::vector<double>(6, 128));
            REQUIRE(speed.meta.delivery == TelemetryDelivery::kCommandResponse);
            REQUIRE(speed.meta.origin == TelemetryValueOrigin::kCommandCache);
            REQUIRE(test_support::require_value(device.get_snapshot().configured_speeds).raw == speed.raw);
            REQUIRE(device.get_snapshot().joints[0].configured_speed_raw == 128);
            REQUIRE(transport->sent().size() == before_speed_read);
            REQUIRE_THROWS_AS(device.get_speeds(), UnsupportedOperationError);
            const auto sent_speed = transport->sent().back();
            for (size_t i = 0; i < speed_target.size(); ++i) {
                REQUIRE(sent_speed.data[i + 1] == speed_target[i]);
            }
            device.close();
            device.open();
            REQUIRE_FALSE(device.get_configured_speeds().meta.valid);
        }
    }
}

TEST_CASE("known absent force fields are rejected without device traffic", "[comm][force-fields]") {
    FakeTransport *transport = nullptr;
    HandDevice device(config_for(Model::kL6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    const auto sent = transport->sent().size();
    REQUIRE_THROWS_AS(device.get_force({"fns"}), UnsupportedOperationError);
    REQUIRE_THROWS_AS(device.get_force({"fn", "px"}), UnsupportedOperationError);
    REQUIRE(transport->sent().size() == sent);
}

TEST_CASE("L20Lite field getters deduplicate native scalar transactions", "[comm][force-fields]") {
    FakeTransport *transport = nullptr;
    HandDevice device(config_for(Model::kL20Lite), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL20Lite);
        return next;
    });
    device.open();
    transport->on_request(0x28, [](const Message &request) {
        Message response{};
        response.id = request.id;
        response.data[0] = request.data[0];
        response.dlc = 6;
        const uint8_t raw = request.data[0] == 0x20 ? 0 : request.data[0] == 0x22 ? 255 : request.data[0];
        std::fill_n(response.data.begin() + 1, 5, raw);
        return std::vector<Message>{response};
    });
    const auto before = transport->sent().size();
    const auto force = device.get_force({"fns", "fts", "fds", "px", "fts"});
    REQUIRE(transport->sent().size() - before == 4);
    REQUIRE(force.requested_fields == std::vector<std::string>{"fns", "fts", "fds", "px"});
    REQUIRE(force.fields.size() == 4);
    REQUIRE(force.field("fns")->raw[0] == std::vector<double>{0});
    REQUIRE(force.field("fds")->valid_values[0] == std::vector<bool>{false});
    REQUIRE(force.field("px")->meta.connection_generation == device.get_snapshot().connection_generation);
    REQUIRE(force.field("px")->meta.sequence > 0);
    REQUIRE(test_support::require_value(device.get_snapshot().force).field("fts") != nullptr);
    REQUIRE(device.get_normal_force_resultant().raw == std::vector<double>(5, 0));
    REQUIRE_THROWS_AS(device.get_force({"fd"}), UnsupportedOperationError);
    transport->on_request(0x28, [](const Message &request) {
        Message response{};
        response.id = request.id;
        response.dlc = 6;
        response.data[0] = request.data[0];
        std::fill_n(response.data.begin() + 1, 5, 42);
        return std::vector<Message>{response};
    });
    device.start_polling({{TelemetrySemantic::kForceFns, std::chrono::milliseconds{20}},
                          {TelemetrySemantic::kForcePx, std::chrono::milliseconds{30}}});
    REQUIRE(wait_until([&device] {
        const auto snapshot = device.get_snapshot();
        const auto *field = snapshot.force ? snapshot.force->field("px") : nullptr;
        return field && field->raw[0] == std::vector<double>{42};
    }));
    device.stop_polling();
    const auto previous = test_support::require_value(device.get_snapshot().force).field("fns")->meta.collected_at;
    transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
    REQUIRE_THROWS_AS(device.get_force({"fns"}, std::chrono::milliseconds{30}), TimeoutError);
    const auto after = device.get_snapshot();
    REQUIRE(after.force.has_value());
    REQUIRE(test_support::require_value(after.force).field("fns") != nullptr);
    REQUIRE(test_support::require_value(after.force).field("fns")->meta.collected_at == previous);
}

TEST_CASE("O30 unified force reads five descriptors once for shared selectors", "[comm][force-fields]") {
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config_for(Model::kO30), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kO30, transport, hop);
        hop->put_ascii(0x41, 0x61, 8, "0.0.4");
        std::vector<uint8_t> metadata(0x73, 0);
        const std::string type = "TEST", descriptor = "FnS_U16_1;Fn_U8_2";
        std::copy(type.begin(), type.end(), metadata.begin());
        metadata[0x20] = 'g';
        metadata[0x29] = 1;
        metadata[0x2A] = 2;
        metadata[0x2B] = static_cast<uint8_t>(descriptor.size());
        std::copy(descriptor.begin(), descriptor.end(), metadata.begin() + 0x2C);
        metadata[0x6C] = 4;
        metadata[0x6E] = 1;
        metadata[0x6F] = 1;
        metadata[0x70] = 2;
        hop->put(0x31, metadata);
        hop->put(0x32, {0, 1, 0, 9});
        return next;
    });
    device.open();
    hop->writes.clear();
    const auto force = device.get_force();
    REQUIRE(force.requested_fields == std::vector<std::string>{"fn", "fns"});
    REQUIRE(force.sensors.size() == 5);
    REQUIRE(force.field("fns")->raw.size() == 5);
    REQUIRE(force.field("fns")->raw[0] == std::vector<double>{256});
    REQUIRE(force.field("fn")->raw[4] == std::vector<double>{0, 9});
    REQUIRE(hop->writes.size() == 5);
    for (size_t i = 0; i < 5; ++i) {
        REQUIRE(hop->writes[i].si == 0x6E);
        REQUIRE(hop->writes[i].payload == std::vector<uint8_t>{static_cast<uint8_t>(i + 1)});
    }
    REQUIRE(test_support::require_value(device.get_snapshot().force).sensors.size() == 5);
    REQUIRE(device.get_capabilities().force_field("fn").supported == true);
    REQUIRE(device.get_capabilities().force_field("ft").supported == false);
    const auto resultant = device.get_normal_force_resultant();
    REQUIRE(resultant.raw == std::vector<double>(5, 256));
    REQUIRE(resultant.units[0] == "g");
    const auto sent_before_unsupported = transport->sent().size();
    REQUIRE_THROWS_AS(device.get_force({"ft"}), UnsupportedOperationError);
    REQUIRE(transport->sent().size() == sent_before_unsupported);
    const auto previous = test_support::require_value(device.get_snapshot().force).meta.collected_at;
    transport->on_request(hop->request_id, [hop](const Message &request) {
        const auto &metadata = hop->objects.at({0x31, false});
        if (request.data[0] == 0x32 && metadata[0x6E] == 3) {
            return std::vector<Message>{};
        }
        return hop->handler()(request);
    });
    REQUIRE_THROWS_AS(device.get_force({"fn"}, std::chrono::milliseconds{50}), TimeoutError);
    REQUIRE(test_support::require_value(device.get_snapshot().force).meta.collected_at == previous);
    REQUIRE(test_support::require_value(device.get_snapshot().force).sensors.size() == 5);
}

TEST_CASE("unified configuration ACKs require complete matching write responses", "[comm][device][parity][ack]") {
    using Setter = CommandHandle (HandDevice::*)(const std::vector<double> &);
    struct Case {
        Model model;
        CommandSemantic semantic;
        Setter setter;
    };
    const std::vector<Case> cases{
        {Model::kL6, CommandSemantic::kCurrentLimits, &HandDevice::set_current_limits},
        {Model::kL6, CommandSemantic::kStallDetectionTimes, &HandDevice::set_stall_detection_times},
        {Model::kL6, CommandSemantic::kStallThresholds, &HandDevice::set_stall_thresholds},
        {Model::kL6, CommandSemantic::kStallHoldCurrents, &HandDevice::set_stall_hold_currents},
        {Model::kO6, CommandSemantic::kSpeeds, &HandDevice::set_speeds},
        {Model::kO6, CommandSemantic::kAccelerations, &HandDevice::set_accelerations},
        {Model::kO6, CommandSemantic::kCurrentLimits, &HandDevice::set_current_limits},
        {Model::kO6, CommandSemantic::kStallDetectionTimes, &HandDevice::set_stall_detection_times},
        {Model::kO6, CommandSemantic::kStallThresholds, &HandDevice::set_stall_thresholds},
        {Model::kO6, CommandSemantic::kStallHoldCurrents, &HandDevice::set_stall_hold_currents},
        {Model::kL20Lite, CommandSemantic::kSpeeds, &HandDevice::set_speeds},
        {Model::kL20Lite, CommandSemantic::kAccelerations, &HandDevice::set_accelerations},
        {Model::kL20Lite, CommandSemantic::kCurrentLimits, &HandDevice::set_current_limits},
    };
    for (const auto &item : cases) {
        DYNAMIC_SECTION("model " << static_cast<int>(item.model) << " command " << static_cast<int>(item.semantic)) {
            auto config = config_for(item.model);
            // Include queueing and every frame of the write in the ACK budget.
            // These tests check matching, not sub-tick Windows scheduling.
            config.command_timeout = std::chrono::milliseconds{500};
            FakeTransport *transport = nullptr;
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_classic_identity(transport, item.model);
                return next;
            });
            device.open();
            const auto count = device.get_capabilities().joints.size();
            const auto frames = item.model == Model::kL20Lite ? 2U : 1U;
            const auto cached = [&](const HandSnapshot &snapshot) {
                const auto value = [](const auto &sample) {
                    REQUIRE(sample.has_value());
                    return std::make_pair(sample->raw, sample->meta.collected_at);
                };
                switch (item.semantic) {
                case CommandSemantic::kSpeeds:
                    return value(snapshot.configured_speeds);
                case CommandSemantic::kAccelerations:
                    return value(snapshot.configured_accelerations);
                case CommandSemantic::kCurrentLimits:
                    return value(snapshot.current_limits);
                case CommandSemantic::kStallDetectionTimes:
                    return value(snapshot.stall_detection_times);
                case CommandSemantic::kStallThresholds:
                    return value(snapshot.stall_thresholds);
                case CommandSemantic::kStallHoldCurrents:
                    return value(snapshot.stall_hold_currents);
                default:
                    throw std::logic_error("unsupported test command");
                }
            };
            transport->on_request(0x28, [](const Message &request) {
                auto reply = request;
                for (size_t index = 1; index < reply.dlc; ++index) {
                    reply.data[index] = 42; // A valid old value still acknowledges the write.
                }
                return std::vector<Message>{reply};
            });
            const auto before = transport->sent().size();
            const auto handle = completed((device.*item.setter)(std::vector<double>(count, 10)));
            REQUIRE(handle.stage() == CommandStage::kAcknowledged);
            REQUIRE(handle.ack_expectation() == AckExpectation::kRequired);
            REQUIRE(handle.sent_frames() == frames);
            REQUIRE(handle.done());
            const auto sent = transport->sent();
            REQUIRE(sent.size() == before + frames);
            for (size_t index = before; index < sent.size(); ++index) {
                REQUIRE(sent[index].dlc > 1); // No post-write query used to fake an ACK.
                for (size_t byte = 1; byte < sent[index].dlc; ++byte) {
                    REQUIRE(sent[index].data[byte] == 10);
                }
            }
            const auto previous = cached(device.get_snapshot());
            REQUIRE(previous.first == std::vector<double>(count, 42));

            SECTION("missing response times out") {
                transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
            }
            SECTION("unrelated response cannot acknowledge a write") {
                transport->on_request(0x28, [](const Message &request) {
                    auto reply = request;
                    reply.data[0] = 0x99;
                    return std::vector<Message>{reply};
                });
            }
            SECTION("truncated response cannot acknowledge a write") {
                transport->on_request(0x28, [](const Message &request) {
                    auto reply = request;
                    --reply.dlc;
                    return std::vector<Message>{reply};
                });
            }
            const auto failed_before = transport->sent().size();
            REQUIRE_THROWS_AS(completed((device.*item.setter)(std::vector<double>(count, 20))), TimeoutError);
            REQUIRE(transport->sent().size() == failed_before + frames);
            const auto retained = cached(device.get_snapshot());
            REQUIRE(retained.first == previous.first);
            REQUIRE(retained.second == previous.second);
        }
    }
}

TEST_CASE("L20Lite configuration ACKs require both response frames", "[comm][device][parity][ack]") {
    for (const auto setter :
         {&HandDevice::set_speeds, &HandDevice::set_accelerations, &HandDevice::set_current_limits}) {
        auto config = config_for(Model::kL20Lite);
        config.command_timeout = std::chrono::milliseconds{500};
        FakeTransport *transport = nullptr;
        HandDevice device(config, [&] {
            auto next = std::make_unique<FakeTransport>();
            transport = next.get();
            install_classic_identity(transport, Model::kL20Lite);
            return next;
        });
        device.open();
        const auto before = transport->sent().size();
        transport->on_request(0x28, [](const Message &request) {
            if (request.data[0] == 0x05 || request.data[0] == 0x87 || request.data[0] == 0x02) {
                return std::vector<Message>{request, request}; // Duplicate A does not replace B.
            }
            return std::vector<Message>{};
        });
        REQUIRE_THROWS_AS(completed((device.*setter)(std::vector<double>(10, 17))), TimeoutError);
        REQUIRE(transport->sent().size() == before + 2);
    }
}

TEST_CASE("L6 rejects ineffective raw zero stall writes before transport access", "[comm][device][parity][stall]") {
    FakeTransport *transport = nullptr;
    HandDevice device(config_for(Model::kL6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    const auto before = transport->sent().size();
    for (const auto invalid : {0.0, -1.0, 256.0, 0.5}) {
        auto values = std::vector<double>(6, 10);
        values[3] = invalid;
        REQUIRE_THROWS_AS(completed(device.set_stall_detection_times(values)), ValidationError);
        REQUIRE(transport->sent().size() == before);
    }
    for (const auto valid : {1.0, 255.0}) {
        REQUIRE(completed(device.set_stall_detection_times(std::vector<double>(6, valid))).stage() ==
                CommandStage::kAcknowledged);
    }
    transport->on_request(0x28, [](const Message &request) {
        Message reply{};
        reply.id = request.id;
        reply.data[0] = request.data[0];
        reply.dlc = 7;
        return std::vector<Message>{reply};
    });
    const auto sample = device.get_stall_detection_times(std::chrono::milliseconds{100});
    REQUIRE(sample.raw == std::vector<double>(6, 0));
    REQUIRE(sample.meta.valid); // Read evidence is preserved even when writes reject zero.
}

TEST_CASE("HandDevice cache and validation stay available during a blocked send",
          "[comm][device][facade-concurrency]") {
    for (const auto model : {Model::kL6, Model::kO6, Model::kL20Lite, Model::kL20, Model::kL30, Model::kO30}) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            auto config = config_for(model);
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });
            device.open();
            const auto before = device.get_snapshot();
            const auto count = device.get_capabilities().joints.size();
            transport->arm_tx_gate();
            auto command =
                std::async(std::launch::async, [&] { return device.set_angles(std::vector<int32_t>(count, 50)); });
            const auto entered = transport->wait_until_tx_entered(std::chrono::seconds{2});
            auto cached = std::async(std::launch::async, [&] {
                const auto snapshot = device.get_snapshot();
                const auto health = device.get_health();
                const auto caps = device.get_capabilities();
                bool invalid_rejected = false;
                try {
                    (void)device.set_percent_angles({-1});
                } catch (const ValidationError &) {
                    invalid_rejected = true;
                }
                return snapshot.connection_generation == before.connection_generation && health.identity_verified &&
                       caps.joints.size() == count && invalid_rejected;
            });
            const auto admitted = command.wait_for(std::chrono::milliseconds{300}) == std::future_status::ready;
            const auto available = cached.wait_for(std::chrono::milliseconds{300}) == std::future_status::ready;
            // Always release before assertions/future destruction, including on
            // a regression: the fake driver cannot interrupt a claimed send.
            transport->release_tx_gate();
            const auto cache_ok = cached.get();
            (void)completed(command.get());
            REQUIRE(admitted);
            REQUIRE(entered);
            REQUIRE(available);
            REQUIRE(cache_ok);
        }
    }
}

TEST_CASE("HandDevice caches and command admission stay available while an ACK is pending",
          "[comm][device][facade-concurrency]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{2};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    const auto before = device.get_snapshot();
    std::atomic<bool> waiting{false};
    transport->on_request(0x28, [&](const Message &request) {
        if (request.data[0] == 0x05) {
            waiting.store(true);
            return std::vector<Message>{};
        }
        return classic_angle_reply(Model::kL6, request);
    });
    auto speed = std::async(std::launch::async, [&] { return device.set_speeds(std::vector<double>(6, 100)); });
    const auto entered = wait_until([&] { return waiting.load(); });
    auto cached = std::async(std::launch::async, [&] {
        const auto sent = transport->sent().size();
        const auto snapshot = device.get_snapshot();
        const auto health = device.get_health();
        (void)device.get_capabilities();
        return transport->sent().size() == sent && health.identity_verified &&
               snapshot.connection_generation == before.connection_generation;
    });
    const auto available = cached.wait_for(std::chrono::milliseconds{300}) == std::future_status::ready;
    const auto cache_ok = available ? cached.get() : false;
    auto stall =
        std::async(std::launch::async, [&] { return device.set_stall_thresholds(std::vector<double>(6, 40)); });
    const auto independent = stall.wait_for(std::chrono::milliseconds{300}) == std::future_status::ready;
    Message reply{};
    reply.id = 0x28;
    reply.dlc = 7;
    reply.data[0] = 0x05;
    std::fill_n(reply.data.begin() + 1, 6, 100);
    transport->inject(reply);
    (void)completed(speed.get());
    (void)completed(stall.get());
    if (!available) {
        (void)cached.get();
    }
    REQUIRE(entered);
    REQUIRE(available);
    REQUIRE(cache_ok);
    REQUIRE(independent);
}

TEST_CASE("HandDevice close cancels an ACK waiter without retaining the state lock",
          "[comm][device][facade-concurrency]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{2};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    std::atomic<bool> waiting{false};
    transport->on_request(0x28, [&](const Message &) {
        waiting.store(true);
        return std::vector<Message>{};
    });
    auto command = std::async(std::launch::async, [&] {
        try {
            (void)completed(device.set_speeds(std::vector<double>(6, 100)));
        } catch (const ClosedError &) {
            return true;
        } catch (const DeviceNotReadyError &) {
            return true;
        }
        return false;
    });
    const auto entered = wait_until([&] { return waiting.load(); });
    auto closing = std::async(std::launch::async, [&] { device.close(); });
    const auto promptly_closed = closing.wait_for(std::chrono::milliseconds{500}) == std::future_status::ready;
    closing.get();
    const auto rejected = command.get();
    REQUIRE(entered);
    REQUIRE(promptly_closed);
    REQUIRE(rejected);
    REQUIRE(device.get_health().lifecycle_state == LifecycleState::kClosed);
    REQUIRE_THROWS_AS(device.get_snapshot(), DeviceNotReadyError);
    REQUIRE_NOTHROW(device.open());
    REQUIRE(device.get_health().identity_verified);
}

TEST_CASE("HandDevice recovery cancels old execution before resetting its caches",
          "[comm][device][facade-concurrency]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{2};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    const auto before = device.get_snapshot();
    auto *old_transport = transport;
    std::atomic<bool> waiting{false};
    old_transport->on_request(0x28, [&](const Message &) {
        waiting.store(true);
        return std::vector<Message>{};
    });
    auto command = std::async(std::launch::async, [&] {
        try {
            (void)completed(device.set_speeds(std::vector<double>(6, 100)));
        } catch (const ClosedError &) {
            return true;
        } catch (const DeviceNotReadyError &) {
            return true;
        }
        return false;
    });
    const auto entered = wait_until([&] { return waiting.load(); });
    auto recovery = std::async(std::launch::async, [&] { device.recover(); });
    const auto rejected = command.get();
    recovery.get();
    const auto after = device.get_snapshot();
    REQUIRE(entered);
    REQUIRE(rejected);
    REQUIRE(after.connection_generation > before.connection_generation);
    REQUIRE_FALSE(after.configured_speeds.has_value());
    REQUIRE(test_support::require_value(after.angles).meta.connection_generation == after.connection_generation);
}

TEST_CASE("HandDevice state views remain available while close drains a claimed send",
          "[comm][device][facade-concurrency]") {
    auto config = config_for(Model::kL6);
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    transport->arm_tx_gate();
    auto command = std::async(std::launch::async, [&] {
        try {
            (void)completed(device.set_angles(std::vector<int32_t>(6, 50)));
        } catch (const HandError &) {
            return true;
        }
        return false;
    });
    const auto entered = transport->wait_until_tx_entered(std::chrono::seconds{2});
    auto closing = std::async(std::launch::async, [&] { device.close(); });
    const auto closing_visible =
        wait_until([&] { return device.get_health().lifecycle_state == LifecycleState::kClosing; });
    auto cached = std::async(std::launch::async, [&] {
        const auto health = device.get_health();
        (void)device.get_capabilities();
        bool rejected = false;
        try {
            (void)device.get_snapshot();
        } catch (const DeviceNotReadyError &) {
            rejected = true;
        }
        return health.lifecycle_state == LifecycleState::kClosing && rejected;
    });
    const auto available = cached.wait_for(std::chrono::milliseconds{300}) == std::future_status::ready;
    const auto cache_ok = available ? cached.get() : false;
    transport->release_tx_gate();
    const auto rejected = command.get();
    closing.get();
    if (!available) {
        (void)cached.get();
    }
    REQUIRE(entered);
    REQUIRE(closing_visible);
    REQUIRE(available);
    REQUIRE(cache_ok);
    REQUIRE(rejected);
    REQUIRE_FALSE(device.is_open());
}

TEST_CASE("HandDevice active reads do not block cached state and close cancels them",
          "[comm][device][facade-concurrency]") {
    for (const bool force : {false, true}) {
        DYNAMIC_SECTION("force read " << force) {
            FakeTransport *transport = nullptr;
            HandDevice device(config_for(Model::kL6), [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_classic_identity(transport, Model::kL6);
                return next;
            });
            device.open();
            const auto before = device.get_snapshot();
            std::atomic<bool> waiting{false};
            transport->on_request(0x28, [&](const Message &) {
                waiting.store(true);
                return std::vector<Message>{};
            });
            auto read = std::async(std::launch::async, [&] {
                try {
                    if (force) {
                        (void)device.get_force({"fn"}, std::chrono::seconds{2});
                    } else {
                        (void)device.get_angles(std::chrono::seconds{2});
                    }
                } catch (const DeviceNotReadyError &) {
                    return true;
                }
                return false;
            });
            const auto entered = wait_until([&] { return waiting.load(); });
            auto cached = std::async(std::launch::async, [&] {
                const auto snapshot = device.get_snapshot();
                const auto health = device.get_health();
                (void)device.get_capabilities();
                return health.identity_verified && test_support::require_value(snapshot.angles).meta.collected_at ==
                                                       test_support::require_value(before.angles).meta.collected_at;
            });
            const auto available = cached.wait_for(std::chrono::milliseconds{300}) == std::future_status::ready;
            const auto cache_ok = available ? cached.get() : false;
            device.close();
            const auto rejected = read.get();
            if (!available) {
                (void)cached.get();
            }
            REQUIRE(entered);
            REQUIRE(available);
            REQUIRE(cache_ok);
            REQUIRE(rejected);
        }
    }
}

TEST_CASE("HandDevice stopping a blocked poller does not hold the state lock", "[comm][device][facade-concurrency]") {
    FakeTransport *transport = nullptr;
    HandDevice device(config_for(Model::kL6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    const auto generation = device.get_snapshot().connection_generation;
    transport->arm_tx_gate();
    device.start_polling();
    const auto entered = transport->wait_until_tx_entered(std::chrono::seconds{2});
    auto stopping = std::async(std::launch::async, [&] { device.stop_polling(); });
    auto cached = std::async(std::launch::async, [&] {
        const auto snapshot = device.get_snapshot();
        const auto health = device.get_health();
        // Cancelling the pending poll may invalidate its angle sample. These
        // cache-only calls must still return without waiting for backend I/O.
        return snapshot.connection_generation == generation && health.identity_verified;
    });
    const auto available = cached.wait_for(std::chrono::milliseconds{300}) == std::future_status::ready;
    transport->release_tx_gate();
    const auto cache_ok = cached.get();
    stopping.get();
    REQUIRE(entered);
    REQUIRE(available);
    REQUIRE(cache_ok);
    REQUIRE_FALSE(device.get_health().polling_active);
}

TEST_CASE("HandDevice re-verification exposes state without waiting for identity I/O",
          "[comm][device][facade-concurrency]") {
    FakeTransport *transport = nullptr;
    HandDevice device(config_for(Model::kL6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    transport->arm_tx_gate();
    auto verify = std::async(std::launch::async, [&] { return device.verify_identity(std::chrono::seconds{2}); });
    const auto entered = transport->wait_until_tx_entered(std::chrono::seconds{2});
    auto cached = std::async(std::launch::async, [&] {
        const auto health = device.get_health();
        const auto caps = device.get_capabilities();
        bool rejected = false;
        try {
            (void)device.set_angles(std::vector<int32_t>(6, 50));
        } catch (const DeviceNotReadyError &) {
            rejected = true;
        }
        return health.lifecycle_state == LifecycleState::kOpening && !device.get_identity().has_value() &&
               caps.model == Model::kL6 && rejected;
    });
    const auto available = cached.wait_for(std::chrono::milliseconds{300}) == std::future_status::ready;
    const auto cache_ok = available ? cached.get() : false;
    transport->release_tx_gate();
    (void)verify.get();
    if (!available) {
        (void)cached.get();
    }
    REQUIRE(entered);
    REQUIRE(available);
    REQUIRE(cache_ok);
    REQUIRE(device.get_health().identity_verified);
}

TEST_CASE("M2 all eight setters return before their transport send completes", "[comm][device][m2]") {
    using Setter = CommandHandle (HandDevice::*)(const std::vector<double> &);
    struct Item {
        Model model;
        CommandSemantic semantic;
        Setter setter;
    };
    const std::vector<Item> items{
        {Model::kL6, CommandSemantic::kSpeeds, &HandDevice::set_speeds},
        {Model::kO6, CommandSemantic::kAccelerations, &HandDevice::set_accelerations},
        {Model::kL6, CommandSemantic::kCurrentLimits, &HandDevice::set_current_limits},
        {Model::kL6, CommandSemantic::kStallDetectionTimes, &HandDevice::set_stall_detection_times},
        {Model::kL6, CommandSemantic::kStallThresholds, &HandDevice::set_stall_thresholds},
        {Model::kL6, CommandSemantic::kStallHoldCurrents, &HandDevice::set_stall_hold_currents},
        {Model::kL6, CommandSemantic::kAngles, nullptr},
        {Model::kL6, CommandSemantic::kAngles, nullptr},
    };
    for (size_t i = 0; i < items.size(); ++i) {
        CAPTURE(i);
        const auto &item = items[i];
        auto config = config_for(item.model);
        config.command_timeout = std::chrono::seconds{3};
        FakeTransport *transport = nullptr;
        std::shared_ptr<o30_comm::HopEndpoint> hop;
        HandDevice device(config, [&] {
            auto next = std::make_unique<FakeTransport>();
            transport = next.get();
            install_fake_device(item.model, transport, hop);
            return next;
        });
        device.open();
        const auto caps = device.get_capabilities();
        const auto *capability = caps.command(item.semantic);
        REQUIRE(capability);
        std::vector<double> values;
        for (size_t n = 0; n < capability->channels.size(); ++n) {
            values.push_back(capability->range_for(n)->minimum);
        }
        transport->arm_tx_gate();
        auto submission = std::async(std::launch::async, [&] {
            if (item.setter) {
                return (device.*item.setter)(values);
            }
            if (i == items.size() - 1) {
                return device.set_percent_angles(std::vector<float>(6, 50));
            }
            return device.set_angles(std::vector<int32_t>(6, 50));
        });
        const auto entered = transport->wait_until_tx_entered(std::chrono::seconds{2});
        const auto returned = submission.wait_for(std::chrono::milliseconds{300}) == std::future_status::ready;
        std::optional<CommandHandle> pending;
        if (returned) {
            pending = submission.get();
        }
        const auto still_pending = pending && !pending->done();
        transport->release_tx_gate();
        if (!returned) {
            pending = submission.get();
        }
        (void)completed(*pending);
        REQUIRE(entered);
        REQUIRE(returned);
        REQUIRE(still_pending);
    }
}

TEST_CASE("M2 percent and raw angles share a copied latest slot", "[comm][device][m2]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{3};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    transport->arm_tx_gate();
    auto first = device.set_angles(std::vector<int32_t>(6, 10));
    const auto entered = transport->wait_until_tx_entered(std::chrono::seconds{2});
    if (!entered) {
        transport->release_tx_gate();
    }
    // Do not assert with an unreleased send gate.
    auto old = device.set_angles(std::vector<int32_t>(6, 20));
    const auto admitted_before = device.get_health().command_families;
    bool invalid_raw = false, invalid_percent = false;
    try {
        (void)device.set_angles({1});
    } catch (const ValidationError &) {
        invalid_raw = true;
    }
    try {
        (void)device.set_percent_angles(std::vector<float>(6, std::numeric_limits<float>::quiet_NaN()));
    } catch (const ValidationError &) {
        invalid_percent = true;
    }
    const auto admitted_after = device.get_health().command_families;
    const auto old_preserved = old.stage() == CommandStage::kAccepted;
    auto fifo = device.set_stall_thresholds(std::vector<double>(6, 30));
    auto percent = std::vector<float>(6, 50);
    auto newest = device.set_percent_angles(percent);
    std::fill(percent.begin(), percent.end(), 0);
    const auto before = device.get_health();
    const auto superseded = old.stage() == CommandStage::kSuperseded && old.sent_frames() == 0;
    transport->release_tx_gate();
    (void)completed(first);
    (void)completed(fifo);
    (void)completed(newest);
    REQUIRE(entered);
    REQUIRE(superseded);
    REQUIRE(invalid_raw);
    REQUIRE(invalid_percent);
    REQUIRE(old_preserved);
    REQUIRE(admitted_before.front().accepted == admitted_after.front().accepted);
    REQUIRE(before.pending_commands() == 3);
    const auto sent = transport->sent();
    REQUIRE(sent[sent.size() - 2].data[0] == 0xC6); // New latest gets a new sequence.
    const auto expected = percent_to_raw_angles(std::vector<double>(6, 50), device.get_capabilities());
    REQUIRE(sent.back().data[0] == 0x01);
    for (size_t i = 0; i < 6; ++i) {
        REQUIRE(sent.back().data[i + 1] == expected[i]);
    }
    const auto health = device.get_health();
    REQUIRE(health.pending_commands() == 0);
    REQUIRE(health.superseded_commands() == 1);
}

TEST_CASE("M2 queue and rate configuration is validated without transport access", "[comm][device][m2]") {
    for (const auto rate : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::denorm_min()}) {
        auto config = config_for(Model::kL6);
        config.command_rate_hz = rate;
        REQUIRE_THROWS_AS(HandDevice(config), ValidationError);
    }
    auto config = config_for(Model::kL6);
    config.discrete_queue_capacity = 0;
    REQUIRE_THROWS_AS(HandDevice(config), ValidationError);
    REQUIRE(HandRuntimeConfig{}.command_rate_hz == 250.0);
    REQUIRE(HandDeviceConfig{}.discrete_queue_capacity == 16);
}

TEST_CASE("M2 polling stops while queued behind a setter ACK", "[comm][device][m2]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{3};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    std::atomic<bool> waiting{false};
    transport->on_request(0x28, [&](const Message &request) {
        if (request.data[0] == 0x05) {
            waiting.store(true);
            return std::vector<Message>{};
        }
        return classic_angle_reply(Model::kL6, request);
    });
    auto setter = device.set_speeds(std::vector<double>(6, 100));
    const auto entered = wait_until([&] { return waiting.load(); });
    device.start_polling({{TelemetrySemantic::kAngles, std::chrono::milliseconds{1}}});
    auto stopping = std::async(std::launch::async, [&] { device.stop_polling(); });
    const auto stopped = stopping.wait_for(std::chrono::milliseconds{500}) == std::future_status::ready;
    // Release before any assertion: even a regressed stop path must be able to finish.
    Message reply{};
    reply.id = 0x28;
    reply.dlc = 7;
    reply.data[0] = 0x05;
    std::fill_n(reply.data.begin() + 1, 6, 100);
    transport->inject(reply);
    stopping.get();
    (void)completed(setter);
    REQUIRE(entered);
    REQUIRE(stopped);
    REQUIRE_FALSE(device.get_health().polling_active);
}

TEST_CASE("Command receipts expose dispatch while ACK waiters remain pending", "[comm][device][command-receipts]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{2};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
    auto handle = device.set_speeds(std::vector<double>(6, 100));
    REQUIRE(wait_until([&] { return handle.stage() == CommandStage::kDispatched; }));
    REQUIRE_FALSE(handle.done());
    REQUIRE(handle.sent_frames() == 1);
    REQUIRE(handle.history().size() == 2);
    REQUIRE_FALSE(handle.wait_for(std::chrono::milliseconds{10}));
    REQUIRE_FALSE(handle.error());
    Message reply{};
    reply.id = 0x28;
    reply.dlc = 7;
    reply.data[0] = 0xC6; // unrelated stall ACK
    std::fill_n(reply.data.begin() + 1, 6, 80);
    transport->inject(reply);
    REQUIRE_FALSE(handle.wait_for(std::chrono::milliseconds{20}));
    auto first = std::async(std::launch::async, [handle] {
        handle.wait();
        return handle.stage();
    });
    auto second = std::async(std::launch::async, [handle] { return handle.wait_for(std::chrono::seconds{2}); });
    reply.data[0] = 0x05;
    transport->inject(reply);
    REQUIRE(first.get() == CommandStage::kAcknowledged);
    REQUIRE(second.get());
    REQUIRE_FALSE(handle.error());
    const auto history = handle.history();
    REQUIRE(history.size() == 3);
    REQUIRE(history[0].timestamp <= history[1].timestamp);
    REQUIRE(history[1].timestamp <= history[2].timestamp);
    REQUIRE(device.get_configured_speeds().raw == std::vector<double>(6, 80));
    const auto health = device.get_health();
    const auto metric = std::find_if(health.command_families.begin(), health.command_families.end(),
                                     [](const auto &m) { return m.family == "speed_targets"; });
    REQUIRE(metric != health.command_families.end());
    REQUIRE(metric->dispatched == 1);
    REQUIRE(wait_until([&] { return device.get_health().pending_commands() == 0; }));
}

TEST_CASE("ACK timeout retains sent count and last successful cache", "[comm][device][command-receipts]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::milliseconds{80};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    (void)completed(device.set_speeds(std::vector<double>(6, 50)));
    // This snapshot is compared after the failure by Catch2 assertions below.
    // NOLINTNEXTLINE(clang-analyzer-deadcode.DeadStores)
    const auto previous = device.get_configured_speeds();
    transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
    auto failed = device.set_speeds(std::vector<double>(6, 100));
    REQUIRE(failed.wait_for(std::chrono::seconds{1}));
    REQUIRE(failed.stage() == CommandStage::kFailed);
    REQUIRE(failed.sent_frames() == 1);
    REQUIRE(failed.history().size() == 3);
    REQUIRE(failed.history()[1].stage == CommandStage::kDispatched);
    REQUIRE_THROWS_AS(std::rethrow_exception(failed.error()), TimeoutError);
    try {
        std::rethrow_exception(failed.error());
    } catch (const HandError &error) {
        REQUIRE_FALSE(error.context().operation.empty());
        REQUIRE_FALSE(error.context().model.empty());
        REQUIRE(error.context().generation > 0);
        REQUIRE(error.context().arbitration_id == 0x28);
    }
    const auto current = device.get_configured_speeds();
    REQUIRE(current.meta.valid);
    REQUIRE(current.raw == previous.raw);
    REQUIRE(current.meta.collected_at == previous.meta.collected_at);
    Message late{};
    late.id = 0x28;
    late.dlc = 7;
    late.data[0] = 0x05;
    std::fill_n(late.data.begin() + 1, 6, 100);
    transport->inject(late);
    REQUIRE(failed.wait_for(std::chrono::milliseconds{1}));
    REQUIRE(device.get_configured_speeds().meta.collected_at == previous.meta.collected_at);
    REQUIRE(failed.history().size() == 3);
    // Classic ACKs have no transaction ID. After an ambiguous timeout the key
    // must remain isolated until recovery, so the old ACK cannot acknowledge
    // a newly accepted handle that happens to use the same opcode.
    auto isolated = device.set_speeds(std::vector<double>(6, 120));
    REQUIRE(isolated.command_id() != failed.command_id());
    REQUIRE(isolated.wait_for(std::chrono::seconds{1}));
    REQUIRE(isolated.stage() == CommandStage::kFailed);
    REQUIRE(isolated.sent_frames() == 0);
    REQUIRE_THROWS_AS(std::rethrow_exception(isolated.error()), DeviceNotReadyError);
    REQUIRE(device.get_configured_speeds().meta.collected_at == previous.meta.collected_at);
}

TEST_CASE("Partial configuration send reports its successful prefix without dispatch",
          "[comm][device][command-receipts]") {
    auto config = config_for(Model::kL20Lite);
    config.command_timeout = std::chrono::milliseconds{300};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL20Lite);
        return next;
    });
    device.open();
    (void)completed(device.set_stall_thresholds(std::vector<double>(10, 30)));
    const auto previous = device.get_stall_thresholds();
    std::atomic<size_t> writes{0};
    transport->on_request(0x28, [&](const Message &) {
        if (++writes == 2) {
            throw linkerbot::can::TransportIOError("second frame write failed");
        }
        return std::vector<Message>{};
    });
    auto failed = device.set_stall_thresholds(std::vector<double>(10, 90));
    REQUIRE(failed.wait_for(std::chrono::seconds{1}));
    REQUIRE(failed.stage() == CommandStage::kFailed);
    REQUIRE(failed.sent_frames() == 1);
    REQUIRE(failed.history().size() == 2);
    REQUIRE_THROWS_AS(std::rethrow_exception(failed.error()), TransportError);
    const auto current = device.get_stall_thresholds();
    REQUIRE(current.meta.valid);
    REQUIRE(current.raw == previous.raw);
    REQUIRE(current.meta.collected_at == previous.meta.collected_at);
    // The executor survives the failure and can serve an independent family.
    transport->on_request(0x28, [](const Message &request) { return classic_angle_reply(Model::kL20Lite, request); });
    REQUIRE(completed(device.set_angles(std::vector<int32_t>(10, 40))).stage() == CommandStage::kDispatched);
}

TEST_CASE("Multi-frame ACK needs every bank and publishes returned values", "[comm][device][command-receipts]") {
    for (const bool missing : {false, true}) {
        DYNAMIC_SECTION("missing bank " << missing) {
            auto config = config_for(Model::kL20Lite);
            config.command_timeout = std::chrono::milliseconds{150};
            FakeTransport *transport = nullptr;
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_classic_identity(transport, Model::kL20Lite);
                return next;
            });
            device.open();
            (void)completed(device.set_stall_detection_times(std::vector<double>(10, 30)));
            const auto previous = device.get_stall_detection_times();
            transport->clear_sent();
            transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
            auto handle = device.set_stall_detection_times(std::vector<double>(10, 80));
            REQUIRE(wait_until([&] { return handle.stage() == CommandStage::kDispatched; }));
            REQUIRE(handle.sent_frames() == 2);
            const auto sent = transport->sent();
            REQUIRE(sent.size() == 2);
            auto bank0 = sent[0], bank1 = sent[1];
            // The first payload byte selects the bank; remaining bytes are values.
            std::fill_n(bank0.data.begin() + 2, 5, 60);
            std::fill_n(bank1.data.begin() + 2, 5, 70);
            transport->inject(bank0);
            transport->inject(bank0);
            REQUIRE_FALSE(handle.wait_for(std::chrono::milliseconds{15}));
            REQUIRE(test_support::require_value(device.get_snapshot().stall_detection_times).meta.collected_at ==
                    previous.meta.collected_at);
            if (!missing) {
                transport->inject(bank1);
            }
            REQUIRE(handle.wait_for(std::chrono::seconds{1}));
            if (missing) {
                REQUIRE(handle.stage() == CommandStage::kFailed);
                REQUIRE_THROWS_AS(std::rethrow_exception(handle.error()), TimeoutError);
                REQUIRE(device.get_stall_detection_times().raw == previous.raw);
            } else {
                REQUIRE(handle.stage() == CommandStage::kAcknowledged);
                auto expected = std::vector<double>(10, 60);
                std::fill(expected.begin() + 5, expected.end(), 70);
                REQUIRE(device.get_stall_detection_times().raw == expected);
            }
        }
    }
}

TEST_CASE("Optional angle feedback cannot change a terminal send receipt", "[comm][device][command-receipts]") {
    auto config = config_for(Model::kL6);
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
    auto handle = device.set_angles(std::vector<int32_t>(6, 100));
    REQUIRE(handle.wait_for(std::chrono::milliseconds{50}));
    REQUIRE(handle.stage() == CommandStage::kDispatched);
    REQUIRE(handle.sent_frames() == 1);
    REQUIRE_FALSE(handle.error());
    Message reply{};
    reply.id = 0x28;
    reply.dlc = 7;
    reply.data[0] = 0x01;
    std::fill_n(reply.data.begin() + 1, 6, 90);
    const auto received_before = device.get_health().session.rx_frames;
    transport->inject(reply);
    REQUIRE(wait_until([&] { return device.get_health().session.rx_frames > received_before; }));
    // The caller may resume after the native 5ms optional window has expired.
    // Whether feedback updates the cache or is discarded, it must not rewrite
    // the terminal receipt. Drain the workers before checking that invariant.
    // Runtime tests exercise successful feedback with an explicit wider window.
    device.close();
    REQUIRE(handle.stage() == CommandStage::kDispatched);
    REQUIRE(handle.history().size() == 2);
    REQUIRE_FALSE(handle.error());
}

TEST_CASE("Claimed driver send after deadline cannot complete or publish a command",
          "[comm][device][command-receipts]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::milliseconds{80};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    (void)completed(device.set_speeds(std::vector<double>(6, 40)));
    // This snapshot is compared after the failure by Catch2 assertions below.
    // NOLINTNEXTLINE(clang-analyzer-deadcode.DeadStores)
    const auto previous = device.get_configured_speeds();
    transport->arm_tx_gate();
    auto handle = device.set_speeds(std::vector<double>(6, 100));
    const auto entered = transport->wait_until_tx_entered(std::chrono::seconds{1});
    const auto expired = handle.wait_for(std::chrono::milliseconds{400});
    const auto stage = handle.stage();
    const auto frames = handle.sent_frames();
    transport->release_tx_gate();
    REQUIRE(entered);
    REQUIRE(expired);
    REQUIRE(stage == CommandStage::kFailed);
    REQUIRE(frames == 0);
    REQUIRE_THROWS_AS(std::rethrow_exception(handle.error()), TimeoutError);
    try {
        std::rethrow_exception(handle.error());
    } catch (const TimeoutError &error) {
        REQUIRE(std::string(error.what()).find("claimed backend I/O") != std::string::npos);
    }
    REQUIRE(wait_until([&] { return device.get_health().pending_commands() == 0; }));
    REQUIRE(handle.sent_frames() == 0);
    REQUIRE(handle.history().size() == 2);
    REQUIRE(device.get_configured_speeds().meta.collected_at == previous.meta.collected_at);
}

TEST_CASE("Sequential HOP writes share one acceptance deadline and final dispatch",
          "[comm][device][command-receipts]") {
    for (const bool expire : {false, true}) {
        DYNAMIC_SECTION("expire second ACK " << expire) {
            auto config = config_for(Model::kO30);
            config.command_timeout = std::chrono::milliseconds{200};
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(Model::kO30, transport, hop);
                return next;
            });
            device.open();
            transport->clear_sent();
            transport->on_request(hop->request_id, [](const Message &) { return std::vector<Message>{}; });
            const auto begun = std::chrono::steady_clock::now();
            auto handle = device.set_speeds(std::vector<double>(20, 80));
            REQUIRE(wait_until([&] { return handle.sent_frames() == 1; }));
            REQUIRE(handle.stage() == CommandStage::kAccepted);
            if (expire) {
                std::this_thread::sleep_until(begun + std::chrono::milliseconds{120});
            }
            const auto first = transport->sent().back();
            transport->inject(o30_comm::hop_reply(hop->response_id(), o30::protocol::kMiSpeed, false, first.data[1],
                                                  std::vector<uint8_t>(first.data[2], 70)));
            REQUIRE(wait_until([&] { return handle.sent_frames() == 2; }));
            REQUIRE(handle.stage() == CommandStage::kAccepted);
            if (expire) {
                REQUIRE(handle.wait_for(std::chrono::seconds{1}));
                const auto ended = handle.history().back().timestamp;
                REQUIRE(ended - handle.history().front().timestamp < std::chrono::milliseconds{290});
                REQUIRE(handle.sent_frames() == 2);
                REQUIRE(handle.history().size() == 2);
                REQUIRE_THROWS_AS(std::rethrow_exception(handle.error()), TimeoutError);
                REQUIRE(transport->sent().size() == 2);
            } else {
                const auto second = transport->sent().back();
                transport->inject(o30_comm::hop_reply(hop->response_id(), o30::protocol::kMiSpeed, false,
                                                      second.data[1], std::vector<uint8_t>(second.data[2], 70)));
                REQUIRE(wait_until([&] { return handle.stage() == CommandStage::kDispatched; }));
                REQUIRE_FALSE(handle.done());
                REQUIRE(handle.sent_frames() == 3);
                const auto third = transport->sent().back();
                transport->inject(o30_comm::hop_reply(hop->response_id(), o30::protocol::kMiSpeed, false, third.data[1],
                                                      std::vector<uint8_t>(third.data[2], 70)));
                REQUIRE(completed(handle).stage() == CommandStage::kAcknowledged);
                REQUIRE(handle.history().size() == 3);
                REQUIRE(test_support::require_value(device.get_snapshot().configured_speeds).raw ==
                        std::vector<double>(20, 70));
                REQUIRE(test_support::require_value(device.get_snapshot().configured_speeds).meta.delivery ==
                        TelemetryDelivery::kCommandResponse);
            }
        }
    }
}

TEST_CASE("An early ACK cannot publish when the claimed transport write fails", "[comm][device][command-receipts]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{1};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    (void)completed(device.set_speeds(std::vector<double>(6, 40)));
    const auto previous = device.get_snapshot().configured_speeds;
    transport->arm_tx_gate();
    auto handle = device.set_speeds(std::vector<double>(6, 100));
    const auto entered = transport->wait_until_tx_entered(std::chrono::seconds{1});
    Message early{};
    early.id = 0x28;
    early.dlc = 7;
    early.data[0] = 0x05;
    std::fill_n(early.data.begin() + 1, 6, 100);
    transport->inject(early);
    const auto pending = !handle.wait_for(std::chrono::milliseconds{20});
    const auto unchanged = test_support::require_value(device.get_snapshot().configured_speeds).meta.collected_at ==
                           test_support::require_value(previous).meta.collected_at;
    transport->fail_writes("claimed write failed after incoming ACK");
    transport->release_tx_gate();
    REQUIRE(entered);
    REQUIRE(pending);
    REQUIRE(unchanged);
    REQUIRE(handle.wait_for(std::chrono::seconds{1}));
    REQUIRE(handle.stage() == CommandStage::kFailed);
    REQUIRE(handle.sent_frames() == 0);
    REQUIRE(handle.history().size() == 2);
    const auto receipts = handle.send_results();
    REQUIRE(receipts.size() == 1);
    REQUIRE(receipts.front().outcome == linkerbot::transport::SendOutcome::failed);
    REQUIRE(receipts.front().current_frame_uncertain);
    REQUIRE(receipts.front().claimed_at.has_value());
    REQUIRE(receipts.front().sent_frames == 0);
    REQUIRE_THROWS_AS(std::rethrow_exception(handle.error()), TransportError);
    REQUIRE(test_support::require_value(device.get_snapshot().configured_speeds).meta.collected_at ==
            test_support::require_value(previous).meta.collected_at);
}

TEST_CASE("HOP negative ACK preserves protocol code and leaves the executor usable",
          "[comm][device][command-receipts]") {
    auto config = config_for(Model::kO30);
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kO30, transport, hop);
        return next;
    });
    device.open();
    (void)completed(device.set_speeds(std::vector<double>(20, 50)));
    const auto previous = device.get_snapshot().configured_speeds;
    transport->on_request(hop->request_id, [hop](const Message &request) {
        if (request.data[1] != o30::kSafeWriteSegments.back().si) {
            return hop->reply(request);
        }
        return std::vector<Message>{
            o30_comm::error_reply(hop->response_id(), request.data[1], o30::protocol::kErrorSubIndexMissing)};
    });
    auto handle = device.set_speeds(std::vector<double>(20, 80));
    REQUIRE(handle.wait_for(std::chrono::seconds{1}));
    REQUIRE(handle.stage() == CommandStage::kFailed);
    REQUIRE(handle.sent_frames() == 3);
    REQUIRE(handle.history().size() == 3);
    REQUIRE(handle.history()[1].stage == CommandStage::kDispatched);
    REQUIRE_THROWS_AS(std::rethrow_exception(handle.error()), ProtocolError);
    try {
        std::rethrow_exception(handle.error());
    } catch (const HandError &error) {
        REQUIRE(error.context().native_status == o30::protocol::kErrorSubIndexMissing);
        REQUIRE(error.context().arbitration_id == hop->request_id);
        REQUIRE(error.context().generation > 0);
    }
    REQUIRE(test_support::require_value(device.get_snapshot().configured_speeds).meta.collected_at ==
            previous->meta.collected_at);
    o30_comm::attach_hop(transport, hop);
    REQUIRE(completed(device.set_speeds(std::vector<double>(20, 60))).stage() == CommandStage::kAcknowledged);
    REQUIRE(device.get_snapshot().configured_speeds->raw == std::vector<double>(20, 60));
}

TEST_CASE("O30 configuration caches use complete returned mounted ranges", "[comm][device][command-receipts]") {
    auto config = config_for(Model::kO30);
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kO30, transport, hop);
        return next;
    });
    device.open();
    transport->on_request(hop->request_id, [hop](const Message &request) {
        if (request.dlc <= o30::protocol::kHeaderBytes) {
            return hop->reply(request);
        }
        return std::vector<Message>{o30_comm::hop_reply(hop->response_id(), o30::protocol::mi_of(request.data[0]),
                                                        o30::protocol::rts_of(request.data[0]), request.data[1],
                                                        std::vector<uint8_t>(request.data[2], 60))};
    });
    using Setter = CommandHandle (HandDevice::*)(const std::vector<double> &);
    struct Item {
        Setter set;
        std::function<std::vector<double>(const HandSnapshot &)> raw;
    };
    const std::vector<Item> items{
        {&HandDevice::set_speeds, [](const auto &s) { return s.configured_speeds.value().raw; }},
        {&HandDevice::set_accelerations, [](const auto &s) { return s.configured_accelerations.value().raw; }},
        {&HandDevice::set_current_limits, [](const auto &s) { return s.current_limits.value().raw; }},
        {&HandDevice::set_stall_detection_times, [](const auto &s) { return s.stall_detection_times.value().raw; }},
        {&HandDevice::set_stall_thresholds, [](const auto &s) { return s.stall_thresholds.value().raw; }},
        {&HandDevice::set_stall_hold_currents, [](const auto &s) { return s.stall_hold_currents.value().raw; }},
    };
    for (size_t i = 0; i < items.size(); ++i) {
        CAPTURE(i);
        auto handle = completed((device.*items[i].set)(std::vector<double>(20, 80)));
        REQUIRE(handle.stage() == CommandStage::kAcknowledged);
        REQUIRE(items[i].raw(device.get_snapshot()) == std::vector<double>(20, 60));
    }
    // A short write ACK is legal in the existing HOP protocol, but does not
    // contain the complete mounted telemetry needed to replace a cache.
    const auto previous = device.get_snapshot().configured_speeds;
    transport->on_request(hop->request_id, [hop](const Message &request) {
        return std::vector<Message>{
            o30_comm::hop_reply(hop->response_id(), o30::protocol::kMiSpeed, false, request.data[1], {90})};
    });
    REQUIRE(completed(device.set_speeds(std::vector<double>(20, 90))).stage() == CommandStage::kAcknowledged);
    REQUIRE(test_support::require_value(device.get_snapshot().configured_speeds).raw ==
            test_support::require_value(previous).raw);
    REQUIRE(test_support::require_value(device.get_snapshot().configured_speeds).meta.collected_at ==
            test_support::require_value(previous).meta.collected_at);
}

namespace {
    struct SendGateGuard {
        FakeTransport *transport;
        bool active = true;
        ~SendGateGuard() { release(); }
        void release() {
            if (active) {
                transport->release_tx_gate();
                active = false;
            }
        }
    };

    bool closed_command(const CommandHandle &handle) {
        if (!handle.done() || handle.stage() != CommandStage::kFailed || !handle.error()) {
            return false;
        }
        try {
            std::rethrow_exception(handle.error());
        } catch (const ClosedError &error) {
            return !error.context().operation.empty() && error.context().generation != 0;
        } catch (...) {
            return false;
        }
    }
}

TEST_CASE("Close terminates claimed and pending setters within its shutdown budget",
          "[comm][device][command-lifecycle]") {
    for (const auto model : kModels) {
        DYNAMIC_SECTION("model " << static_cast<int>(model)) {
            auto config = config_for(model);
            config.command_timeout = std::chrono::seconds{3};
            // Opening/reopening builds a multi-frame baseline on Windows. It is
            // outside the shutdown timing contract exercised by the gated write.
            config.transaction_timeout = std::chrono::seconds{1};
            config.shutdown_timeout = std::chrono::milliseconds{80};
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(model, transport, hop);
                return next;
            });
            device.open();
            transport->clear_sent();
            transport->arm_tx_gate();
            SendGateGuard gate{transport};
            const auto joints = device.get_capabilities().joints.size();
            auto active = device.set_angles(std::vector<int32_t>(joints, 50));
            const bool entered = transport->wait_until_tx_entered(std::chrono::seconds{2});
            auto queued = device.set_angles(std::vector<int32_t>(joints, 60));
            auto waiting =
                std::async(std::launch::async, [active] { return active.wait_for(std::chrono::seconds{1}); });
            auto closing = std::async(std::launch::async, [&] {
                try {
                    device.close();
                } catch (const TimeoutError &) {
                    return true;
                }
                return false;
            });
            const bool bounded = closing.wait_for(std::chrono::milliseconds{700}) == std::future_status::ready;
            const bool all_terminal =
                active.wait_for(std::chrono::milliseconds{100}) && queued.wait_for(std::chrono::milliseconds{100});
            const auto history = active.history();
            const bool both_closed = closed_command(active) && closed_command(queued);
            const auto frames = active.sent_frames();
            const auto health = device.get_health();
            const bool awake = waiting.wait_for(std::chrono::milliseconds{100}) == std::future_status::ready;
            gate.release(); // No assertion can retain a claimed driver.
            const bool timed_out = closing.get();
            REQUIRE(entered);
            REQUIRE(bounded);
            REQUIRE(timed_out);
            REQUIRE(all_terminal);
            REQUIRE(both_closed);
            REQUIRE(awake);
            REQUIRE(waiting.get());
            REQUIRE(frames == 0);
            REQUIRE(queued.sent_frames() == 0);
            REQUIRE(health.pending_commands() == 0);
            REQUIRE_FALSE(health.identity_verified);
            REQUIRE_NOTHROW(device.close()); // Join retained cleanup, idempotently.
            REQUIRE(active.history().size() == history.size());
            REQUIRE(active.sent_frames() == frames);
            uint64_t failed = 0;
            for (const auto &family : device.get_health().command_families) {
                failed += family.failed;
            }
            REQUIRE(failed == 2);
            REQUIRE_NOTHROW(device.close());
            REQUIRE_NOTHROW(device.open());
            REQUIRE(device.get_health().identity_verified);
            // New generation only sends the baseline; neither accepted angle
            // target is replayed on the replacement transport.
            for (const auto &message : transport->sent()) {
                if (model == Model::kL6 || model == Model::kO6 || model == Model::kL20 || model == Model::kL20Lite) {
                    REQUIRE_FALSE((message.data[0] == 0x01 && message.dlc > 1));
                }
            }
        }
    }
}

TEST_CASE("Close preserves completed success and terminates ACK waiters without late mutation",
          "[comm][device][command-lifecycle]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{3};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    auto success = completed(device.set_speeds(std::vector<double>(6, 40)));
    transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
    auto active = device.set_speeds(std::vector<double>(6, 80));
    REQUIRE(wait_until([&] { return active.stage() == CommandStage::kDispatched; }));
    auto queued = device.set_stall_thresholds(std::vector<double>(6, 30));
    auto waiter1 = std::async(std::launch::async, [active] { return active.wait(); });
    auto waiter2 = std::async(std::launch::async, [queued] { return queued.wait(); });
    device.close();
    REQUIRE(waiter1.get() == CommandStage::kFailed);
    REQUIRE(waiter2.get() == CommandStage::kFailed);
    REQUIRE(closed_command(active));
    REQUIRE(closed_command(queued));
    REQUIRE(active.sent_frames() == 1);
    REQUIRE(queued.sent_frames() == 0);
    REQUIRE(success.stage() == CommandStage::kAcknowledged);
    REQUIRE_FALSE(success.error());
    REQUIRE(device.get_health().pending_commands() == 0);
    REQUIRE_NOTHROW(device.close());
}

TEST_CASE("Recover cancels old ACK and pending work before identity and baseline reopen admission",
          "[comm][device][command-lifecycle]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{3};
    config.shutdown_timeout = std::chrono::seconds{3};
    FakeTransport *transport = nullptr;
    std::atomic<size_t> opens{0};
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        if (++opens > 1) {
            next->arm_tx_gate();
        }
        return next;
    });
    device.open();
    const auto old_generation = device.get_health().connection_generation;
    transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
    auto active = device.set_speeds(std::vector<double>(6, 80));
    REQUIRE(wait_until([&] { return active.stage() == CommandStage::kDispatched; }));
    auto queued = device.set_stall_thresholds(std::vector<double>(6, 30));
    auto recovery = std::async(std::launch::async, [&] { device.recover(); });
    const bool factory_ran = wait_until([&] { return opens.load() > 1; });
    auto *replacement = transport;
    SendGateGuard gate{replacement};
    const bool entered = replacement->wait_until_tx_entered(std::chrono::seconds{2});
    const bool terminal =
        active.wait_for(std::chrono::milliseconds{100}) && queued.wait_for(std::chrono::milliseconds{100});
    bool rejected = false;
    try {
        (void)device.set_angles(std::vector<int32_t>(6, 50));
    } catch (const DeviceNotReadyError &) {
        rejected = true;
    }
    const auto health = device.get_health();
    gate.release();
    recovery.get();
    REQUIRE(factory_ran);
    REQUIRE(entered);
    REQUIRE(terminal);
    REQUIRE(rejected);
    REQUIRE(closed_command(active));
    REQUIRE(closed_command(queued));
    REQUIRE(active.sent_frames() == 1);
    REQUIRE(queued.sent_frames() == 0);
    REQUIRE(health.lifecycle_state == LifecycleState::kRecovering);
    REQUIRE_FALSE(health.identity_verified);
    const auto after = device.get_snapshot();
    REQUIRE(after.connection_generation > old_generation);
    REQUIRE_FALSE(after.configured_speeds);
    REQUIRE(device.get_health().identity_verified);
    for (const auto &message : replacement->sent()) {
        REQUIRE_FALSE((message.data[0] == 0x05 || message.data[0] == 0xC6));
    }
    auto new_command = completed(device.set_speeds(std::vector<double>(6, 20)));
    REQUIRE(new_command.command_id() > queued.command_id());
}

TEST_CASE("Recovery timeout cannot publish Ready after a claimed send eventually returns",
          "[comm][device][command-lifecycle]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{3};
    config.shutdown_timeout = std::chrono::milliseconds{80};
    FakeTransport *transport = nullptr;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device.open();
    auto *old = transport;
    old->arm_tx_gate();
    SendGateGuard gate{old};
    auto active = device.set_speeds(std::vector<double>(6, 80));
    const bool entered = old->wait_until_tx_entered(std::chrono::seconds{2});
    auto queued = device.set_stall_thresholds(std::vector<double>(6, 30));
    auto recovery = std::async(std::launch::async, [&] {
        try {
            device.recover();
        } catch (const TimeoutError &) {
            return true;
        }
        return false;
    });
    const bool bounded = recovery.wait_for(std::chrono::milliseconds{700}) == std::future_status::ready;
    const bool terminal =
        active.wait_for(std::chrono::milliseconds{100}) && queued.wait_for(std::chrono::milliseconds{100});
    const auto health = device.get_health();
    gate.release();
    const bool timed_out = recovery.get();
    REQUIRE(entered);
    REQUIRE(bounded);
    REQUIRE(timed_out);
    REQUIRE(terminal);
    REQUIRE(closed_command(active));
    REQUIRE(closed_command(queued));
    REQUIRE(health.lifecycle_state == LifecycleState::kFaulted);
    REQUIRE_FALSE(device.is_open());
    REQUIRE_THROWS_AS(device.set_angles(std::vector<int32_t>(6, 50)), DeviceNotReadyError);
    // close joins the retained recovery before cleaning its backend. It cannot
    // race a late baseline into Ready or allow the old queue to run.
    REQUIRE_NOTHROW(device.close());
    REQUIRE_FALSE(device.is_open());
    REQUIRE_NOTHROW(device.open());
    REQUIRE_FALSE(device.get_snapshot().configured_speeds);
}

TEST_CASE("Handles outlive facade destruction while claimed I O resources remain safely owned",
          "[comm][device][command-lifecycle]") {
    struct TrackedTransport : FakeTransport {
        std::shared_ptr<std::promise<void>> destroyed;
        explicit TrackedTransport(std::shared_ptr<std::promise<void>> done) : destroyed(std::move(done)) {}
        ~TrackedTransport() override { destroyed->set_value(); }
    };
    auto destroyed = std::make_shared<std::promise<void>>();
    auto gone = destroyed->get_future();
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{3};
    config.shutdown_timeout = std::chrono::milliseconds{80};
    FakeTransport *transport = nullptr;
    auto device = std::make_unique<HandDevice>(config, [&] {
        auto next = std::make_unique<TrackedTransport>(destroyed);
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    device->open();
    transport->arm_tx_gate();
    SendGateGuard gate{transport};
    auto active = device->set_speeds(std::vector<double>(6, 80));
    const bool entered = transport->wait_until_tx_entered(std::chrono::seconds{2});
    auto queued = device->set_stall_thresholds(std::vector<double>(6, 30));
    auto destroying = std::async(std::launch::async, [&] { device.reset(); });
    const bool bounded = destroying.wait_for(std::chrono::milliseconds{700}) == std::future_status::ready;
    const bool terminal =
        active.wait_for(std::chrono::milliseconds{100}) && queued.wait_for(std::chrono::milliseconds{100});
    const bool retained = gone.wait_for(std::chrono::milliseconds{0}) == std::future_status::timeout;
    gate.release();
    destroying.get();
    REQUIRE(entered);
    REQUIRE(bounded);
    REQUIRE(terminal);
    REQUIRE(retained);
    REQUIRE(gone.wait_for(std::chrono::seconds{2}) == std::future_status::ready);
    REQUIRE(closed_command(active));
    REQUIRE(closed_command(queued));
    REQUIRE(active.wait() == CommandStage::kFailed);
    REQUIRE(active.sent_frames() == 0);
    REQUIRE(wait_until([&] { return !active.send_results().empty(); }));
    REQUIRE(active.send_results().front().outcome == linkerbot::transport::SendOutcome::dispatched);
    REQUIRE(active.send_results().front().sent_frames == 1);
    REQUIRE(queued.send_results().empty());
}

TEST_CASE("Runtime callback close is rejected before self join and facade destruction is deferred",
          "[comm][device][command-lifecycle]") {
    auto config = config_for(Model::kL6);
    FakeTransport *transport = nullptr;
    auto session = linkerbot::transport::Session::create([&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        return next;
    });
    auto device = std::make_unique<HandDevice>(config, session);
    device->open();
    auto runtime = linkerbot::hand::runtime::DeviceRuntime::acquire(session, "l6/40", {{0x28, false}});
    const auto provenance = runtime->response_provenance();
    std::promise<bool> callback_result;
    auto result = callback_result.get_future();
    runtime->subscribe("lifecycle-test", [&](const auto &) {
        bool rejected = false;
        try {
            device->close();
        } catch (const LifecycleError &) {
            rejected = true;
        }
        device.reset();
        callback_result.set_value(rejected);
    });
    runtime->publish("lifecycle-test", 1, provenance);
    REQUIRE(result.wait_for(std::chrono::seconds{2}) == std::future_status::ready);
    REQUIRE(result.get());
    REQUIRE_FALSE(device);
    REQUIRE(wait_until([&] { return runtime->health().lifecycle_state == LifecycleState::kClosed; }));
    session->close();
}

TEST_CASE("Lifecycle interruption preserves known partial HOP sends and never replays remaining ranges",
          "[comm][device][command-lifecycle]") {
    for (const bool recover : {false, true}) {
        DYNAMIC_SECTION("recover " << recover) {
            auto config = config_for(Model::kO30);
            config.command_timeout = std::chrono::seconds{3};
            FakeTransport *transport = nullptr;
            std::shared_ptr<o30_comm::HopEndpoint> hop;
            HandDevice device(config, [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_fake_device(Model::kO30, transport, hop);
                return next;
            });
            device.open();
            transport->clear_sent();
            transport->on_request(hop->request_id, [](const Message &) { return std::vector<Message>{}; });
            auto partial = device.set_speeds(std::vector<double>(20, 80));
            REQUIRE(wait_until([&] { return partial.sent_frames() == 1; }));
            REQUIRE(partial.stage() == CommandStage::kAccepted);
            REQUIRE(transport->sent().size() == 1);
            auto queued = device.set_speeds(std::vector<double>(20, 90));
            if (recover) {
                device.recover();
            } else {
                device.close();
            }
            REQUIRE(closed_command(partial));
            REQUIRE(closed_command(queued));
            REQUIRE(partial.sent_frames() == 1);
            REQUIRE(queued.sent_frames() == 0);
            REQUIRE(partial.history().size() == 2);
            if (!recover) {
                device.open();
            }
            REQUIRE_FALSE(device.get_snapshot().configured_speeds);
            for (const auto &message : transport->sent()) {
                REQUIRE_FALSE((message.is_fd && message.data[0] == o30::protocol::kMiSpeed));
            }
        }
    }
}

TEST_CASE("Concurrent close interrupts recovery without resurrecting admission or duplicating handle completion",
          "[comm][device][command-lifecycle]") {
    auto config = config_for(Model::kL6);
    config.command_timeout = std::chrono::seconds{3};
    config.shutdown_timeout = std::chrono::seconds{2};
    FakeTransport *transport = nullptr;
    std::atomic<size_t> opens{0};
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL6);
        if (++opens > 1) {
            next->arm_tx_gate();
        }
        return next;
    });
    device.open();
    transport->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
    auto active = device.set_speeds(std::vector<double>(6, 80));
    REQUIRE(wait_until([&] { return active.stage() == CommandStage::kDispatched; }));
    auto recovery = std::async(std::launch::async, [&] {
        try {
            device.recover();
        } catch (const HandError &) {
            return true;
        }
        return false;
    });
    const bool reconnected = wait_until([&] { return opens.load() > 1; });
    SendGateGuard gate{transport};
    const bool entered = transport->wait_until_tx_entered(std::chrono::seconds{1});
    auto closing1 = std::async(std::launch::async, [&] { device.close(); });
    const bool closing = wait_until([&] { return device.get_health().lifecycle_state == LifecycleState::kClosing; });
    auto closing2 = std::async(std::launch::async, [&] { device.close(); });
    const bool failed = active.wait_for(std::chrono::milliseconds{100});
    gate.release();
    const bool aborted = recovery.get();
    closing1.get();
    closing2.get();
    REQUIRE(reconnected);
    REQUIRE(entered);
    REQUIRE(closing);
    REQUIRE(failed);
    REQUIRE(aborted);
    REQUIRE(closed_command(active));
    REQUIRE(active.history().size() == 3);
    REQUIRE(device.get_health().lifecycle_state == LifecycleState::kClosed);
    REQUIRE(device.get_health().pending_commands() == 0);
    REQUIRE_FALSE(device.is_open());
    REQUIRE_NOTHROW(device.close());
}

TEST_CASE("O6 configured speed reads preserve the acknowledged setting without IO", "[comm][device][o6][speed]") {
    FakeTransport *transport = nullptr;
    HandDevice device(config_for(Model::kO6), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        std::shared_ptr<o30_comm::HopEndpoint> unused;
        install_fake_device(Model::kO6, transport, unused);
        return next;
    });
    device.open();
    const auto capabilities = device.get_capabilities();
    const auto *channel = capabilities.telemetry_channel(TelemetrySemantic::kConfiguredSpeeds);
    REQUIRE(channel != nullptr);
    REQUIRE(channel->read_mode == ReadMode::kCommandResponse);
    REQUIRE_FALSE(channel->device_queried);
    const auto before = transport->sent().size();
    REQUIRE_THROWS_AS(device.get_configured_speeds(), DeviceNotReadyError);
    REQUIRE(transport->sent().size() == before);
    transport->on_request(0x28, [](const Message &request) {
        if (request.dlc == 7 && request.data[0] == 0x05) {
            return std::vector<Message>{request};
        }
        // V1.3.2 treats the bare opcode as zero speed, not a query.
        if (request.dlc == 1 && request.data[0] == 0x05) {
            Message response{};
            response.id = request.id;
            response.dlc = 7;
            response.data[0] = 0x05;
            return std::vector<Message>{response};
        }
        return classic_angle_reply(Model::kO6, request);
    });
    REQUIRE(completed(device.set_speeds(std::vector<double>(6, 20))).stage() == CommandStage::kAcknowledged);
    const auto sent = transport->sent().size();
    for (int repeat = 0; repeat < 2; ++repeat) {
        const auto sample = device.get_configured_speeds();
        REQUIRE(sample.raw == std::vector<double>(6, 20));
        REQUIRE(sample.meta.delivery == TelemetryDelivery::kCommandResponse);
        REQUIRE(sample.meta.origin == TelemetryValueOrigin::kCommandCache);
        REQUIRE(transport->sent().size() == sent);
    }
    device.recover();
    const auto recovered = transport->sent().size();
    REQUIRE_THROWS_AS(device.get_configured_speeds(), DeviceNotReadyError);
    REQUIRE(transport->sent().size() == recovered);
    device.close();
}

TEST_CASE("L20Lite verified firmware rejects unavailable scalar force without IO",
          "[comm][device][l20lite][force-fields]") {
    for (const auto firmware : {std::array<uint8_t, 3>{4, 6, 10}, std::array<uint8_t, 3>{4, 6, 11}}) {
        DYNAMIC_SECTION("firmware patch " << unsigned(firmware[2])) {
            FakeTransport *transport = nullptr;
            HandDevice device(config_for(Model::kL20Lite), [&] {
                auto next = std::make_unique<FakeTransport>();
                transport = next.get();
                install_classic_identity(transport, Model::kL20Lite, {}, 0x28, true, std::chrono::milliseconds{0},
                                         firmware);
                return next;
            });
            device.open();
            for (int phase = 0; phase < 3; ++phase) {
                if (phase == 1) {
                    (void)device.verify_identity();
                }
                if (phase == 2) {
                    device.recover();
                }
                const auto caps = device.get_capabilities();
                REQUIRE(caps.validate());
                REQUIRE(caps.supports(TelemetrySemantic::kForce));
                REQUIRE(caps.force_field("fn").supported == true);
                const bool scalar_supported = firmware[2] != 10;
                REQUIRE(caps.supports(TelemetrySemantic::kForceFns) == scalar_supported);
                for (const auto name : {"fns", "fts", "fds", "px"}) {
                    REQUIRE(caps.force_field(name).supported == scalar_supported);
                    if (!scalar_supported) {
                        const auto sent = transport->sent().size();
                        REQUIRE_THROWS_AS(device.get_force({name}), UnsupportedOperationError);
                        REQUIRE(transport->sent().size() == sent);
                    }
                }
                if (!scalar_supported) {
                    const auto sent = transport->sent().size();
                    REQUIRE_THROWS_AS(device.get_normal_force_resultant(), UnsupportedOperationError);
                    REQUIRE(transport->sent().size() == sent);
                }
            }
            device.close();
        }
    }
}

TEST_CASE("L20Lite measured speed never populates or overwrites configured speed",
          "[comm][device][l20lite][configured-speed]") {
    FakeTransport *transport = nullptr;
    HandDevice device(config_for(Model::kL20Lite), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL20Lite);
        return next;
    });
    device.open();
    transport->on_request(0x28, [](const Message &request) {
        auto reply = request;
        if (request.data[0] == 0x05 || request.data[0] == 0x06) {
            reply.dlc = 6;
            if (request.dlc == 1) {
                for (size_t i = 1; i < 6; ++i) {
                    reply.data[i] = 0;
                }
            }
            return std::vector<Message>{reply};
        }
        return classic_angle_reply(Model::kL20Lite, request);
    });
    REQUIRE(device.get_speeds().raw == std::vector<double>(10, 0));
    REQUIRE_THROWS_AS(device.get_configured_speeds(), DeviceNotReadyError);
    REQUIRE_FALSE(device.get_snapshot().configured_speeds.has_value());
    REQUIRE(completed(device.set_speeds(std::vector<double>(10, 20))).stage() == CommandStage::kAcknowledged);
    const auto configured = device.get_configured_speeds();
    REQUIRE(configured.raw == std::vector<double>(10, 20));
    REQUIRE(configured.meta.delivery == TelemetryDelivery::kCommandResponse);
    REQUIRE(device.get_speeds().raw == std::vector<double>(10, 0));
    const auto sent = transport->sent().size();
    REQUIRE(device.get_configured_speeds().raw == configured.raw);
    REQUIRE(transport->sent().size() == sent);
    const auto snapshot = device.get_snapshot();
    REQUIRE(test_support::require_value(snapshot.measured_speeds).raw == std::vector<double>(10, 0));
    REQUIRE(test_support::require_value(snapshot.configured_speeds).raw == std::vector<double>(10, 20));
    REQUIRE(snapshot.joints[0].measured_speed_raw == 0);
    REQUIRE(snapshot.joints[0].configured_speed_raw == 20);
    device.recover();
    REQUIRE_THROWS_AS(device.get_configured_speeds(), DeviceNotReadyError);
    device.close();
}

TEST_CASE("unified polling accepts deployed measured-speed and voltage channels", "[comm][python-phase1]") {
    using namespace std::chrono_literals;
    for (const auto model : {Model::kL20Lite, Model::kL20, Model::kL30, Model::kO30}) {
        CAPTURE(model);
        FakeTransport *raw = nullptr;
        std::shared_ptr<o30_comm::HopEndpoint> hop;
        auto config = config_for(model);
        // This test checks channel support, not the default initialization deadline.
        // Allow the FakeTransport workers to start on loaded platform runners.
        config.transaction_timeout = 2s;
        HandDevice device(config, [&] {
            auto transport = std::make_unique<FakeTransport>();
            raw = transport.get();
            install_fake_telemetry(model, raw, hop);
            install_fake_identity(model, raw, hop);
            if (model == Model::kL30) {
                raw->on_request(l30_comm::query_id(l30::protocol::kSubSpeed),
                                l30_comm::i16_handler(l30_comm::filled_i16(7)));
            }
            return transport;
        });
        {
            INFO("initializing the offline polling fixture");
            device.open();
        }
        PollingPlan plan{{TelemetrySemantic::kMeasuredSpeeds, 20ms}};
        if (model == Model::kO30) {
            plan.push_back({TelemetrySemantic::kVoltages, 20ms});
        }
        raw->clear_sent();
        device.start_polling(plan);
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        bool received = false;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto snapshot = device.get_snapshot();
            received = snapshot.measured_speeds && snapshot.measured_speeds->meta.valid &&
                       (model != Model::kO30 || (snapshot.voltages && snapshot.voltages->meta.valid));
            if (received) {
                break;
            }
            std::this_thread::sleep_for(2ms);
        }
        device.stop_polling();
        REQUIRE((received));
        REQUIRE_FALSE(raw->sent().empty());
        device.close();
    }
}

TEST_CASE("Python runtime initializes reads and recovers each Classic and L30 model", "[comm][python-phase2]") {
    for (const auto model : {Model::kL6, Model::kO6, Model::kL20Lite, Model::kL20, Model::kL30}) {
        CAPTURE(model);
        auto config = config_for(model);
        config.runtime_behavior = RuntimeBehavior::kPython;
        config.response_timeout = std::chrono::milliseconds{200};
        std::shared_ptr<o30_comm::HopEndpoint> hop;
        HandDevice device(config, [&] {
            auto transport = std::make_unique<FakeTransport>();
            install_fake_device(model, transport.get(), hop);
            return transport;
        });
        device.open();
        REQUIRE((device.get_angles(std::chrono::seconds{2}).meta.valid));
        REQUIRE((device.refresh_device_info(std::chrono::seconds{2}).model == model));
        REQUIRE((device.verify_identity(std::chrono::seconds{2}).model == model));
        device.recover_for(std::chrono::seconds{2});
        REQUIRE((device.is_open()));
        REQUIRE((device.get_cached_snapshot().connection_generation == 2));
        device.close();
        REQUIRE_FALSE(device.is_open());
    }
}

TEST_CASE("Python published metadata is stable across Classic and L30 models", "[comm][python-phase3]") {
    using namespace std::chrono_literals;
    for (const auto model : {Model::kL6, Model::kO6, Model::kL20Lite, Model::kL20, Model::kL30}) {
        CAPTURE(model);
        auto config = config_for(model);
        config.runtime_behavior = RuntimeBehavior::kPython;
        config.route_queue_capacity = 32;
        config.response_timeout = 200ms;
        std::shared_ptr<o30_comm::HopEndpoint> hop;
        HandDevice device(config, [&] {
            auto transport = std::make_unique<FakeTransport>();
            install_fake_device(model, transport.get(), hop);
            return transport;
        });
        device.open();
        const auto initial = device.get_cached_snapshot();
        REQUIRE((test_support::require_value(initial.angles).meta.sequence == 1));
        REQUIRE((test_support::require_value(initial.faults).meta.sequence == 1));
        (void)device.get_angles(2s);
        (void)device.get_angles(2s);
        const auto updated = device.get_cached_snapshot();
        REQUIRE((test_support::require_value(updated.angles).meta.sequence == 3));
        REQUIRE((test_support::require_value(updated.faults).meta.sequence == 1));
        REQUIRE((test_support::require_value(updated.device_info).meta.sequence == 1));
        REQUIRE((test_support::require_value(updated.device_info).meta.valid));
        device.close();
        device.open();
        const auto reopened = device.get_cached_snapshot();
        REQUIRE((reopened.connection_generation == 2));
        REQUIRE((device.get_health().connection_generation == reopened.connection_generation));
        for (const auto &channel : device.get_health().channels) {
            REQUIRE((channel.data_age.generation == 2));
        }
        REQUIRE((test_support::require_value(reopened.angles).meta.sequence == 1));
        REQUIRE((test_support::require_value(reopened.angles).meta.connection_generation == 2));
        REQUIRE((test_support::require_value(reopened.device_info).meta.connection_generation == 2));
        REQUIRE((test_support::require_value(device.get_identity()).meta.connection_generation == 2));
        REQUIRE((device.get_angles(2s).meta.connection_generation == 2));
        device.close();
    }
}

TEST_CASE("Python L20Lite scalar queries retain independent data and degradation", "[comm][python-phase3]") {
    using namespace std::chrono_literals;
    auto config = config_for(Model::kL20Lite);
    config.runtime_behavior = RuntimeBehavior::kPython;
    config.response_timeout = 30ms;
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL20Lite, transport, hop);
        return next;
    });
    device.open();
    const auto count = transport->sent().size();
    const auto normal = device.get_force({"fns"}, 1s);
    REQUIRE((transport->sent().size() == count + 1));
    REQUIRE((transport->sent().back().data[0] == 0x20));
    REQUIRE((normal.field("fns") != nullptr));
    REQUIRE((normal.field("fts") == nullptr));
    const auto sequence = normal.field("fns")->meta.sequence;
    const auto tangential = device.get_force({"fts"}, 1s);
    REQUIRE((transport->sent().back().data[0] == 0x21));
    REQUIRE((tangential.field("fns") == nullptr));
    const auto cached = device.get_cached_snapshot();
    REQUIRE((test_support::require_value(cached.force).field("fns")->meta.sequence == sequence));
    REQUIRE((test_support::require_value(cached.force).field("fts")->meta.sequence == 1));
    transport->on_request(0x28, [](const Message &request) {
        if (request.data[0] == 0x23) {
            return std::vector<Message>{};
        }
        return classic_angle_reply(Model::kL20Lite, request);
    });
    REQUIRE_THROWS_AS(device.get_force({"px"}, 1s), TimeoutError);
    REQUIRE_THROWS_AS(device.get_force({"px"}, 1s), TimeoutError);
    REQUIRE_THROWS_AS(device.get_force({"px"}, 1s), UnsupportedFirmwareError);
    REQUIRE((device.get_capabilities().force_field("px").firmware_unsupported));
    REQUIRE((device.get_capabilities().force_field("fns").supported == true));
    REQUIRE((device.get_force({"fns"}, 1s).field("fns")->meta.sequence > sequence));
    REQUIRE((test_support::require_value(device.get_cached_snapshot().force).status("px") == "unsupported"));
    device.close();
}

TEST_CASE("Python optional polling degradation preserves other poll channels and resets on recovery",
          "[comm][python-phase3]") {
    using namespace std::chrono_literals;
    auto config = config_for(Model::kL6);
    config.runtime_behavior = RuntimeBehavior::kPython;
    config.response_timeout = 20ms;
    FakeTransport *transport = nullptr;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config, [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_fake_device(Model::kL6, transport, hop);
        return next;
    });
    device.open();
    transport->on_request(0x28, [](const Message &request) {
        if (request.data[0] == 0x36) {
            return std::vector<Message>{};
        }
        return classic_angle_reply(Model::kL6, request);
    });
    device.start_polling({{TelemetrySemantic::kCurrents, 10ms}, {TelemetrySemantic::kAngles, 10ms}});
    REQUIRE((wait_until([&] { return !device.get_capabilities().supports(TelemetrySemantic::kCurrents); })));
    REQUIRE((device.get_health().connected()));
    const auto before = test_support::require_value(device.get_cached_snapshot().angles).meta.sequence;
    REQUIRE((wait_until(
        [&] { return test_support::require_value(device.get_cached_snapshot().angles).meta.sequence > before; })));
    device.stop_polling();
    device.recover_for(2s);
    REQUIRE((device.get_capabilities().supports(TelemetrySemantic::kCurrents)));
    REQUIRE((device.get_currents(1s).meta.valid));
    device.close();
}

TEST_CASE("Python O30 no-sensor downgrade is recorded after successful metadata queries", "[comm][python-phase4]") {
    using namespace std::chrono_literals;
    auto config = config_for(Model::kO30);
    config.runtime_behavior = RuntimeBehavior::kPython;
    config.o30_layout = O30Layout::kPythonV25;
    config.side = HandSide::kLeft;
    config.identity_timeout = 300ms;
    config.response_timeout = 300ms;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config, [&] {
        auto t = std::make_unique<FakeTransport>();
        hop = std::make_shared<o30_comm::HopEndpoint>();
        hop->request_id = 2;
        install_fake_device(Model::kO30, t.get(), hop);
        hop->put_ascii(0x41, 0x61, 8, "0.0.4");
        std::vector<uint8_t> metadata(0x73, 0);
        const std::string type = "NO_SENSOR";
        std::copy(type.begin(), type.end(), metadata.begin());
        hop->put(o30::protocol::kMiSensorInfo, metadata);
        return t;
    });
    device.open();
    REQUIRE_THROWS_AS(device.get_force(3s), UnsupportedFirmwareError);
    const auto health = device.get_health();
    REQUIRE((test_support::require_value(health.recent_error).code == ErrorCode::kUnsupportedFirmware));
    REQUIRE((health.consecutive_errors == 1));
    REQUIRE_FALSE(device.get_capabilities().supports(TelemetrySemantic::kForce));
    device.close();
}

TEST_CASE("Python O30 force fields project one completed shared polling query", "[comm][python-phase5]") {
    using namespace std::chrono_literals;
    auto config = config_for(Model::kO30);
    config.runtime_behavior = RuntimeBehavior::kPython;
    config.o30_layout = O30Layout::kPythonV25;
    config.side = HandSide::kLeft;
    config.identity_timeout = 300ms;
    config.response_timeout = 300ms;
    std::shared_ptr<o30_comm::HopEndpoint> hop;
    HandDevice device(config, [&] {
        auto t = std::make_unique<FakeTransport>();
        hop = std::make_shared<o30_comm::HopEndpoint>();
        hop->request_id = 2;
        install_fake_device(Model::kO30, t.get(), hop);
        hop->put_ascii(0x41, 0x61, 8, "0.0.4");
        std::vector<uint8_t> metadata(0x73, 0);
        const std::string type = "TEST", descriptor = "FnS_U16_1;Fn_U8_2";
        std::copy(type.begin(), type.end(), metadata.begin());
        metadata[0x20] = 'g';
        metadata[0x29] = 1;
        metadata[0x2A] = 2;
        metadata[0x2B] = static_cast<uint8_t>(descriptor.size());
        std::copy(descriptor.begin(), descriptor.end(), metadata.begin() + 0x2C);
        metadata[0x6C] = 4;
        metadata[0x6E] = 1;
        metadata[0x6F] = 1;
        metadata[0x70] = 2;
        hop->put(0x31, metadata);
        hop->put(0x32, {0, 1, 0, 9});
        return t;
    });
    device.open();
    const auto baseline = device.get_health().transaction_latency.count;
    PollingChannel force{TelemetrySemantic::kForce, 1s};
    force.force_fields = {{"fn", 1s}, {"fns", 2s}};
    device.start_polling({force});
    REQUIRE(wait_until([&] {
        const auto health = device.get_health();
        return std::any_of(health.channels.begin(), health.channels.end(),
                           [](const auto &c) { return c.channel == "force" && c.completed_reads == 1; });
    }));
    const auto health = device.get_health();
    REQUIRE(health.transaction_latency.count == baseline + 1);
    for (const auto &field : health.force_fields) {
        if (field.field == "fn" || field.field == "fns") {
            REQUIRE(field.supported == true);
            REQUIRE(field.query_channel == TelemetrySemantic::kForce);
            REQUIRE(field.requested_hz == (field.field == "fn" ? 1.0 : 0.5));
            REQUIRE(field.query_hz == 1.0);
            REQUIRE(field.achieved_hz > 0);
        }
    }
    device.stop_polling();
    device.close();
}

TEST_CASE("Python L20 non-angle writes release reads without optional replies", "[comm][python-phase5][l20-dispatch]") {
    using namespace std::chrono_literals;
    for (const auto semantic : {CommandSemantic::kSpeeds, CommandSemantic::kCurrentLimits}) {
        CAPTURE(semantic);
        auto config = config_for(Model::kL20);
        config.runtime_behavior = RuntimeBehavior::kPython;
        config.identity_timeout = 300ms;
        config.response_timeout = 5s;
        FakeTransport *transport = nullptr;
        std::shared_ptr<o30_comm::HopEndpoint> hop;
        HandDevice device(config, [&] {
            auto next = std::make_unique<FakeTransport>();
            transport = next.get();
            install_fake_device(Model::kL20, transport, hop);
            return next;
        });
        device.open();
        const auto before = device.get_health().transaction_latency.count;
        transport->on_request(0x28, [](const Message &request) {
            return request.dlc > 1 ? std::vector<Message>{} : classic_angle_reply(Model::kL20, request);
        });
        const std::vector<double> values(device.get_capabilities().joints.size(), 10);
        auto command =
            semantic == CommandSemantic::kSpeeds ? device.set_speeds(values) : device.set_current_limits(values);
        completed(command);
        // The caller budget is shorter than the old optional reply window.
        // A same-key read must succeed without waiting for that window.
        if (semantic == CommandSemantic::kSpeeds) {
            REQUIRE(device.get_speeds(750ms).meta.valid);
        } else {
            REQUIRE(device.get_currents(750ms).meta.valid);
        }
        const auto health = device.get_health();
        REQUIRE(health.transaction_latency.count == before + 2);
        REQUIRE(health.consecutive_errors == 0);
        REQUIRE_FALSE(health.recent_error);
        REQUIRE(command.stage() == CommandStage::kDispatched);
        REQUIRE_FALSE(command.error());
        device.close();
    }
}

namespace {
    HandDeviceConfig python_l20_dispatch_config() {
        auto config = config_for(Model::kL20);
        config.runtime_behavior = RuntimeBehavior::kPython;
        config.identity_timeout = std::chrono::milliseconds{750};
        config.response_timeout = std::chrono::seconds{5};
        return config;
    }

    void install_l20_dispatch_only(FakeTransport *transport, uint32_t address = 0x28) {
        transport->on_request(address, [](const Message &request) {
            if (request.dlc > 1) {
                return std::vector<Message>{};
            }
            return classic_angle_reply(Model::kL20, request);
        });
    }

    void verify_l20_write_read(HandDevice &device) {
        const std::vector<double> values(16, 10);
        completed(device.set_speeds(values));
        REQUIRE(device.get_speeds(std::chrono::milliseconds{750}).meta.valid);
        completed(device.set_current_limits(values));
        REQUIRE(device.get_currents(std::chrono::milliseconds{750}).raw == std::vector<double>(16, 64));
        completed(device.clear_faults());
        REQUIRE(device.get_faults(std::chrono::milliseconds{750}).meta.valid);
        REQUIRE(device.get_health().consecutive_errors == 0);
        REQUIRE_FALSE(device.get_health().recent_error);
    }
}

TEST_CASE("Python L20 repeated dispatch-only controls do not hold read keys", "[comm][python-phase5][l20-dispatch]") {
    FakeTransport *transport = nullptr;
    HandDevice device(python_l20_dispatch_config(), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL20);
        return next;
    });
    device.open();
    install_l20_dispatch_only(transport);
    const auto baseline = device.get_health().transaction_latency.count;
    for (int round = 0; round < 3; ++round) {
        verify_l20_write_read(device);
    }
    REQUIRE(device.get_health().transaction_latency.count == baseline + 18);
    device.close();
}

TEST_CASE("Python L20 idle late write replies do not publish or complete future reads",
          "[comm][python-phase5][l20-dispatch]") {
    using namespace std::chrono_literals;
    FakeTransport *transport = nullptr;
    HandDevice device(python_l20_dispatch_config(), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL20);
        return next;
    });
    device.open();
    install_l20_dispatch_only(transport);
    const auto before = device.get_snapshot();
    const auto baseline = device.get_health().transaction_latency.count;
    completed(device.set_speeds(std::vector<double>(16, 10)));
    completed(device.set_current_limits(std::vector<double>(16, 10)));
    // Physical dispatch may precede logical metric finalization. Observe both
    // writes before measuring whether unsolicited frames create extra samples.
    REQUIRE(wait_until([&] { return device.get_health().transaction_latency.count == baseline + 2; }));
    const auto latency = device.get_health().transaction_latency.count;
    const auto sent = transport->sent();
    for (const auto &request : sent) {
        if (request.dlc > 1) {
            for (const auto &reply : classic_angle_reply(Model::kL20, request)) {
                transport->inject(reply);
            }
            auto malformed = request;
            malformed.dlc = 2;
            transport->inject(malformed);
        }
    }
    // Complete an unrelated angle query behind the injected backlog. It is a
    // receive-worker barrier, rather than a sleep used to guess processing time.
    REQUIRE(device.get_angles(750ms).meta.valid);
    const auto after = device.get_snapshot();
    REQUIRE(after.currents.has_value() == before.currents.has_value());
    if (before.currents) {
        REQUIRE(after.currents->meta.sequence == before.currents->meta.sequence);
    }
    REQUIRE(after.measured_speeds.has_value() == before.measured_speeds.has_value());
    REQUIRE(wait_until([&] { return device.get_health().transaction_latency.count == latency + 1; }));
    REQUIRE(device.get_speeds(750ms).raw != std::vector<double>(16, 10));
    REQUIRE(device.get_currents(750ms).raw == std::vector<double>(16, 64));
    REQUIRE(device.get_health().consecutive_errors == 0);
    REQUIRE_FALSE(device.get_health().recent_error);
    device.close();
}

TEST_CASE("Python L20 dispatch-only writes leave no reply state across close reopen and recovery",
          "[comm][python-phase5][l20-dispatch]") {
    using namespace std::chrono_literals;
    FakeTransport *transport = nullptr;
    HandDevice device(python_l20_dispatch_config(), [&] {
        auto next = std::make_unique<FakeTransport>();
        transport = next.get();
        install_classic_identity(transport, Model::kL20);
        return next;
    });
    device.open();
    install_l20_dispatch_only(transport);
    completed(device.set_speeds(std::vector<double>(16, 10)));
    device.close();
    REQUIRE_THROWS_AS(device.get_speeds(750ms), DeviceNotReadyError);
    device.open();
    install_l20_dispatch_only(transport);
    verify_l20_write_read(device);
    completed(device.set_current_limits(std::vector<double>(16, 10)));
    // Recovery performs identity and baseline reads, so restore the device's
    // normal identity responder before entering the recovery state machine.
    install_classic_identity(transport, Model::kL20);
    device.recover();
    install_l20_dispatch_only(transport);
    verify_l20_write_read(device);
    device.close();
}

TEST_CASE("Python L20 dispatch-only state remains device-local on a shared Session",
          "[comm][python-phase5][l20-dispatch]") {
    using namespace std::chrono_literals;
    auto transport = std::make_unique<FakeTransport>();
    auto *raw = transport.get();
    install_classic_identity(raw, Model::kL20, "LHT20-1-123-L", 0x28);
    install_classic_identity(raw, Model::kL20, "LHT20-1-456-R", 0x27);
    auto session = linkerbot::transport::Session::own(std::move(transport));
    auto left_config = python_l20_dispatch_config();
    auto right_config = left_config;
    right_config.side = HandSide::kRight;
    HandDevice left(left_config, session);
    HandDevice right(right_config, session);
    left.open();
    right.open();
    install_l20_dispatch_only(raw, 0x28);
    install_l20_dispatch_only(raw, 0x27);
    completed(left.set_speeds(std::vector<double>(16, 10)));
    const auto right_before = right.get_health().transaction_latency.count;
    REQUIRE(right.get_speeds(750ms).meta.valid);
    REQUIRE(right.get_health().transaction_latency.count == right_before + 1);
    REQUIRE(left.get_speeds(750ms).meta.valid);
    completed(left.set_current_limits(std::vector<double>(16, 10)));
    left.close();
    REQUIRE(session->health().open);
    verify_l20_write_read(right);
    install_classic_identity(raw, Model::kL20, "LHT20-1-123-L", 0x28);
    left.open();
    install_l20_dispatch_only(raw, 0x28);
    verify_l20_write_read(left);
    verify_l20_write_read(right);
    left.close();
    right.close();
    session->close();
}

TEST_CASE("Python L20 queued reads reject write replies received before physical claim",
          "[comm][python-phase5][l20-dispatch]") {
    using namespace std::chrono_literals;
    auto transport = std::make_unique<FakeTransport>();
    auto *raw = transport.get();
    install_classic_identity(raw, Model::kL20);
    auto session = linkerbot::transport::Session::own(std::move(transport));
    HandDevice device(python_l20_dispatch_config(), session);
    device.open();
    install_l20_dispatch_only(raw);
    const auto expected = device.get_speeds(750ms).raw;
    completed(device.set_speeds(std::vector<double>(16, 10)));
    const auto sent = raw->sent();

    Message unrelated{};
    unrelated.id = 0x123;
    unrelated.dlc = 1;
    linkerbot::transport::MessageBatch blocker{{unrelated}};
    blocker.initial_delay = 800ms;
    auto receipt = session->submit(std::move(blocker));
    REQUIRE(wait_until([&] { return receipt.result().claimed_at.has_value(); }));
    auto read = std::async(std::launch::async, [&] { return device.get_speeds(2s); });
    REQUIRE(wait_until([&] { return session->health().pending_fifo > 0; }));
    for (const auto &request : sent) {
        if (request.dlc > 1 && request.data[0] >= 0x49 && request.data[0] <= 0x4D) {
            for (const auto &reply : classic_angle_reply(Model::kL20, request)) {
                raw->inject(reply);
            }
        }
    }
    REQUIRE(read.get().raw == expected);
    REQUIRE(receipt.completed());
    REQUIRE(device.get_health().consecutive_errors == 0);
    device.close();
    session->close();
}
