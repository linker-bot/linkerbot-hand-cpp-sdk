#include "../../../src/hand/unified/detail/python_identity.hpp"
#include "../../../src/hand/unified/detail/python_transactions.hpp"
#include <catch2/catch_test_macros.hpp>
#include <linkerbot/hand/l20/protocol_backend.hpp>
#include <linkerbot/hand/l20lite/protocol_backend.hpp>
#include <linkerbot/hand/l6/protocol_backend.hpp>
#include <linkerbot/hand/o6/protocol_backend.hpp>

using namespace linkerbot::hand;
using namespace std::chrono_literals;

TEST_CASE("Python fixed vector and bank response errors are strict only on the matching route and opcode",
          "[comm][python-phase2]") {
    for (const auto model : {Model::kL6, Model::kO6, Model::kL20Lite, Model::kL20}) {
        CAPTURE(model);
        HandDeviceConfig config;
        config.model = model;
        auto plan = model == Model::kL6        ? l6::protocol::angle(0x28, {}, true, 5ms)
                    : model == Model::kO6      ? o6::protocol::read_angle(0x28)
                    : model == Model::kL20Lite ? l20lite::protocol::read_angle(0x28)
                                               : l20::protocol::read_angle(0x28);
        plan = unified_detail::python_transaction(std::move(plan), config);
        auto malformed = plan.frames.front();
        malformed.dlc = 2;
        auto unrelated = malformed;
        unrelated.id = 0x27;
        REQUIRE((plan.consume(unrelated).kind == protocol::ProgressKind::unmatched));
        unrelated = malformed;
        unrelated.data[0] = 0xFE;
        REQUIRE((plan.consume(unrelated).kind == protocol::ProgressKind::unmatched));
        REQUIRE_THROWS_AS(plan.consume(malformed), ProtocolError);
        REQUIRE_FALSE(plan.poison_on_timeout);
        REQUIRE((plan.preserve_last_success));
        REQUIRE((plan.publish_after_send));
    }
}

TEST_CASE("Python protocol windows distinguish reads configuration writes and optional feedback",
          "[comm][python-phase2]") {
    HandDeviceConfig config;
    config.model = Model::kL6;
    auto read = unified_detail::python_transaction(l6::protocol::read_telemetry(0x28, l6::kCurrentLimitOpcode), config);
    REQUIRE((read.response_window == 50ms));
    auto write = l6::protocol::command(0x28, l6::kCurrentLimitOpcode, std::vector<uint8_t>(6, 20));
    write.consume = read.consume;
    auto configured = unified_detail::python_transaction(std::move(write), config);
    REQUIRE((configured.response_window == 100ms));
    auto optional = l6::protocol::angle(0x28, std::array<uint8_t, 6>{}, true, 5ms);
    optional.optional_response_window = 200ms;
    auto feedback = unified_detail::python_transaction(std::move(optional), config);
    REQUIRE_FALSE(feedback.response_window);
    REQUIRE((feedback.optional_response_window == 50ms));
    config.response_timeout = 70ms;
    auto overridden =
        unified_detail::python_transaction(l6::protocol::read_telemetry(0x28, l6::kCurrentLimitOpcode), config);
    REQUIRE((overridden.response_window == 70ms));
}

TEST_CASE("Python serial assembly accepts terminated short codes and rejects conflicting fragments",
          "[comm][python-phase2]") {
    auto consume = unified_detail::python_serial(0x28, Model::kL6);
    linkerbot::can::Message first{};
    first.id = 0x28;
    first.dlc = 8;
    first.data[0] = 0xC0;
    first.data[1] = 0;
    const std::string head = "LHL6-1";
    std::copy(head.begin(), head.end(), first.data.begin() + 2);
    REQUIRE((consume(first).kind == protocol::ProgressKind::pending));
    auto second = first;
    second.dlc = 6;
    second.data[1] = 6;
    const std::string tail = "-1-L";
    std::copy(tail.begin(), tail.end(), second.data.begin() + 2);
    auto result = consume(second);
    REQUIRE((result.kind == protocol::ProgressKind::complete));
    REQUIRE((std::any_cast<std::string>(result.value) == "LHL6-1-1-L"));
    second.data[1] = 1; // The index and byte-offset encodings identify the same row.
    REQUIRE((consume(second).kind == protocol::ProgressKind::complete));
    second.data[2] = 'X';
    REQUIRE_THROWS_AS(consume(second), ProtocolError);
}

TEST_CASE("Python L30 response validation distinguishes protocol errors from device status", "[comm][python-phase2]") {
    HandDeviceConfig config;
    config.model = Model::kL30;
    auto plan = l30::protocol::request_plan(l30::protocol::request_frame(
        1, 0, l30::protocol::kParentQuery, l30::protocol::kSubPosition, l30::protocol::kAccessRead,
        l30::protocol::empty_request_payload(), l30::protocol::kEmptyRequestBytes));
    plan = unified_detail::python_transaction(std::move(plan), config);
    auto message = plan.frames.front();
    message.id = l30::protocol::expected_response_id(message.id);
    message.dlc = 2;
    REQUIRE_THROWS_AS(plan.consume(message), ProtocolError);
    message.dlc = 48;
    message.data[0] = 17;
    message.data[1] = 0;
    message.data[2] = 0;
    REQUIRE_THROWS_AS(plan.consume(message), ProtocolError);
    message.data[0] = 34;
    message.data[2] = 1;
    REQUIRE_THROWS_AS(plan.consume(message), DeviceFaultError);
    message.data[2] = 0;
    REQUIRE((plan.consume(message).kind == protocol::ProgressKind::complete));
    auto tactile = unified_detail::python_transaction(l30::protocol::tactile_plan(1, 0, 1), config);
    REQUIRE((tactile.response_window == 50ms));
    REQUIRE((tactile.late_guard == 20ms));
}

TEST_CASE("Python L20 non-angle controls release native reply reservations", "[comm][python-phase5][l20-dispatch]") {
    HandDeviceConfig config;
    config.model = Model::kL20;
    config.response_timeout = 5s;
    l20::protocol::Backend backend(0x28);
    for (const auto *operation : {"speed", "current_limit", "fault"}) {
        CAPTURE(operation);
        auto native = backend.make_command(operation, std::array<uint8_t, l20::kJointCount>{});
        REQUIRE(native.consume);
        REQUIRE(native.optional_response_window == 200ms);
        REQUIRE(native.supersede_optional_feedback);
        const auto profile = unified_detail::python_transaction(native, config);
        REQUIRE_FALSE(profile.consume);
        REQUIRE(profile.keys.empty());
        REQUIRE_FALSE(profile.response_window);
        REQUIRE_FALSE(profile.optional_response_window);
        REQUIRE_FALSE(profile.supersede_optional_feedback);
        REQUIRE(profile.frames.size() == native.frames.size());
        REQUIRE(profile.gap == native.gap);
        REQUIRE(profile.command_family == native.command_family);
    }
    auto angle = unified_detail::python_transaction(
        backend.make_command("angle", std::array<uint8_t, l20::kJointCount>{}), config);
    REQUIRE(angle.consume);
    REQUIRE(angle.optional_response_window == 5s);
    REQUIRE(angle.supersede_optional_feedback);
    auto read = unified_detail::python_transaction(backend.make_read("speed"), config);
    REQUIRE(read.consume);
    REQUIRE(read.response_window == 5s);
}
