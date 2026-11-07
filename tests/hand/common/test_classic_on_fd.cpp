#include <catch2/catch_test_macros.hpp>
#include <linkerbot/can/frame.hpp>
#include <linkerbot/hand/l20/protocol_backend.hpp>
#include <linkerbot/hand/l20lite/protocol_backend.hpp>
#include <linkerbot/hand/l6/protocol_backend.hpp>
#include <linkerbot/hand/o6/protocol_backend.hpp>

TEST_CASE("Classic hand plans stay classic on an FD-capable Session", "[comm][python-migration][frame-format]") {
    using namespace linkerbot;
    using namespace hand;
    const std::vector<protocol::TransactionPlan> plans{
        l6::protocol::read_serial(0x28),     l6::protocol::read_telemetry(0x28, l6::kTorqueOpcode),
        o6::protocol::read_angle(0x28),      o6::protocol::read_current(0x28),
        l20lite::protocol::read_speed(0x28), l20lite::protocol::read_acceleration(0x28),
        l20::protocol::read_version_sn(0x28)};
    for (const auto &plan : plans) {
        REQUIRE_FALSE(plan.frames.empty());
        for (const auto &frame : plan.frames) {
            REQUIRE((frame.format == can::FrameFormat::kClassic));
            REQUIRE_FALSE(can::resolve_tx_flags(can::LinkKind::kCanFd, frame).is_fd);
            REQUIRE_FALSE(can::resolve_tx_flags(can::LinkKind::kClassic, frame).is_fd);
        }
    }
}
