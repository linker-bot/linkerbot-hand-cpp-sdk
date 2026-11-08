#include "../fake_classic_hand.hpp"
#include <can_transport.hpp> // exported compatibility include path
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <future>
#include <iostream>
#include <linkerbot/can/fake_transport.hpp>
#include <linkerbot/hand/adapter_transport.hpp>
#include <linkerbot/hand/discovery.hpp>
#include <linkerbot/hand/l30/l30.hpp>
#include <linkerbot/hand/l6/l6.hpp>
#include <linkerbot/hand/o30/protocol.hpp>
#include <linkerbot/transport/session.hpp>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
    using linkerbot::hand::CommandHandle;
    using linkerbot::hand::HandDevice;

    // Keep the installed-package consumer compiling against every unified
    // setter.  Runtime admission is exercised by the FakeTransport checks
    // below; these assertions protect the public async return contract.
    static_assert(
        std::is_same_v<decltype(std::declval<HandDevice &>().set_angles(std::declval<const std::vector<int32_t> &>())),
                       CommandHandle>);
    static_assert(std::is_same_v<
                  decltype(std::declval<HandDevice &>().set_percent_angles(std::declval<const std::vector<float> &>())),
                  CommandHandle>);
#define LINKERBOT_ASSERT_SETTER(name)                                                                                  \
    static_assert(                                                                                                     \
        std::is_same_v<decltype(std::declval<HandDevice &>().name(std::declval<const std::vector<double> &>())),       \
                       CommandHandle>)
    LINKERBOT_ASSERT_SETTER(set_position_kp);
    LINKERBOT_ASSERT_SETTER(set_position_kd);
    LINKERBOT_ASSERT_SETTER(set_torque_kd);
    LINKERBOT_ASSERT_SETTER(set_torques);
    LINKERBOT_ASSERT_SETTER(set_limit_compensations);
    LINKERBOT_ASSERT_SETTER(set_torques_limits);
    LINKERBOT_ASSERT_SETTER(set_motion_times);
    LINKERBOT_ASSERT_SETTER(set_speeds);
    LINKERBOT_ASSERT_SETTER(set_accelerations);
    LINKERBOT_ASSERT_SETTER(set_current_limits);
    LINKERBOT_ASSERT_SETTER(set_stall_detection_times);
    LINKERBOT_ASSERT_SETTER(set_stall_thresholds);
    LINKERBOT_ASSERT_SETTER(set_stall_hold_currents);
#undef LINKERBOT_ASSERT_SETTER

    static_assert(std::is_same_v<decltype(std::declval<const CommandHandle &>().send_results()),
                                 std::vector<linkerbot::transport::SendResult>>);

    static_assert(std::is_same_v<decltype(&HandDevice::wait_for_snapshot),
                                 linkerbot::hand::HandSnapshot (HandDevice::*)(const linkerbot::hand::HandSnapshot &,
                                                                               std::chrono::milliseconds) const>);

    void verify_python_runtime_contract() {
        using namespace std::chrono_literals;
        auto config = validation::config();
        config.runtime_behavior = linkerbot::hand::RuntimeBehavior::kPython;
        config.command_timeout = 5ms;
        config.response_timeout = 50ms;
        config.route_queue_capacity = 17;
        validation::FakeHand device(config);
        auto claimed = device.block();
        validation::require(!claimed.wait_for(20ms), "profile command expired during a claimed send");
        device.transport->release_tx_gate();
        validation::complete(claimed);
        device.hand.recover_for(2s);
        validation::require(device.hand.is_open(), "profile recovery failed in installed consumer");
        const auto before = device.hand.get_cached_snapshot();
        validation::require(device.hand.get_health().connection_generation == before.connection_generation,
                            "installed Python health generation differs from cache");
        const auto refreshed = device.hand.refresh_device_info(1s);
        validation::require(refreshed.meta.valid && refreshed.meta.sequence > 0, "installed identity metadata missing");
        const auto changed = device.hand.wait_for_cached_snapshot(before, 100ms, false);
        validation::require(changed.device_info && changed.device_info->meta.valid,
                            "installed filtered cache wait failed");
        device.hand.start_polling({{linkerbot::hand::TelemetrySemantic::kAngles, 20ms,
                                    linkerbot::hand::AngleFeedbackPolicy::kCommandFeedbackOnly}});
        device.hand.stop_polling();
    }

    void verify_unified_contract() {
        validation::FakeHand device;
        const auto handle = device.block();
        auto cached = std::async(std::launch::async, [&] {
            (void)device.hand.get_snapshot();
            (void)device.hand.get_health();
        });
        const bool available = cached.wait_for(std::chrono::milliseconds{500}) == std::future_status::ready;
        const bool timed_out = !handle.wait_for(std::chrono::milliseconds{1});
        device.transport->release_tx_gate();
        cached.get();
        validation::require(available && timed_out, "installed setter/cache blocked on transport");
        validation::complete(handle);
        validation::require(handle.stage() == linkerbot::hand::CommandStage::kDispatched,
                            "installed no-ACK command has wrong terminal stage");
        validation::complete(device.hand.set_stall_thresholds(std::vector<double>(6, 10)));
        validation::complete(device.hand.submit_operation(linkerbot::hand::DeviceOperation::kClearFaults));
        validation::complete(device.hand.set_position_kp(std::vector<double>(6, 50)));
        validation::require(device.hand.get_snapshot().position_kp.has_value(), "installed gain snapshot missing");
        device.transport->on_request(0x28, [](const auto &request) {
            if (request.data[0] == 0xCF) {
                auto reply = request;
                reply.dlc = 2;
                reply.data[1] = 1;
                return std::vector<linkerbot::can::Message>{reply};
            }
            return validation::l6_reply(request);
        });
        const auto saved = device.hand.save_parameters_confirmed();
        validation::complete(saved);
        validation::require(saved.stage() == linkerbot::hand::CommandStage::kAcknowledged &&
                                saved.send_results().size() == 1,
                            "installed confirmed-save contract failed");

        const auto readable = device.hand.get_cached_snapshot();
        validation::require(readable.angles && readable.angles->meta.valid, "installed readable cache is invalid");
        const auto refreshed = device.hand.refresh_device_info();
        validation::require(refreshed.model == linkerbot::hand::Model::kL6 &&
                                device.hand.get_cached_snapshot().connection_generation ==
                                    readable.connection_generation,
                            "installed refresh changed the cache generation");
        const auto before = device.transport->sent().size();
        bool rejected = false;
        try {
            (void)device.hand.set_angles({10});
        } catch (const linkerbot::hand::ValidationError &) {
            rejected = true;
        }
        validation::require(rejected && device.transport->sent().size() == before,
                            "installed admission error was not synchronous and zero-I/O");
        device.transport->fail_writes("installed consumer send failure");
        const auto failed = device.hand.set_angles(std::vector<int32_t>(6, 10));
        validation::require(failed.wait_for(std::chrono::seconds{2}) &&
                                failed.stage() == linkerbot::hand::CommandStage::kFailed && failed.error(),
                            "installed execution error was not retained on handle");
        bool transport_error = false;
        try {
            std::rethrow_exception(failed.error());
        } catch (const linkerbot::hand::TransportError &) {
            transport_error = true;
        }
        validation::require(transport_error, "installed execution error lost its type");
    }
}

static int run_consumer() {
    try {
        verify_unified_contract();
        verify_python_runtime_contract();
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    linkerbot::hand::DiscoveryConfig discovery_config;
    discovery_config.pcan.library_path = "__linkerbot_missing__/PCANBasic.dll";
    linkerbot::hand::PCANDiscoverySource pcan;
    linkerbot::hand::SLCANDiscoverySource slcan;
    linkerbot::hand::MiniCANFDDiscoverySource minicanfd;
    const auto pcan_report = linkerbot::hand::discover_hands(discovery_config, pcan);
    const auto slcan_report = linkerbot::hand::discover_hands(discovery_config, slcan);
    const bool optional_backends_are_runtime_optional =
        pcan_report.candidates().empty() && pcan_report.failures().size() == 1 &&
        pcan_report.failures().front().code() == "backend_unavailable" && slcan_report.candidates().empty() &&
        slcan_report.failures().size() == 1 && slcan_report.failures().front().code() == "allowlist_required";
#if LINKERBOT_HAND_HAS_SLCAN
    linkerbot::can::SlcanConfig serial;
    serial.port = "COM10";
    const auto serial_factory = linkerbot::hand::make_slcan_transport_factory(serial);
#if LINKERBOT_COMM_SLCAN_FD
    serial.protocol = linkerbot::can::SlcanProtocol::kCanableFd;
    serial.fd = true;
    const auto fd_factory = linkerbot::hand::make_slcan_transport_factory(serial);
    if (!fd_factory) {
        return 3;
    }
#endif
    if (!serial_factory) {
        return 2; // Construct only; installed consumer performs no serial I/O.
    }
#endif
    (void)minicanfd; // Construction must not load the optional vendor library.

    auto transport = std::make_unique<linkerbot::can::FakeTransport>();
    transport->echo_cmd(0x28, 1, 6, 51);
    const auto request = linkerbot::hand::l30::protocol::FrameId{0, 0, 5, 1, 1, 0}.to_int();
    transport->on_request(request, [](const linkerbot::can::Message &m) {
        auto reply = m;
        reply.id = linkerbot::hand::l30::protocol::expected_response_id(m.id);
        reply.dlc = 48;
        reply.data.fill(0);
        reply.data[0] = 34;
        return std::vector<linkerbot::can::Message>{reply};
    });
    auto session = linkerbot::transport::Session::own(std::move(transport));
    linkerbot::hand::l6::L6 hand(linkerbot::hand::l6::Side::kLeft, session, "installed-offline",
                                 std::filesystem::current_path() / "mapping.toml");
    const auto data = hand.angle.get_blocking(std::chrono::seconds(1));
    const auto frame = linkerbot::hand::o30::protocol::build_message(1, 1, 0, 1, {}, false);
    const bool valid = data.device_raw[5] == 51 && frame.id == 1 && hand.runtime()->runtime_health().completed == 1;
    linkerbot::hand::l30::L30 l30(session, 1, 0, false);
    (void)l30.angle.get_blocking(std::chrono::seconds(1));
    const bool l30_valid = l30.angle.get_snapshot().has_value() && l30.runtime_health().completed == 1 &&
                           l30.runtime_health().in_flight == 0;
    l30.stop();
    hand.stop();
    session->close();
    return valid && l30_valid && optional_backends_are_runtime_optional ? 0 : 1;
}

int main() {
    try {
        return run_consumer();
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
