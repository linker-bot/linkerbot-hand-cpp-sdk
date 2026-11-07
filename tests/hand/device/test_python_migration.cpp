#include "../../../validation/fake_classic_hand.hpp"
#include "hand/l30/comm_helpers.hpp"
#include "hand/o30/comm_helpers.hpp"
#include "hand/support/optional_value.hpp"
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <future>
#include <linkerbot/hand/l30/l30.hpp>
#include <linkerbot/hand/o30/v25.hpp>
#include <numeric>

using namespace linkerbot::hand;
using namespace std::chrono_literals;
namespace {
    void acknowledged(const CommandHandle &handle) {
        REQUIRE((handle.wait_for(2s)));
        if (handle.error()) {
            std::rethrow_exception(handle.error());
        }
        REQUIRE((handle.stage() == CommandStage::kAcknowledged));
    }
}
TEST_CASE("L6 unified controls preserve response bytes instead of target echoes", "[comm][python-migration]") {
    validation::FakeHand device;
    device.transport->on_request(0x28, [](const auto &request) {
        auto replies = validation::l6_reply(request);
        if (request.data[0] == 0x06 || request.data[0] == 0x07 || request.data[0] == 0x08 || request.data[0] == 0x38 ||
            request.data[0] == 0x03) {
            replies.front().dlc = 7;
            std::fill_n(replies.front().data.begin() + 1, 6, uint8_t{213});
        }
        return replies;
    });
    const std::vector<double> target(6, 42);
    acknowledged(device.hand.set_position_kp(target));
    acknowledged(device.hand.set_position_kd(target));
    acknowledged(device.hand.set_torque_kd(target));
    acknowledged(device.hand.set_limit_compensations(target));
    acknowledged(device.hand.set_torques(target));
    const auto snapshot = device.hand.get_snapshot();
    REQUIRE((test_support::require_value(snapshot.position_kp).raw == std::vector<double>(6, 213)));
    REQUIRE_FALSE(test_support::require_value(snapshot.position_kp).meta.within_declared_range);
    REQUIRE((test_support::require_value(snapshot.position_kp).meta.delivery == TelemetryDelivery::kCommandResponse));
    REQUIRE((test_support::require_value(snapshot.position_kd).raw == std::vector<double>(6, 213)));
    REQUIRE((test_support::require_value(snapshot.torque_kd).raw == std::vector<double>(6, 213)));
    REQUIRE((test_support::require_value(snapshot.limit_compensations).raw == std::vector<double>(6, 213)));
    REQUIRE((test_support::require_value(snapshot.torques).raw == std::vector<double>(6, 213)));
    REQUIRE((snapshot.joints.front().position_kp_raw == 213));
    REQUIRE((device.hand.get_position_kp().raw == std::vector<double>(6, 213)));
    const auto sent = device.transport->sent().size();
    REQUIRE_THROWS_AS(device.hand.set_position_kp(std::vector<double>(6, 201)), ValidationError);
    REQUIRE_THROWS_AS(device.hand.set_torque_kd({1, 2}), ValidationError);
    REQUIRE((device.transport->sent().size() == sent));
}
TEST_CASE("confirmed save requires CF 01 and preserves legacy send-only save", "[comm][python-migration]") {
    for (const auto reply_kind : {0, 1, 2}) {
        CAPTURE(reply_kind);
        validation::FakeHand device;
        device.transport->on_request(0x28, [reply_kind](const auto &request) {
            if (request.data[0] != 0xCF) {
                return validation::l6_reply(request);
            }
            if (reply_kind == 0) {
                return std::vector<linkerbot::can::Message>{};
            }
            auto reply = request;
            reply.dlc = 2;
            reply.data[1] = reply_kind == 1 ? 1 : 0;
            return std::vector<linkerbot::can::Message>{reply};
        });
        device.transport->clear_sent();
        auto handle = device.hand.save_parameters_confirmed(50ms);
        REQUIRE((handle.wait_for(1s)));
        REQUIRE((handle.sent_frames() == 1));
        REQUIRE((handle.send_results().size() == 1));
        const auto sent = device.transport->sent();
        REQUIRE((sent.size() == 1));
        REQUIRE((sent.front().dlc == 8));
        REQUIRE((std::all_of(sent.front().data.begin(), sent.front().data.begin() + 8,
                             [](auto byte) { return byte == 0xCF; })));
        if (reply_kind == 1) {
            REQUIRE((handle.stage() == CommandStage::kAcknowledged));
        } else {
            REQUIRE((handle.stage() == CommandStage::kFailed));
            if (reply_kind == 0) {
                REQUIRE_THROWS_AS(std::rethrow_exception(handle.error()), TimeoutError);
            } else {
                REQUIRE_THROWS_AS(std::rethrow_exception(handle.error()), ProtocolError);
            }
        }
        REQUIRE((device.hand.save_parameters().stage() == CommandStage::kDispatched));
    }
}
TEST_CASE("L30 calibration stops after the first failed ACK", "[comm][python-migration]") {
    namespace lp = linkerbot::hand::l30::protocol;
    for (int fail_at : {-1, 0, 1, 2}) {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        auto *raw = transport.get();
        const std::array<uint32_t, 3> ids{
            l30_comm::write_id(lp::kSubDisable),
            l30_comm::request_id(lp::kParentConfig, lp::kSubConfigUnlock, lp::kAccessWrite),
            l30_comm::request_id(lp::kParentConfig, 0x05, lp::kAccessWrite)};
        for (size_t i = 0; i < ids.size(); ++i) {
            raw->on_request(ids[i], l30_comm::ack_handler(static_cast<int>(i) == fail_at ? 0x23 : 0));
        }
        l30::L30 hand(std::move(transport), 1, 0, false);
        raw->clear_sent();
        if (fail_at < 0) {
            REQUIRE_NOTHROW(hand.control.calibrate(300ms));
        } else {
            REQUIRE_THROWS_AS(hand.control.calibrate(300ms), lp::ProtocolError);
        }
        const auto sent = raw->sent();
        REQUIRE((sent.size() == static_cast<size_t>(fail_at < 0 ? 3 : fail_at + 1)));
        for (size_t i = 0; i < sent.size(); ++i) {
            REQUIRE((sent[i].id == ids[i]));
        }
        if (sent.size() > 1) {
            REQUIRE((sent[1].dlc == 12));
            REQUIRE((std::vector<uint8_t>(sent[1].data.begin(), sent[1].data.begin() + 8) ==
                     std::vector<uint8_t>{6, 0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}));
        }
    }
}
TEST_CASE("O30 V25 is explicit and uses compact whole-hand objects", "[comm][python-migration]") {
    HandDeviceConfig config;
    config.model = Model::kO30;
    config.side = HandSide::kLeft;
    config.o30_layout = O30Layout::kPythonV25;
    config.address = 1;
    REQUIRE_FALSE(declared_capabilities(Model::kO30).supports(CommandSemantic::kTorqueLimits));
    REQUIRE((declared_capabilities(config).supports(CommandSemantic::kTorqueLimits)));
    REQUIRE((declared_capabilities(config).supports(CommandSemantic::kMotionTimes)));
    auto hop = std::make_shared<o30_comm::HopEndpoint>();
    hop->seed_defaults();
    hop->put_ascii(o30::protocol::kMiProductInfo, 0xD9, 8, "left");
    linkerbot::can::FakeTransport *raw = nullptr;
    HandDevice device(config, [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        raw = transport.get();
        raw->on_request(1, [hop](const auto &request) {
            if (request.data[1] == 0 && request.data[2] == 25) {
                auto reply = request;
                reply.id = 0x401;
                reply.dlc = 32;
                if (request.dlc <= 3) {
                    std::fill_n(reply.data.begin() + 3, 25, uint8_t{27});
                } else if (request.data[0] == 1) {
                    std::fill_n(reply.data.begin() + 3, 25, uint8_t{31});
                }
                return std::vector<linkerbot::can::Message>{reply};
            }
            return hop->reply(request);
        });
        return transport;
    });
    device.open();
    const auto health = device.get_health();
    const auto angle_metrics = std::find_if(health.channels.begin(), health.channels.end(),
                                            [](const auto &channel) { return channel.channel == "angles"; });
    REQUIRE((angle_metrics != health.channels.end()));
    REQUIRE((angle_metrics->data_age.valid));
    raw->clear_sent();
    std::vector<double> values(20);
    std::iota(values.begin(), values.end(), 1);
    acknowledged(device.set_torques_limits(values));
    acknowledged(device.set_motion_times(values));
    const auto sent = raw->sent();
    REQUIRE((sent.size() == 2));
    REQUIRE((sent[0].data[0] == 6));
    REQUIRE((sent[1].data[0] == 8));
    for (const auto &frame : sent) {
        REQUIRE((frame.is_fd));
        REQUIRE_FALSE(frame.is_extended);
        REQUIRE((frame.data[1] == 0));
        REQUIRE((frame.data[2] == 25));
        REQUIRE((std::vector<uint8_t>(frame.data.begin() + 3, frame.data.begin() + 28) == o30::v25::encode(values)));
    }
    acknowledged(device.set_angles(std::vector<int32_t>(20, 5)));
    const auto angles = test_support::require_value(device.get_snapshot().angles);
    REQUIRE((angles.raw == std::vector<double>(20, 31)));
    REQUIRE((angles.meta.delivery == TelemetryDelivery::kCommandResponse));
    REQUIRE_FALSE(device.get_snapshot().torque_limits);
    REQUIRE((device.get_torques_limits().raw == std::vector<double>(20, 27)));
    REQUIRE((device.get_motion_times().raw == std::vector<double>(20, 27)));
    const auto snapshot = device.get_snapshot();
    REQUIRE(
        (test_support::require_value(snapshot.torque_limits).meta.origin == TelemetryValueOrigin::kDeviceConfigured));
    REQUIRE((snapshot.joints.front().torque_limits_raw == 27));
    device.close();
}
TEST_CASE("V25 rejects short ACKs and invalid layout without I/O", "[comm][python-migration]") {
    auto plan = o30::v25::transaction(1, o30::v25::object(CommandSemantic::kTorqueLimits), std::vector<uint8_t>(25, 4));
    auto short_reply = o30::protocol::build_message(1, 6, 0, 1, {1}, false);
    short_reply.id = 0x401;
    REQUIRE((plan.consume(short_reply).kind == protocol::ProgressKind::pending));
    auto remainder = o30::protocol::build_message(1, 6, 1, 24, std::vector<uint8_t>(24, 9), false);
    remainder.id = 0x401;
    REQUIRE((plan.consume(remainder).kind == protocol::ProgressKind::complete));
    REQUIRE_THROWS_AS(o30::v25::encode(std::vector<double>(20, 256)), ValidationError);
    HandDeviceConfig config;
    config.model = Model::kL6;
    config.side = HandSide::kLeft;
    config.o30_layout = O30Layout::kPythonV25;
    bool created = false;
    REQUIRE_THROWS_AS(HandDevice(config,
                                 [&] {
                                     created = true;
                                     return std::make_unique<linkerbot::can::FakeTransport>();
                                 }),
                      ValidationError);
    REQUIRE_FALSE(created);
}

TEST_CASE("L6 torque zero transition remains FIFO and keeps a bounded guard", "[comm][python-migration]") {
    validation::FakeHand device;
    auto blocking = device.block();
    auto zero = device.hand.set_torques(std::vector<double>(6, 0), 1s);
    auto nonzero = device.hand.set_torques(std::vector<double>(6, 1), 1s);
    REQUIRE_FALSE(zero.done());
    REQUIRE_FALSE(nonzero.done());
    device.transport->release_tx_gate();
    validation::complete(blocking);
    acknowledged(zero);
    acknowledged(nonzero);
    const auto receipts = zero.send_results();
    REQUIRE((receipts.size() == 1));
    REQUIRE((std::chrono::steady_clock::now() - test_support::require_value(receipts.front().completed_at) >= 150ms));
    REQUIRE((zero.family() == CommandFamily::kTorqueTarget));
    REQUIRE((zero.stage() != CommandStage::kSuperseded));
}

TEST_CASE("explicit empty polling plan can disable without changing native default", "[comm][python-migration]") {
    auto config = validation::config();
    config.empty_polling_plan_disables = true;
    validation::FakeHand device(config);
    device.hand.start_polling();
    device.hand.start_polling({});
    REQUIRE_FALSE(device.hand.get_health().polling_active);
    device.transport->clear_sent();
    REQUIRE_NOTHROW(device.hand.start_polling({}));
    REQUIRE((device.transport->sent().empty()));
}

TEST_CASE("O30 V25 endpoint defaults match Python sides", "[comm][python-migration]") {
    HandDeviceConfig config;
    config.model = Model::kO30;
    config.o30_layout = O30Layout::kPythonV25;
    config.side = HandSide::kLeft;
    REQUIRE((declared_capabilities(config).transport.request_id == 2));
    REQUIRE((declared_capabilities(config).transport.response_id == 0x402));
    config.side = HandSide::kRight;
    REQUIRE((declared_capabilities(config).transport.request_id == 1));
    config.address = 7;
    REQUIRE((declared_capabilities(config).transport.response_id == 0x407));
    config.side = HandSide::kUnknown;
    REQUIRE_THROWS_AS(declared_capabilities(config), ValidationError);
}

TEST_CASE("O6 confirmed persistence uses the shared acknowledged operation path", "[comm][python-migration]") {
    auto config = validation::config();
    config.model = Model::kO6;
    linkerbot::can::FakeTransport *raw = nullptr;
    HandDevice device(config, [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        raw = transport.get();
        raw->on_request(0x28, [](const auto &request) {
            auto replies = validation::l6_reply(request);
            if (request.data[0] == 0xC0) {
                replies.front().data[4] = 'O';
            }
            if (request.data[0] == 0xC1) {
                replies.front().dlc = 4;
            }
            if (request.data[0] == 0xCF) {
                replies.front().dlc = 2;
                replies.front().data[1] = 1;
            }
            return replies;
        });
        return transport;
    });
    device.open();
    raw->clear_sent();
    auto saved = device.save_parameters_confirmed();
    acknowledged(saved);
    REQUIRE((raw->sent().size() == 1));
    REQUIRE((saved.send_results().size() == 1));
    REQUIRE((saved.family() == CommandFamily::kDiscreteOperation));
    device.close();
}

TEST_CASE("new persistence operations use the existing bounded FIFO", "[comm][python-migration]") {
    validation::FakeHand device(validation::config(1));
    auto blocking = device.block();
    auto pending = device.hand.save_parameters_confirmed(2s);
    REQUIRE_FALSE(pending.done());
    REQUIRE_THROWS_AS(device.hand.save_parameters_confirmed(2s), QueueFullError);
    REQUIRE((pending.family() == CommandFamily::kDiscreteOperation));
    device.transport->release_tx_gate();
    validation::complete(blocking);
    // The default fake has no CF 01. A failed operation retains its receipt.
    REQUIRE((pending.wait_for(3s)));
    REQUIRE((pending.stage() == CommandStage::kFailed));
    REQUIRE((pending.send_results().size() == 1));
}

TEST_CASE("unified L30 calibration retains all three physical receipts", "[comm][python-migration]") {
    namespace lp = linkerbot::hand::l30::protocol;
    HandDeviceConfig config;
    config.model = Model::kL30;
    linkerbot::can::FakeTransport *raw = nullptr;
    HandDevice device(config, [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        raw = transport.get();
        const std::vector<uint8_t> info{0x13, 0, 0, 0, 0x7B, 1, 2, 3, 2, 3, 4, 3, 4, 5, 1, 0, 'B', 'A'};
        const std::string product = "LHT30-06-123-L-B-3-A";
        raw->on_request(l30_comm::request_id(lp::kParentConfig, lp::kSubDeviceInfo, lp::kAccessRead),
                        l30_comm::payload_handler(info));
        raw->on_request(l30_comm::request_id(lp::kParentConfig, lp::kSubProductCode, lp::kAccessRead),
                        l30_comm::payload_handler(std::vector<uint8_t>(product.begin(), product.end())));
        raw->on_request(l30_comm::query_id(lp::kSubPosition), l30_comm::i16_handler(l30_comm::filled_i16(0)));
        raw->on_request(l30_comm::query_id(lp::kSubFault), l30_comm::u8_handler(l30_comm::filled_u8(0)));
        for (const auto pair : std::array<std::pair<uint8_t, uint8_t>, 3>{{{1, 8}, {3, 1}, {3, 5}}}) {
            raw->on_request(l30_comm::request_id(pair.first, pair.second, lp::kAccessWrite), l30_comm::ack_handler());
        }
        return transport;
    });
    device.open();
    raw->clear_sent();
    auto calibrated = device.calibrate();
    acknowledged(calibrated);
    REQUIRE((calibrated.sent_frames() == 3));
    REQUIRE((calibrated.send_results().size() == 3));
    REQUIRE((raw->sent().size() == 3));
    REQUIRE((raw->sent().back().id == l30_comm::request_id(3, 5, lp::kAccessWrite)));
    device.close();
}

TEST_CASE("two hand endpoints retain one Session and independent close", "[comm][python-migration]") {
    int factories = 0;
    auto session = linkerbot::transport::Session::create([&] {
        ++factories;
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        transport->on_request(0x28, validation::l6_reply);
        transport->on_request(0x27, [](const auto &request) {
            auto replies = validation::l6_reply(request);
            if (request.data[0] == 0xC0) {
                for (auto &reply : replies) {
                    if (reply.data[1] == 1) {
                        reply.data[7] = 'R';
                    }
                }
            }
            return replies;
        });
        return transport;
    });
    auto left_config = validation::config();
    auto right_config = left_config;
    right_config.side = HandSide::kRight;
    HandDevice left(left_config, session);
    HandDevice right(right_config, session);
    left.open();
    right.open();
    auto torque = left.set_torques(std::vector<double>(6, 0), 1s);
    REQUIRE((right.get_angles(1s).meta.valid));
    left.close();
    REQUIRE((right.get_angles(1s).meta.valid));
    REQUIRE((factories == 1));
    REQUIRE((torque.done()));
    right.close();
    session->close();
}

TEST_CASE("generic operation admission shares the setter FIFO without changing legacy methods",
          "[comm][python-migration]") {
    validation::FakeHand device(validation::config(1));
    auto blocking = device.block();
    auto pending = device.hand.submit_operation(DeviceOperation::kClearFaults, 2s);
    REQUIRE_FALSE(pending.done());
    REQUIRE((pending.ack_expectation() == AckExpectation::kNone));
    REQUIRE_THROWS_AS(device.hand.submit_operation(DeviceOperation::kClearFaults, 2s), QueueFullError);
    REQUIRE_THROWS_AS(device.hand.submit_operation(DeviceOperation::kReadDeviceInfo), UnsupportedOperationError);
    // A fixed-underlying enum can represent every uint8_t value in C++17.
    // Exercise rejection of an unknown operation, not an out-of-range integer conversion.
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    const auto unknown_operation = static_cast<DeviceOperation>(255);
    REQUIRE_THROWS_AS(device.hand.submit_operation(unknown_operation), UnsupportedOperationError);
    device.transport->release_tx_gate();
    validation::complete(blocking);
    validation::complete(pending);
    REQUIRE((pending.stage() == CommandStage::kDispatched));
    REQUIRE((pending.send_results().size() == 1));
    REQUIRE((device.hand.clear_faults().stage() == CommandStage::kDispatched));
}

TEST_CASE("torque transition guard survives a malformed ACK", "[comm][python-migration]") {
    validation::FakeHand device;
    std::atomic<int64_t> torque_ns{0}, angle_ns{0};
    device.transport->on_request(0x28, [&](const auto &request) {
        const auto now =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
                .count();
        if (request.data[0] == 3) {
            torque_ns = now;
            auto reply = request;
            reply.dlc = 2;
            return std::vector<linkerbot::can::Message>{reply};
        }
        if (request.data[0] == 1 && request.dlc > 1) {
            angle_ns = now;
        }
        return validation::l6_reply(request);
    });
    auto torque = device.hand.set_torques(std::vector<double>(6, 0), 1s);
    REQUIRE((torque.wait_for(1s)));
    REQUIRE((torque.stage() == CommandStage::kFailed));
    REQUIRE_THROWS_AS(std::rethrow_exception(torque.error()), ProtocolError);
    auto angle = device.hand.set_angles(std::vector<int32_t>(6, 10));
    validation::complete(angle);
    REQUIRE((torque_ns.load() > 0));
    REQUIRE(
        (angle_ns.load() - torque_ns.load() >= std::chrono::duration_cast<std::chrono::nanoseconds>(140ms).count()));
}

namespace {
    class EarlyTorqueAck final : public linkerbot::can::FakeTransport {
    public:
        std::function<void()> after_torque;
        void write(const linkerbot::can::Message &message) override {
            FakeTransport::write(message);
            if (message.data[0] == 3 && after_torque) {
                after_torque();
            }
        }
        void write_batch(const std::vector<linkerbot::can::Message> &messages) override {
            FakeTransport::write_batch(messages);
            for (const auto &message : messages) {
                if (message.data[0] == 3 && after_torque) {
                    after_torque();
                }
            }
        }
    };
}

TEST_CASE("early malformed torque ACK retains the physical completion guard", "[comm][python-migration]") {
    std::promise<void> entered, release;
    const auto gate = release.get_future().share();
    std::atomic<int64_t> angle_ns{0};
    HandDevice device(validation::config(), [&] {
        auto transport = std::make_unique<EarlyTorqueAck>();
        transport->after_torque = [&] {
            entered.set_value();
            gate.wait();
        };
        transport->on_request(0x28, [&](const auto &request) {
            if (request.data[0] == 3) {
                auto reply = request;
                reply.dlc = 2;
                return std::vector<linkerbot::can::Message>{reply};
            }
            if (request.data[0] == 1 && request.dlc > 1) {
                angle_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
            }
            return validation::l6_reply(request);
        });
        return transport;
    });
    struct Releaser {
        std::promise<void> &promise;
        bool released = false;
        void finish() {
            if (!released) {
                promise.set_value();
                released = true;
            }
        }
        ~Releaser() { finish(); }
    } cleanup{release};
    device.open();
    auto torque = device.set_torques(std::vector<double>(6, 0), 2s);
    REQUIRE((entered.get_future().wait_for(2s) == std::future_status::ready));
    REQUIRE((torque.wait_for(1s)));
    REQUIRE_THROWS_AS(std::rethrow_exception(torque.error()), ProtocolError);
    auto angle = device.set_angles(std::vector<int32_t>(6, 10));
    REQUIRE_FALSE(angle.wait_for(10ms));
    const auto released_at =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    cleanup.finish();
    validation::complete(angle);
    REQUIRE((angle_ns.load() - released_at >= std::chrono::duration_cast<std::chrono::nanoseconds>(140ms).count()));
    REQUIRE((torque.send_results().size() == 1));
    REQUIRE((torque.send_results().front().sent_frames == 1));
    REQUIRE((torque.stage() == CommandStage::kFailed));
    device.close();
}

TEST_CASE("added L6 gain reads and writes reject short matching replies", "[comm][python-migration]") {
    validation::FakeHand device;
    acknowledged(device.hand.set_position_kp(std::vector<double>(6, 50)));
    device.transport->on_request(0x28, [](const auto &request) {
        auto replies = validation::l6_reply(request);
        if (request.data[0] == 6) {
            replies.front().dlc = 2;
        }
        return replies;
    });
    auto handle = device.hand.set_position_kp(std::vector<double>(6, 40));
    REQUIRE((handle.wait_for(1s)));
    REQUIRE_THROWS_AS(std::rethrow_exception(handle.error()), ProtocolError);
    REQUIRE((test_support::require_value(device.hand.get_snapshot().position_kp).raw == std::vector<double>(6, 50)));
    REQUIRE_THROWS_AS(device.hand.get_position_kp(), ProtocolError);
}
