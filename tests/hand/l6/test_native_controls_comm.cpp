#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <vector>

#include <linkerbot/can/fake_transport.hpp>
#include <linkerbot/hand/l6/l6.hpp>

#include "hand/support/temp_path.hpp"

using namespace std::chrono_literals;
using linkerbot::can::FakeTransport;
using linkerbot::can::Message;
using linkerbot::hand::l6::L6;
using linkerbot::hand::l6::Side;

namespace {
    using linkerbot::hand::l6::RawControlData;
    using linkerbot::hand::l6::RawControlManager;
    using linkerbot::hand::l6::Snapshot;

    struct Gain {
        uint8_t opcode;
        uint8_t maximum;
        RawControlManager L6::*manager;
        std::optional<RawControlData> Snapshot::*cached;
    };

    constexpr std::array<Gain, 3> gains{{
        {0x06, 200, &L6::position_kp, &Snapshot::position_kp},
        {0x07, 255, &L6::position_kd, &Snapshot::position_kd},
        {0x08, 255, &L6::torque_kd, &Snapshot::torque_kd},
    }};
}

TEST_CASE("comm: L6 native gain managers retain readback and pre-write response evidence",
          "[comm][l6][native-controls]") {
    for (const auto side : {Side::kLeft, Side::kRight}) {
        for (const auto &gain : gains) {
            CAPTURE(side, gain.opcode);
            const uint32_t id = side == Side::kLeft ? 0x28 : 0x27;
            std::array<uint8_t, 6> settings{};
            settings.fill(21);
            auto transport = std::make_unique<FakeTransport>();
            auto *raw = transport.get();
            raw->on_request(id, [&](const Message &request) {
                Message reply{};
                reply.id = id;
                reply.dlc = 7;
                reply.data[0] = gain.opcode;
                std::copy(settings.begin(), settings.end(), reply.data.begin() + 1);
                if (request.dlc == 7) {
                    std::copy_n(request.data.begin() + 1, settings.size(), settings.begin());
                }
                // A matching short response must not finish a transaction.
                auto partial = reply;
                partial.dlc = 2;
                return std::vector<Message>{partial, reply};
            });
            L6 hand(side, std::move(transport), "can0", test_util::unique_mapping_path());
            auto &manager = hand.*gain.manager;
            REQUIRE_FALSE(manager.get_snapshot());
            const auto initial = manager.get_blocking(500ms);
            REQUIRE(initial.values == settings);
            REQUIRE_FALSE(initial.command_response);
            REQUIRE(initial.sequence > 0);

            const std::array<uint8_t, 6> target{0, 1, 41, 127, uint8_t(gain.maximum - 1), gain.maximum};
            manager.set_values(target, 500ms);
            const auto written = manager.get_snapshot();
            REQUIRE(written.has_value());
            REQUIRE(written->values == initial.values);
            REQUIRE(written->command_response);
            REQUIRE(written->sequence > initial.sequence);
            REQUIRE(written->generation > initial.generation);
            const auto queried = manager.get_blocking(500ms);
            REQUIRE(queried.values == target);
            REQUIRE_FALSE(queried.command_response);
            REQUIRE(queried.sequence > written->sequence);
            const auto cached = hand.get_snapshot().*gain.cached;
            REQUIRE(cached.has_value());
            REQUIRE(cached->values == target);
            REQUIRE(cached->sequence == queried.sequence);

            const auto sent = raw->sent();
            REQUIRE(sent.size() == 3);
            for (size_t i = 0; i < sent.size(); ++i) {
                REQUIRE(sent[i].id == id);
                REQUIRE(sent[i].data[0] == gain.opcode);
                REQUIRE(sent[i].dlc == (i == 1 ? 7 : 1));
            }
            REQUIRE(std::equal(target.begin(), target.end(), sent[1].data.begin() + 1));

            // Reads retain firmware evidence even beyond Kp's documented write range.
            settings.fill(255);
            REQUIRE(manager.get_blocking(500ms).values == settings);
            REQUIRE(manager.get_snapshot()->values == settings);
        }
    }
}

TEST_CASE("comm: L6 native position Kp rejects writes above its range before I O",
          "[comm][l6][native-controls]") {
    for (const auto side : {Side::kLeft, Side::kRight}) {
        auto transport = std::make_unique<FakeTransport>();
        auto *raw = transport.get();
        L6 hand(side, std::move(transport), "can0", test_util::unique_mapping_path());
        const auto before = raw->sent().size();
        for (const auto value : {201, 255}) {
            std::array<uint8_t, 6> values{};
            values[3] = static_cast<uint8_t>(value);
            REQUIRE_THROWS_AS(hand.position_kp.set_values(values, 30ms), linkerbot::hand::ValidationError);
        }
        REQUIRE(raw->sent().size() == before);
        REQUIRE_FALSE(hand.position_kp.get_snapshot());
    }
}

TEST_CASE("comm: L6 native gains time out on unrelated incomplete or absent replies",
          "[comm][l6][native-controls]") {
    for (const auto side : {Side::kLeft, Side::kRight}) {
        for (const auto &gain : gains) {
            for (const auto kind : {0, 1, 2, 3, 4}) {
                for (const auto writing : {false, true}) {
                    CAPTURE(side, gain.opcode, kind, writing);
                    const uint32_t id = side == Side::kLeft ? 0x28 : 0x27;
                    auto transport = std::make_unique<FakeTransport>();
                    transport->on_request(id, [=](const Message &request) {
                        if (kind == 4) {
                            return std::vector<Message>{};
                        }
                        auto reply = request;
                        reply.dlc = 7;
                        if (kind == 0) {
                            reply.data[0] = gain.opcode == 0x06 ? 0x07 : 0x06;
                        } else if (kind == 1) {
                            reply.dlc = 6;
                        } else if (kind == 2) {
                            reply.id = id == 0x28 ? 0x27 : 0x28;
                        } else if (kind == 3) {
                            reply.is_fd = true;
                        }
                        return std::vector<Message>{reply};
                    });
                    L6 hand(side, std::move(transport), "can0", test_util::unique_mapping_path());
                    auto &manager = hand.*gain.manager;
                    if (writing) {
                        std::array<uint8_t, 6> target{};
                        target.fill(50);
                        REQUIRE_THROWS_AS(manager.set_values(target, 30ms), linkerbot::hand::TimeoutError);
                    } else {
                        REQUIRE_THROWS_AS(manager.get_blocking(30ms), linkerbot::hand::TimeoutError);
                    }
                    REQUIRE_FALSE(manager.get_snapshot());
                    // A timed-out key stays quarantined instead of accepting an ambiguous late reply.
                    REQUIRE_THROWS_AS(manager.get_blocking(30ms), linkerbot::hand::DeviceNotReadyError);
                    REQUIRE_FALSE(manager.get_snapshot());
                }
            }
        }
    }
}

TEST_CASE("comm: L6 native torque and limit compensation retain read and write support", "[comm][l6][native-controls]") {
    for (const auto side : {Side::kLeft, Side::kRight}) {
        DYNAMIC_SECTION("side " << static_cast<int>(side)) {
            const uint32_t id = side == Side::kLeft ? 0x28 : 0x27;
            std::array<uint8_t, 6> torque{};
            std::array<uint8_t, 6> compensation{};
            auto transport = std::make_unique<FakeTransport>();
            auto *raw = transport.get();
            raw->on_request(id, [&](const Message &request) {
                if (request.data[0] != 0x03 && request.data[0] != 0x38) {
                    return std::vector<Message>{};
                }
                auto &values = request.data[0] == 0x03 ? torque : compensation;
                if (request.dlc == 7) {
                    std::copy_n(request.data.begin() + 1, values.size(), values.begin());
                }
                Message reply{};
                reply.id = id;
                reply.dlc = 7;
                reply.data[0] = request.data[0];
                std::copy(values.begin(), values.end(), reply.data.begin() + 1);
                return std::vector<Message>{reply};
            });
            L6 hand(side, std::move(transport), "can0", test_util::unique_mapping_path());
            const std::array<uint8_t, 6> target{0, 1, 41, 127, 200, 255};

            hand.torque.set_torques(target);
            REQUIRE(hand.torque.get_blocking(500ms).torque.to_array() == target);
            hand.limit_compensation.set_values(target, 500ms);
            REQUIRE(hand.limit_compensation.get_blocking(500ms).values == target);
            REQUIRE(hand.limit_compensation.get_snapshot().has_value());
            REQUIRE(hand.limit_compensation.get_snapshot()->values == target);

            const auto sent = raw->sent();
            REQUIRE(sent.size() == 4);
            for (size_t i = 0; i < sent.size(); ++i) {
                REQUIRE(sent[i].id == id);
                REQUIRE(sent[i].data[0] == (i < 2 ? 0x03 : 0x38));
                REQUIRE(sent[i].dlc == (i % 2 == 0 ? 7 : 1));
                if (sent[i].dlc == 7) {
                    REQUIRE(std::equal(target.begin(), target.end(), sent[i].data.begin() + 1));
                }
            }
        }
    }
}
