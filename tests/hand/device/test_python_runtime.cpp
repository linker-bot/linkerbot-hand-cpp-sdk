#include "../../../src/hand/unified/detail/read_coordinator.hpp"
#include "../../../validation/fake_classic_hand.hpp"
#include "hand/support/optional_value.hpp"
#include <algorithm>
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <future>
#include <thread>
#include <type_traits>

using namespace linkerbot::hand;
using namespace std::chrono_literals;
static_assert(std::is_same_v<decltype(&HandDevice::recover), void (HandDevice::*)()>);
namespace {
    HandDeviceConfig python_config() {
        auto config = validation::config();
        config.runtime_behavior = RuntimeBehavior::kPython;
        config.response_timeout = 100ms;
        config.identity_timeout = 100ms;
        return config;
    }
}

TEST_CASE("Python read wait timeout does not cancel a shared transaction", "[comm][python-phase2]") {
    validation::FakeHand device(python_config());
    device.transport->arm_tx_gate();
    auto first = std::async(std::launch::async, [&] {
        try {
            (void)device.hand.get_angles(30ms);
            return false;
        } catch (const TimeoutError &) {
            return true;
        }
    });
    const bool entered = device.transport->wait_until_tx_entered(1s);
    auto second = std::async(std::launch::async, [&] { return device.hand.get_angles(1s); });
    const auto first_timed_out = first.get();
    const auto count = device.transport->sent().size();
    device.transport->release_tx_gate();
    const auto result = second.get();
    REQUIRE((entered));
    REQUIRE((first_timed_out));
    REQUIRE((result.raw == std::vector<double>(6, 64)));
    REQUIRE((device.transport->sent().size() == count + 1));
}

TEST_CASE("Python response window starts after a delayed physical send", "[comm][python-phase2]") {
    auto config = python_config();
    config.command_timeout = 10ms;
    config.response_timeout = 20ms;
    validation::FakeHand device(config);
    auto command = device.block();
    auto pending = device.hand.set_angles(std::vector<int32_t>(6, 20));
    std::this_thread::sleep_for(150ms);
    const bool active_done = command.done();
    const bool pending_done = pending.done();
    device.transport->release_tx_gate();
    validation::complete(command);
    validation::complete(pending);
    REQUIRE_FALSE(active_done);
    REQUIRE_FALSE(pending_done);
    REQUIRE((pending.stage() == CommandStage::kDispatched));
}

TEST_CASE("Python matching short responses fail without replacing the successful cache", "[comm][python-phase2]") {
    validation::FakeHand device(python_config());
    const auto before = device.hand.get_cached_snapshot();
    device.transport->on_request(0x28, [](const auto &request) {
        auto replies = validation::l6_reply(request);
        if (request.data[0] == 1) {
            replies.front().dlc = 2;
        }
        return replies;
    });
    REQUIRE_THROWS_AS(device.hand.get_angles(1s), ProtocolError);
    const auto after = device.hand.get_cached_snapshot();
    REQUIRE((test_support::require_value(after.angles).raw == test_support::require_value(before.angles).raw));
    REQUIRE((test_support::require_value(after.angles).meta.valid));
    REQUIRE((device.hand.is_open()));
}

TEST_CASE("Python close preserves a claimed no-ACK delivery and rejects pending commands", "[comm][python-phase2]") {
    validation::FakeHand device(python_config());
    auto claimed = device.block();
    auto pending = device.hand.set_angles(std::vector<int32_t>(6, 20));
    auto closed = std::async(std::launch::async, [&] { device.hand.close(); });
    const auto limit = std::chrono::steady_clock::now() + 1s;
    while (device.hand.is_open() && std::chrono::steady_clock::now() < limit) {
        std::this_thread::yield();
    }
    const bool claimed_done = claimed.done();
    const bool pending_finished = pending.wait_for(100ms);
    device.transport->release_tx_gate();
    closed.get();
    validation::complete(claimed);
    REQUIRE_FALSE(claimed_done);
    REQUIRE((pending_finished));
    REQUIRE((pending.stage() == CommandStage::kFailed));
    REQUIRE((claimed.stage() == CommandStage::kDispatched));
    REQUIRE((claimed.send_results().size() == 1));
}

TEST_CASE("Python recovery caller timeout leaves one shared recovery running", "[comm][python-phase2]") {
    auto config = python_config();
    std::atomic<int> factories{0};
    std::atomic<linkerbot::can::FakeTransport *> next{nullptr};
    HandDevice device(config, [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        transport->on_request(0x28, validation::l6_reply);
        if (++factories == 2) {
            transport->arm_tx_gate();
            next = transport.get();
        }
        return transport;
    });
    device.open();
    REQUIRE_THROWS_AS(device.recover_for(30ms), TimeoutError);
    const auto reconnected = next.load();
    const bool entered = reconnected && reconnected->wait_until_tx_entered(1s);
    auto joined = std::async(std::launch::async, [&] { device.recover_for(2s); });
    std::this_thread::sleep_for(20ms);
    if (reconnected) {
        reconnected->release_tx_gate();
    }
    joined.get();
    REQUIRE((entered));
    REQUIRE((device.is_open()));
    REQUIRE((factories.load() == 2));
    device.close();
}

TEST_CASE("Python invalid runtime configuration is rejected before transport construction", "[comm][python-phase2]") {
    for (unsigned invalid = 0; invalid < 3; ++invalid) {
        auto config = python_config();
        if (invalid == 0) {
            // Deliberately exercise rejection of an invalid configuration enum.
            // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
            config.runtime_behavior = static_cast<RuntimeBehavior>(99);
        } else if (invalid == 1) {
            config.read_waiter_capacity = 0;
        } else {
            config.response_timeout = 0ms;
        }
        int factories = 0;
        REQUIRE_THROWS_AS(HandDevice(config,
                                     [&] {
                                         ++factories;
                                         return std::make_unique<linkerbot::can::FakeTransport>();
                                     }),
                          ValidationError);
        REQUIRE((factories == 0));
    }
}

TEST_CASE("Python recovery retries only lost responses without reopening the backend again", "[comm][python-phase2]") {
    auto config = python_config();
    config.response_timeout = 20ms;
    config.identity_timeout = 20ms;
    config.recovery_quiet = 20ms;
    config.recovery_attempts = 3;
    std::atomic<int> factories{0};
    std::atomic<int> identity_reads{0};
    HandDevice device(config, [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        const auto generation = ++factories;
        transport->on_request(0x28, [&, generation](const auto &request) {
            if (generation > 1 && request.data[0] == 0xC0 && ++identity_reads == 1) {
                return std::vector<linkerbot::can::Message>{};
            }
            return validation::l6_reply(request);
        });
        return transport;
    });
    device.open();
    device.recover_for(2s);
    REQUIRE((device.is_open()));
    REQUIRE((factories.load() == 2));
    REQUIRE((identity_reads.load() == 2));
    REQUIRE((device.get_cached_snapshot().connection_generation == 2));
    device.close();
}

TEST_CASE("Python recovery does not retry malformed replies and can recover from Faulted", "[comm][python-phase2]") {
    auto config = python_config();
    config.recovery_attempts = 3;
    std::atomic<int> factories{0};
    std::atomic<int> baseline_reads{0};
    HandDevice device(config, [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        const auto generation = ++factories;
        transport->on_request(0x28, [&, generation](const auto &request) {
            auto replies = validation::l6_reply(request);
            if (generation == 2 && request.data[0] == 1) {
                ++baseline_reads;
                replies.front().dlc = 2;
            }
            return replies;
        });
        return transport;
    });
    device.open();
    REQUIRE_THROWS_AS(device.recover_for(2s), ProtocolError);
    REQUIRE_FALSE(device.is_open());
    REQUIRE((baseline_reads.load() == 1));
    REQUIRE((factories.load() == 2));
    REQUIRE((device.get_cached_snapshot().connection_generation == 1));
    device.recover_for(2s);
    REQUIRE((device.is_open()));
    REQUIRE((factories.load() == 3));
    REQUIRE((device.get_cached_snapshot().connection_generation == 2));
    device.close();
}

TEST_CASE("Python recovery preparation expiry cannot start a delayed reconnect", "[comm][python-phase2]") {
    auto config = python_config();
    std::atomic<int> factories{0};
    linkerbot::can::FakeTransport *raw = nullptr;
    HandDevice device(config, [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        raw = transport.get();
        ++factories;
        raw->on_request(0x28, validation::l6_reply);
        return transport;
    });
    device.open();
    raw->arm_tx_gate();
    const auto command = device.set_angles(std::vector<int32_t>(6, 10));
    const bool entered = raw->wait_until_tx_entered(1s);
    bool timed_out = false;
    try {
        device.recover_for(30ms);
    } catch (const TimeoutError &) {
        timed_out = true;
    }
    // Let preparation observe its deadline while the claimed send is blocked.
    std::this_thread::sleep_for(30ms);
    raw->release_tx_gate();
    device.close();
    REQUIRE((entered));
    REQUIRE((timed_out));
    REQUIRE((factories.load() == 1));
    REQUIRE_FALSE(device.is_open());
    REQUIRE((command.done()));
}

TEST_CASE("Python close cannot turn a claimed required ACK command into success", "[comm][python-phase2]") {
    validation::FakeHand device(python_config());
    device.transport->arm_tx_gate();
    auto command = device.hand.set_position_kp(std::vector<double>(6, 20));
    const bool entered = device.transport->wait_until_tx_entered(1s);
    auto closed = std::async(std::launch::async, [&] { device.hand.close(); });
    const bool terminal = command.wait_for(500ms);
    device.transport->release_tx_gate();
    closed.get();
    REQUIRE((entered));
    REQUIRE((terminal));
    REQUIRE((command.stage() == CommandStage::kFailed));
    REQUIRE((command.send_results().size() == 1));
    REQUIRE_FALSE(device.hand.is_open());
}

TEST_CASE("Python lost response retains the cache and allows a fresh transaction after its guard",
          "[comm][python-phase2]") {
    auto config = python_config();
    config.response_timeout = 20ms;
    validation::FakeHand device(config);
    const auto before = device.hand.get_cached_snapshot();
    device.transport->on_request(0x28, [](const auto &) { return std::vector<linkerbot::can::Message>{}; });
    REQUIRE_THROWS_AS(device.hand.get_angles(1s), ResponseTimeoutError);
    const auto cached = device.hand.get_cached_snapshot();
    REQUIRE((test_support::require_value(cached.angles).meta.valid));
    REQUIRE((test_support::require_value(cached.angles).raw == test_support::require_value(before.angles).raw));
    device.transport->on_request(0x28, validation::l6_reply);
    REQUIRE((device.hand.get_angles(1s).raw == std::vector<double>(6, 64)));
    REQUIRE((device.hand.is_open()));
}

TEST_CASE("Python latest targets remain replaceable while a read owns the device lane", "[comm][python-phase2]") {
    validation::FakeHand device(python_config());
    device.transport->arm_tx_gate();
    auto reading = std::async(std::launch::async, [&] { return device.hand.get_angles(2s); });
    const bool entered = device.transport->wait_until_tx_entered(1s);
    auto older = device.hand.set_angles(std::vector<int32_t>(6, 10));
    std::this_thread::sleep_for(10ms);
    auto newest = device.hand.set_angles(std::vector<int32_t>(6, 20));
    const auto older_stage = older.stage();
    device.transport->release_tx_gate();
    (void)reading.get();
    validation::complete(newest);
    REQUIRE((entered));
    REQUIRE((older_stage == CommandStage::kSuperseded));
    REQUIRE((older.sent_frames() == 0));
}

TEST_CASE("Python polling shares an active query without consuming reader capacity", "[comm][python-phase2]") {
    auto config = python_config();
    config.read_waiter_capacity = 1;
    validation::FakeHand device(config);
    device.transport->clear_sent();
    device.transport->arm_tx_gate();
    device.hand.start_polling({{TelemetrySemantic::kAngles, 500ms}});
    const bool entered = device.transport->wait_until_tx_entered(1s);
    auto reading = std::async(std::launch::async, [&] { return device.hand.get_angles(1s); });
    std::this_thread::sleep_for(20ms);
    device.transport->release_tx_gate();
    const auto result = reading.get();
    device.hand.stop_polling();
    REQUIRE((entered));
    REQUIRE((result.raw == std::vector<double>(6, 64)));
    REQUIRE((device.transport->sent().size() == 1));
}

TEST_CASE("Python recovery loss has a finite attempt count and cannot publish a partial generation",
          "[comm][python-phase2]") {
    auto config = python_config();
    config.response_timeout = 20ms;
    config.identity_timeout = 20ms;
    config.recovery_quiet = 20ms;
    config.recovery_attempts = 3;
    std::atomic<int> factories{0};
    std::atomic<int> identity_reads{0};
    HandDevice device(config, [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        const auto generation = ++factories;
        transport->on_request(0x28, [&, generation](const auto &request) {
            if (generation > 1 && request.data[0] == 0xC0) {
                ++identity_reads;
                return std::vector<linkerbot::can::Message>{};
            }
            return validation::l6_reply(request);
        });
        return transport;
    });
    device.open();
    REQUIRE_THROWS_AS(device.recover_for(2s), ResponseTimeoutError);
    REQUIRE((identity_reads.load() == 3));
    REQUIRE((factories.load() == 2));
    REQUIRE_FALSE(device.is_open());
    const auto cached = device.get_cached_snapshot();
    REQUIRE((cached.connection_generation == 1));
    REQUIRE_FALSE(test_support::require_value(cached.angles).meta.valid);
    device.close();
}

TEST_CASE("Python recovery quiet periods cannot extend the execution budget", "[comm][python-phase2]") {
    auto config = python_config();
    config.response_timeout = 20ms;
    config.identity_timeout = 20ms;
    config.recovery_quiet = 1s;
    config.recovery_timeout = 80ms;
    config.recovery_attempts = 3;
    std::atomic<int> factories{0};
    std::atomic<int> identity_reads{0};
    HandDevice device(config, [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        const auto generation = ++factories;
        transport->on_request(0x28, [&, generation](const auto &request) {
            if (generation > 1 && request.data[0] == 0xC0) {
                ++identity_reads;
                return std::vector<linkerbot::can::Message>{};
            }
            return validation::l6_reply(request);
        });
        return transport;
    });
    device.open();
    const auto started = std::chrono::steady_clock::now();
    REQUIRE_THROWS_AS(device.recover_for(1s), TimeoutError);
    REQUIRE((std::chrono::steady_clock::now() - started < 500ms));
    REQUIRE((identity_reads.load() == 1));
    REQUIRE_FALSE(device.is_open());
    device.close();
}

TEST_CASE("Python device read and close do not stop a peer on the shared Session", "[comm][python-phase2]") {
    auto transport = std::make_unique<linkerbot::can::FakeTransport>();
    auto *raw = transport.get();
    std::atomic<bool> drop{false};
    std::promise<void> request_seen;
    raw->on_request(0x28, [&](const auto &request) {
        if (drop.load() && request.data[0] == 1) {
            request_seen.set_value();
            return std::vector<linkerbot::can::Message>{};
        }
        return validation::l6_reply(request);
    });
    raw->on_request(0x27, [](const auto &request) {
        auto replies = validation::l6_reply(request);
        if (request.data[0] == 0xC0) {
            const size_t index = std::string("LHL6-1-123-L").size() - 1;
            replies.at(index / 6).data[2 + index % 6] = 'R';
        }
        return replies;
    });
    auto session = linkerbot::transport::Session::own(std::move(transport));
    auto config = python_config();
    config.response_timeout = 1s;
    HandDevice left(config, session);
    config.side = HandSide::kRight;
    HandDevice right(config, session);
    left.open();
    right.open();
    drop.store(true);
    auto reading = std::async(std::launch::async, [&] {
        try {
            (void)left.get_angles(2s);
            return false;
        } catch (const HandError &) {
            return true;
        }
    });
    const auto entered = request_seen.get_future().wait_for(1s);
    const auto peer = right.get_angles(1s);
    left.close();
    const bool ended = reading.get();
    REQUIRE((entered == std::future_status::ready));
    REQUIRE((ended));
    REQUIRE((peer.raw == std::vector<double>(6, 64)));
    REQUIRE((right.is_open()));
    REQUIRE((session->health().open));
    REQUIRE((right.get_angles(1s).raw == std::vector<double>(6, 64)));
    right.close();
    session->close();
}

TEST_CASE("Python close cannot start an abandoned read after the claimed command releases its lane",
          "[comm][python-phase2]") {
    auto transport = std::make_unique<linkerbot::can::FakeTransport>();
    auto *raw = transport.get();
    raw->on_request(0x28, validation::l6_reply);
    auto session = linkerbot::transport::Session::own(std::move(transport));
    HandDevice hand(python_config(), session);
    hand.open();
    raw->clear_sent();
    raw->arm_tx_gate();
    auto claimed = hand.set_angles(std::vector<int32_t>(6, 10));
    const bool entered = raw->wait_until_tx_entered(1s);
    auto read_error = ErrorCode::kInternal;
    try {
        (void)hand.get_angles(10ms);
    } catch (const HandError &error) {
        read_error = error.code();
    } catch (...) {
        read_error = ErrorCode::kInternal;
    }
    auto closed = std::async(std::launch::async, [&] { hand.close(); });
    const auto limit = std::chrono::steady_clock::now() + 1s;
    while (hand.is_open() && std::chrono::steady_clock::now() < limit) {
        std::this_thread::yield();
    }
    std::this_thread::sleep_for(20ms);
    raw->release_tx_gate();
    closed.get();
    const auto sent = raw->sent();
    REQUIRE((entered));
    REQUIRE((read_error == ErrorCode::kTimeout));
    validation::complete(claimed);
    REQUIRE((sent.size() == 1));
    REQUIRE((sent.front().dlc == 7));
    hand.open();
    const auto reopened = hand.set_angles(std::vector<int32_t>(6, 20));
    validation::complete(reopened);
    REQUIRE((reopened.stage() == CommandStage::kDispatched));
    hand.close();
    session->close();
}

TEST_CASE("Python unclaimed reads with no remaining callers do not send after the command lane becomes free",
          "[comm][python-phase2]") {
    validation::FakeHand device(python_config());
    device.transport->clear_sent();
    auto claimed = device.block();
    bool timed_out = false;
    try {
        (void)device.hand.get_angles(10ms);
    } catch (const TimeoutError &) {
        timed_out = true;
    }
    device.transport->release_tx_gate();
    validation::complete(claimed);
    // A later identity transaction also drains prior read coordination work.
    (void)device.hand.refresh_device_info(1s);
    const auto sent = device.transport->sent();
    const auto queries = std::count_if(sent.begin(), sent.end(),
                                       [](const auto &message) { return message.data[0] == 1 && message.dlc == 1; });
    REQUIRE((timed_out));
    REQUIRE((queries == 0));
    REQUIRE((device.hand.is_open()));
}

TEST_CASE("Python query-only poll demand survives its internal waiter timeout while the device lane is busy",
          "[comm][python-phase2]") {
    struct State {
        std::promise<void> polled;
        std::atomic<bool> observed{false};
    };
    const auto state = std::make_shared<State>();
    auto polled = state->polled.get_future();
    validation::FakeHand device(python_config());
    device.transport->on_request(0x28, [state](const auto &request) {
        if (request.data[0] == 1 && request.dlc == 1 && !state->observed.exchange(true)) {
            state->polled.set_value();
        }
        return validation::l6_reply(request);
    });
    auto claimed = device.block();
    device.hand.start_polling({{TelemetrySemantic::kAngles, 2s, AngleFeedbackPolicy::kQueryOnly}});
    std::this_thread::sleep_for(150ms); // Longer than the internal 100 ms poll wait.
    device.transport->release_tx_gate();
    validation::complete(claimed);
    const auto progressed = polled.wait_for(500ms);
    device.hand.stop_polling();
    device.hand.close();
    REQUIRE((progressed == std::future_status::ready));
}

TEST_CASE("Python identity response windows are independent of the ordinary transaction override",
          "[comm][python-phase2]") {
    struct State {
        std::atomic<bool> delay{false};
        std::promise<void> requested;
        std::vector<linkerbot::can::Message> replies;
    };
    const auto state = std::make_shared<State>();
    auto requested = state->requested.get_future();
    auto config = python_config();
    config.response_timeout = 20ms;
    config.identity_timeout = 300ms;
    validation::FakeHand device(config);
    device.transport->on_request(0x28, [state](const auto &request) {
        auto replies = validation::l6_reply(request);
        if (state->delay.load() && request.data[0] == 0xC0) {
            state->replies = std::move(replies);
            state->requested.set_value();
            return std::vector<linkerbot::can::Message>{};
        }
        return replies;
    });
    state->delay.store(true);
    auto refreshing = std::async(std::launch::async, [&] { return device.hand.refresh_device_info(1s); });
    const auto entered = requested.wait_for(1s);
    std::this_thread::sleep_for(100ms);
    if (entered == std::future_status::ready) {
        for (const auto &reply : state->replies) {
            device.transport->inject(reply);
        }
    }
    const auto identity = refreshing.get();
    device.hand.close();
    REQUIRE((entered == std::future_status::ready));
    REQUIRE((identity.model == Model::kL6));
}

TEST_CASE("native and Python policies preserve command errors during unwinding", "[comm][python-phase2]") {
    for (const auto behavior : {RuntimeBehavior::kNative, RuntimeBehavior::kPython}) {
        auto config = python_config();
        config.runtime_behavior = behavior;
        config.command_timeout = 30ms;
        config.response_timeout = 30ms;
        validation::FakeHand device(config);
        device.transport->on_request(0x28, [](const auto &request) {
            if (request.data[0] == 0xC6) {
                return std::vector<linkerbot::can::Message>{};
            }
            return validation::l6_reply(request);
        });
        const auto failed = device.hand.set_stall_thresholds(std::vector<double>(6, 20));
        REQUIRE((failed.wait_for(2s)));
        REQUIRE((failed.stage() == CommandStage::kFailed));
        REQUIRE_THROWS_AS(std::rethrow_exception(failed.error()), TimeoutError);
        const auto next = device.hand.set_angles(std::vector<int32_t>(6, 20));
        validation::complete(next);
        REQUIRE((next.stage() == CommandStage::kDispatched));
        device.hand.close();
    }
}

TEST_CASE("recovery resumes an endpoint after draining its claimed dispatch", "[comm][python-phase2]") {
    validation::FakeHand device(python_config());
    auto *blocked = device.transport;
    blocked->arm_tx_gate();
    const auto command = device.hand.set_angles(std::vector<int32_t>(6, 10));
    const bool claimed = blocked->wait_until_tx_entered(1s);
    auto recovering = std::async(std::launch::async, [&] { device.hand.recover_for(2s); });
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (device.hand.is_open() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    blocked->release_tx_gate();
    recovering.get();
    REQUIRE((claimed));
    const auto receipts = command.send_results();
    CAPTURE(command.stage(), command.sent_frames(), receipts.size());
    if (!receipts.empty()) {
        CAPTURE(receipts.front().outcome, receipts.front().sent_frames);
    }
    REQUIRE((command.stage() == CommandStage::kDispatched));
    validation::complete(command);
    const auto next = device.hand.set_angles(std::vector<int32_t>(6, 20));
    validation::complete(next);
    REQUIRE((next.stage() == CommandStage::kDispatched));
    REQUIRE((device.hand.get_cached_snapshot().connection_generation == 2));
    device.hand.close();
}
