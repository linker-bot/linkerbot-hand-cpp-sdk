#include "../../../src/hand/unified/detail/cached_snapshot.hpp"
#include "../../../src/hand/unified/detail/freshness.hpp"
#include "../../../validation/fake_classic_hand.hpp"
#include "hand/support/optional_value.hpp"
#include <algorithm>
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <linkerbot/can/fake_transport.hpp>
#include <linkerbot/can/message.hpp>
#include <linkerbot/hand/api/config.hpp>
#include <linkerbot/hand/api/device.hpp>
#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/health.hpp>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace linkerbot::hand;
using namespace std::chrono_literals;

TEST_CASE("cached snapshots are shaped and zero-I/O before open for all supported models", "[comm][python-phase1]") {
    for (const auto model : supported_models()) {
        HandDeviceConfig config;
        config.model = model;
        config.side = model == Model::kL30 ? HandSide::kUnknown : HandSide::kLeft;
        if (model == Model::kO30) {
            config.o30_layout = O30Layout::kPythonV25;
        }
        int factories = 0;
        const HandDevice device(config, [&] {
            ++factories;
            return std::make_unique<linkerbot::can::FakeTransport>();
        });
        auto snapshot = device.get_cached_snapshot();
        const auto caps = device.get_capabilities();
        REQUIRE((factories == 0));
        REQUIRE((snapshot.model == model));
        REQUIRE((snapshot.connection_generation == 0));
        REQUIRE((snapshot.snapshot_sequence == 0));
        REQUIRE((snapshot.joints.size() == caps.joints.size()));
        REQUIRE((test_support::require_value(snapshot.angles).raw.size() == caps.joints.size()));
        unified_detail::visit_snapshot_samples(snapshot, [](auto &sample, TelemetrySemantic) {
            if (sample) {
                REQUIRE_FALSE(sample->meta.valid);
                REQUIRE((sample->meta.stale));
                REQUIRE_FALSE(sample->meta.collected_at);
            }
        });
        REQUIRE_THROWS_AS(device.get_snapshot(), DeviceNotReadyError);
    }
}

TEST_CASE("cached snapshots retain invalid evidence across close and reset on reopen", "[comm][python-phase1]") {
    linkerbot::can::FakeTransport *raw = nullptr;
    HandDevice device(validation::config(), [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        raw = transport.get();
        raw->on_request(0x28, validation::l6_reply);
        return transport;
    });
    device.open();
    const auto ready = device.get_cached_snapshot();
    REQUIRE((ready.connection_generation == 1));
    REQUIRE((test_support::require_value(ready.angles).meta.valid));
    REQUIRE_FALSE(test_support::require_value(ready.position_kp).meta.valid);
    REQUIRE((device.get_cached_snapshot().snapshot_sequence == ready.snapshot_sequence));
    raw->on_request(0x28, [](const auto &request) {
        auto replies = validation::l6_reply(request);
        if (request.data[0] == 1) {
            std::fill_n(replies.front().data.begin() + 1, 6, uint8_t{88});
        }
        return replies;
    });
    REQUIRE((device.get_angles().raw == std::vector<double>(6, 88)));
    // No explicit snapshot projection between this last query and close.
    device.close();
    const auto closed = device.get_cached_snapshot();
    REQUIRE((test_support::require_value(closed.angles).raw == std::vector<double>(6, 88)));
    REQUIRE_FALSE(test_support::require_value(closed.angles).meta.valid);
    REQUIRE((test_support::require_value(closed.angles).meta.stale));
    REQUIRE_FALSE(test_support::require_value(closed.angles).meta.collected_at);
    REQUIRE_FALSE(closed.joints.front().angle_raw);
    REQUIRE_FALSE(closed.device_info);
    REQUIRE((closed.connection_generation == ready.connection_generation));
    REQUIRE((closed.snapshot_sequence > ready.snapshot_sequence));
    REQUIRE((device.get_cached_snapshot().snapshot_sequence == closed.snapshot_sequence));
    REQUIRE_THROWS_AS(device.get_snapshot(), DeviceNotReadyError);
    device.open();
    const auto reopened = device.get_cached_snapshot();
    REQUIRE((reopened.connection_generation == 2));
    REQUIRE((test_support::require_value(reopened.angles).meta.connection_generation == 2));
    REQUIRE((test_support::require_value(reopened.angles).meta.valid));
    REQUIRE((test_support::require_value(reopened.angles).raw == std::vector<double>(6, 64)));
    REQUIRE_FALSE(test_support::require_value(reopened.position_kp).meta.valid);
    REQUIRE((reopened.snapshot_sequence > closed.snapshot_sequence));
    device.close();
}

TEST_CASE("identity refresh preserves queued commands generation and polling", "[comm][python-phase1]") {
    validation::FakeHand device;
    const auto before = device.hand.get_cached_snapshot();
    auto executing = device.block();
    auto pending = device.hand.set_position_kp(std::vector<double>(6, 42));
    device.hand.start_polling({{TelemetrySemantic::kAngles, 20ms}});
    auto identity = std::async(std::launch::async, [&] { return device.hand.refresh_device_info(1s); });
    const auto waiting = identity.wait_for(20ms);
    const auto still_open = device.hand.is_open();
    const auto pending_before_release = pending.done();
    device.transport->release_tx_gate();
    REQUIRE((waiting == std::future_status::timeout));
    REQUIRE((still_open));
    REQUIRE_FALSE(pending_before_release);
    REQUIRE((identity.get().model == Model::kL6));
    validation::complete(executing);
    validation::complete(pending);
    REQUIRE((device.hand.get_health().polling_active));
    const auto refreshed = device.hand.get_cached_snapshot();
    REQUIRE((refreshed.connection_generation == before.connection_generation));
    REQUIRE((test_support::require_value(refreshed.device_info).serial_number ==
             test_support::require_value(before.device_info).serial_number));
    REQUIRE((refreshed.snapshot_sequence > before.snapshot_sequence));
    device.hand.stop_polling();
}

TEST_CASE("identity refresh timeout does not end readiness or invalidate data", "[comm][python-phase1]") {
    validation::FakeHand device;
    const auto before = device.hand.get_cached_snapshot();
    const auto sent = device.transport->sent().size();
    REQUIRE_THROWS_AS(device.hand.refresh_device_info(0ms), ValidationError);
    REQUIRE((device.transport->sent().size() == sent));
    device.transport->on_request(0x28, [](const auto &request) {
        return request.data[0] == 0xC0 ? std::vector<linkerbot::can::Message>{} : validation::l6_reply(request);
    });
    REQUIRE_THROWS_AS(device.hand.refresh_device_info(30ms), TimeoutError);
    REQUIRE((device.hand.is_open()));
    const auto after = device.hand.get_cached_snapshot();
    REQUIRE((test_support::require_value(after.angles).meta.valid));
    REQUIRE((after.connection_generation == before.connection_generation));
    validation::complete(device.hand.set_position_kp(std::vector<double>(6, 20)));
}

TEST_CASE("identity refresh mismatch faults admission and invalidates cached evidence", "[comm][python-phase1]") {
    validation::FakeHand device;
    const auto before = device.hand.get_cached_snapshot();
    device.transport->on_request(0x28, [](const auto &request) {
        auto replies = validation::l6_reply(request);
        if (request.data[0] == 0xC0) {
            const std::string code = "LHL6-1-999-L";
            for (size_t chunk = 0; chunk < replies.size(); ++chunk) {
                for (size_t i = 0; i < 6; ++i) {
                    const auto index = chunk * 6 + i;
                    replies[chunk].data[i + 2] = index < code.size() ? static_cast<uint8_t>(code[index]) : uint8_t{0};
                }
            }
        }
        return replies;
    });
    REQUIRE_THROWS_AS(device.hand.refresh_device_info(), IdentityMismatchError);
    REQUIRE_FALSE(device.hand.is_open());
    const auto health = device.hand.get_health();
    REQUIRE((health.lifecycle_state == LifecycleState::kFaulted));
    REQUIRE((test_support::require_value(health.recent_error).code == ErrorCode::kIdentityMismatch));
    REQUIRE_THROWS_AS(device.hand.set_angles(std::vector<int32_t>(6, 1)), DeviceNotReadyError);
    const auto cached = device.hand.get_cached_snapshot();
    REQUIRE((test_support::require_value(cached.angles).raw == test_support::require_value(before.angles).raw));
    REQUIRE_FALSE(test_support::require_value(cached.angles).meta.valid);
    REQUIRE_FALSE(cached.device_info);
    REQUIRE_FALSE(cached.joints.front().angle_raw);
    REQUIRE((cached.connection_generation == before.connection_generation));
}

TEST_CASE("V25 freshness uses Python torque-limit and motion-time boundaries", "[comm][python-phase1]") {
    HandDeviceConfig config;
    config.model = Model::kO30;
    config.side = HandSide::kLeft;
    config.o30_layout = O30Layout::kPythonV25;
    const auto caps = declared_capabilities(config);
    const auto now = std::chrono::steady_clock::now();
    for (const auto semantic : {TelemetrySemantic::kTorqueLimits, TelemetrySemantic::kMotionTimes}) {
        const auto *channel = caps.telemetry_channel(semantic);
        const auto limit = semantic == TelemetrySemantic::kTorqueLimits ? 100ms : 300ms;
        REQUIRE((channel->freshness_ms == static_cast<double>(limit.count())));
        SampleMeta meta;
        meta.valid = true;
        meta.collected_at = now;
        unified_detail::complete_freshness(meta, channel, now + limit - 1us);
        REQUIRE_FALSE(meta.stale);
        unified_detail::complete_freshness(meta, channel, now + limit);
        REQUIRE_FALSE(meta.stale);
        unified_detail::complete_freshness(meta, channel, now + limit + 1us);
        REQUIRE((meta.stale));
    }
    REQUIRE_FALSE(declared_capabilities(Model::kO30).supports(TelemetrySemantic::kTorqueLimits));
}

TEST_CASE("readable cache is invalid during recovery and commits only a successful generation",
          "[comm][python-phase1]") {
    std::atomic<int> factories{0};
    std::promise<void> entered;
    std::promise<void> release;
    auto entered_future = entered.get_future();
    auto gate = release.get_future().share();
    HandDevice device(validation::config(), [&] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        const bool recovery = ++factories == 2;
        transport->on_request(0x28, [&, recovery](const auto &request) {
            if (recovery && request.data[0] == 0xC0) {
                entered.set_value();
                gate.wait();
            }
            return validation::l6_reply(request);
        });
        return transport;
    });
    device.open();
    validation::complete(device.set_position_kp(std::vector<double>(6, 42)));
    const auto before = device.get_cached_snapshot();
    auto recovery = std::async(std::launch::async, [&] { device.recover(); });
    const auto reached = entered_future.wait_for(1s);
    const auto during = device.get_cached_snapshot();
    release.set_value();
    recovery.get();
    REQUIRE((reached == std::future_status::ready));
    REQUIRE_FALSE(test_support::require_value(during.angles).meta.valid);
    REQUIRE_FALSE(during.joints.front().angle_raw);
    REQUIRE((during.connection_generation == before.connection_generation));
    REQUIRE((test_support::require_value(during.position_kp).raw == std::vector<double>(6, 42)));
    REQUIRE_FALSE(test_support::require_value(during.position_kp).meta.valid);
    const auto after = device.get_cached_snapshot();
    REQUIRE((after.connection_generation == before.connection_generation + 1));
    REQUIRE((test_support::require_value(after.angles).meta.valid));
    REQUIRE_FALSE(test_support::require_value(after.position_kp).meta.valid);
    REQUIRE((test_support::require_value(after.position_kp).raw == std::vector<double>(6, 0)));
    REQUIRE((after.snapshot_sequence > during.snapshot_sequence));
    device.close();
}

TEST_CASE("concurrent close fences a non-destructive identity refresh", "[comm][python-phase1]") {
    validation::FakeHand device;
    const auto before = device.hand.get_cached_snapshot();
    std::promise<void> entered;
    std::promise<void> release;
    auto entered_future = entered.get_future();
    auto gate = release.get_future().share();
    device.transport->on_request(0x28, [&](const auto &request) {
        if (request.data[0] == 0xC0) {
            entered.set_value();
            gate.wait();
        }
        return validation::l6_reply(request);
    });
    auto refresh = std::async(std::launch::async, [&] { return device.hand.refresh_device_info(1s); });
    const auto reached = entered_future.wait_for(1s);
    auto closing = std::async(std::launch::async, [&] { device.hand.close(); });
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (device.hand.is_open() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    const auto during = device.hand.get_cached_snapshot();
    release.set_value();
    closing.get();
    REQUIRE((reached == std::future_status::ready));
    REQUIRE_THROWS_AS(refresh.get(), HandError);
    REQUIRE_FALSE(test_support::require_value(during.angles).meta.valid);
    REQUIRE((test_support::require_value(during.angles).raw == test_support::require_value(before.angles).raw));
    REQUIRE_FALSE(test_support::require_value(device.hand.get_cached_snapshot().angles).meta.valid);
}

TEST_CASE("failed open never publishes a partial baseline into the readable cache", "[comm][python-phase1]") {
    auto config = validation::config();
    config.transaction_timeout = 20ms;
    HandDevice device(config, [] {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        transport->on_request(0x28, [](const auto &request) {
            return request.data[0] == 0x35 ? std::vector<linkerbot::can::Message>{} : validation::l6_reply(request);
        });
        return transport;
    });
    REQUIRE_THROWS_AS(device.open(), TimeoutError);
    const auto cached = device.get_cached_snapshot();
    REQUIRE((cached.connection_generation == 0));
    REQUIRE((cached.snapshot_sequence == 0));
    REQUIRE_FALSE(test_support::require_value(cached.angles).meta.valid);
    REQUIRE_FALSE(cached.device_info);
    REQUIRE_FALSE(cached.joints.front().angle_raw);
}

TEST_CASE("readable cache neither waits for a claimed write nor adds device I/O", "[comm][python-phase1]") {
    validation::FakeHand device;
    const auto baseline = device.hand.get_cached_snapshot();
    const auto command = device.block();
    const auto sent = device.transport->sent().size();
    auto reading = std::async(std::launch::async, [&] {
        const auto cache = device.hand.get_cached_snapshot();
        return std::make_pair(cache, device.transport->sent().size());
    });
    const auto available = reading.wait_for(500ms);
    device.transport->release_tx_gate();
    const auto observed = reading.get();
    validation::complete(command);
    REQUIRE((available == std::future_status::ready));
    REQUIRE((observed.second == sent));
    REQUIRE(
        (test_support::require_value(observed.first.angles).raw == test_support::require_value(baseline.angles).raw));
    REQUIRE((test_support::require_value(observed.first.angles).meta.valid));
}
