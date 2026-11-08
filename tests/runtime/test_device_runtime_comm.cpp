#include "../../src/hand/unified/detail/command_delivery.hpp"
#include "../../src/hand/unified/detail/runtime_access.hpp"
#include "hand/support/optional_value.hpp"
#include "hand/support/throw_as.hpp"
#include <algorithm>
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <condition_variable>
#include <future>
#include <linkerbot/can/dispatcher.hpp>
#include <linkerbot/can/fake_transport.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/o6/protocol_backend.hpp>
#include <linkerbot/hand/runtime/cache.hpp>
#include <linkerbot/hand/runtime/device_runtime.hpp>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

using namespace std::chrono_literals;
namespace {
    using linkerbot::can::FakeTransport;
    using linkerbot::can::Message;
    using linkerbot::hand::runtime::Clock;
    using linkerbot::hand::runtime::DeviceRuntime;
    using linkerbot::transport::Session;
    struct Rig {
        FakeTransport *fake = nullptr;
        std::shared_ptr<Session> session;
        std::shared_ptr<DeviceRuntime> runtime;
        Rig() {
            auto transport = std::make_unique<FakeTransport>();
            fake = transport.get();
            session = Session::own(std::move(transport));
            runtime = DeviceRuntime::acquire(session, "test/0x28", {{0x28, false}});
        }
        ~Rig() {
            runtime->close();
            session->close();
        }
    };
    auto plan() { return linkerbot::hand::o6::protocol::read_angle(0x28); }
}
TEST_CASE("comm: runtime registers before immediate reply and stores generation", "[comm][runtime]") {
    Rig rig;
    rig.fake->on_request(0x28, [](const Message &) {
        Message response;
        response.id = 0x28;
        response.dlc = 7;
        response.data = {1, 0, 51, 102, 153, 204, 255};
        return std::vector<Message>{response};
    });
    const auto sample = rig.runtime->execute(plan(), Clock::now() + 500ms);
    REQUIRE(sample.valid);
    REQUIRE(sample.physical_generation == rig.session->health().generation);
    REQUIRE(sample.origin == linkerbot::hand::protocol::Origin::response);
    REQUIRE(sample.received_at <= Clock::now());
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
    const auto data = std::any_cast<linkerbot::hand::o6::AngleData>(sample.value);
    REQUIRE(data.angle.to_array().back() == 100.0f);
}
TEST_CASE("comm: unbounded runtime request waits allow backend completion", "[comm][runtime][regression]") {
    for (const bool wait_for_send : {false, true}) {
        Rig rig;
        linkerbot::hand::runtime::TransactionPlan command;
        command.domain = "angle";
        Message frame;
        frame.id = 0x28;
        frame.dlc = 1;
        frame.data[0] = 0x01;
        command.frames = {frame};
        rig.fake->arm_tx_gate();
        const auto no_deadline = linkerbot::hand::runtime::Deadline::max();
        auto request = rig.runtime->submit(command, no_deadline);
        REQUIRE(rig.fake->wait_until_tx_entered(500ms));
        auto release = std::async(std::launch::async, [&] {
            std::this_thread::sleep_for(10ms);
            rig.fake->release_tx_gate();
        });
        if (wait_for_send) {
            REQUIRE(request.wait_sent_until(no_deadline).outcome == linkerbot::transport::SendOutcome::dispatched);
        } else {
            (void)request.wait();
            REQUIRE(request.completed());
        }
        release.get();
        REQUIRE(rig.fake->sent().size() == 1);
    }
}

TEST_CASE("comm: health returns a cached zero-I/O aggregate snapshot", "[comm][runtime][health]") {
    Rig rig;
    rig.fake->clear_sent();

    const auto health = rig.runtime->health();

    REQUIRE(health.lifecycle_state == linkerbot::hand::LifecycleState::kOpen);
    REQUIRE(health.connection_state == linkerbot::hand::ConnectionState::kConnected);
    REQUIRE(health.connected());
    REQUIRE(health.identity_verified);
    REQUIRE_FALSE(health.polling_active);
    REQUIRE(health.connection_generation == rig.runtime->runtime_health().device_generation);
    REQUIRE(health.reconnect_count == 0);
    REQUIRE(health.transport.connected());
    REQUIRE(health.session.open);
    REQUIRE(rig.fake->sent().empty());
}
TEST_CASE("comm: health polling state is synchronized with runtime workers", "[comm][runtime][health]") {
    Rig rig;
    std::promise<void> ran;
    rig.runtime->start_polling({{"health", 30s, [&](auto) { ran.set_value(); }}});
    REQUIRE(ran.get_future().wait_for(500ms) == std::future_status::ready);
    REQUIRE(rig.runtime->health().polling_active);

    rig.runtime->stop_polling();
    REQUIRE_FALSE(rig.runtime->health().polling_active);
}
TEST_CASE("comm: health latency window stays bounded during concurrent reads", "[comm][runtime][health]") {
    Rig rig;
    rig.fake->echo_cmd(0x28, 1, 6, 42);
    auto writer = std::async(std::launch::async, [&] {
        for (size_t i = 0; i < linkerbot::hand::kHealthLatencyWindowCapacity + 7; ++i) {
            (void)rig.runtime->execute(plan(), Clock::now() + 1s);
        }
    });

    while (writer.wait_for(0ms) != std::future_status::ready) {
        const auto health = rig.runtime->health();
        REQUIRE(health.transaction_latency.count <= linkerbot::hand::kHealthLatencyWindowCapacity);
        REQUIRE(health.connection_state == linkerbot::hand::ConnectionState::kConnected);
    }
    writer.get();

    const auto health = rig.runtime->health();
    REQUIRE(health.transaction_latency.count == linkerbot::hand::kHealthLatencyWindowCapacity);
    REQUIRE(health.transaction_latency.p50.has_value());
    REQUIRE(health.transaction_latency.p95.has_value());
    REQUIRE(health.transaction_latency.p99.has_value());
    REQUIRE(test_support::require_value(health.transaction_latency.p50) <=
            test_support::require_value(health.transaction_latency.p95));
    REQUIRE(test_support::require_value(health.transaction_latency.p95) <=
            test_support::require_value(health.transaction_latency.p99));
}
TEST_CASE("comm: health exposes registered channels before their first sample", "[comm][runtime][health]") {
    Rig rig;
    const auto observer = rig.runtime->observe(
        "telemetry", [](const Message &) { return linkerbot::hand::protocol::Progress::unmatched(); });

    const auto before = rig.runtime->health();
    const auto find_channel = [](const auto &health, const std::string &name) {
        return std::find_if(health.channels.begin(), health.channels.end(),
                            [&](const auto &channel) { return channel.channel == name; });
    };
    const auto unseen = find_channel(before, "telemetry");
    REQUIRE(unseen != before.channels.end());
    REQUIRE_FALSE(unseen->data_age.valid);
    REQUIRE(unseen->data_age.stale);
    REQUIRE_FALSE(unseen->data_age.age.has_value());
    REQUIRE(unseen->data_age.sequence == 0);

    rig.fake->echo_cmd(0x28, 1, 6, 42);
    (void)rig.runtime->execute(plan(), Clock::now() + 1s);
    const auto after = rig.runtime->health();
    const auto sampled = find_channel(after, "angles");
    REQUIRE(sampled != after.channels.end());
    REQUIRE(sampled->data_age.valid);
    REQUIRE_FALSE(sampled->data_age.stale);
    REQUIRE(sampled->data_age.age.has_value());
    REQUIRE(sampled->data_age.sequence == 1);
    REQUIRE(sampled->data_age.generation == after.connection_generation);
    rig.runtime->invalidate("angle");
    const auto invalidated = rig.runtime->health();
    const auto stale = find_channel(invalidated, "angles");
    REQUIRE(stale != invalidated.channels.end());
    REQUIRE_FALSE(stale->data_age.valid);
    REQUIRE(stale->data_age.stale);
    rig.runtime->unobserve(observer);
}
TEST_CASE("comm: health retains structured context for the latest request error", "[comm][runtime][health]") {
    Rig rig;

    REQUIRE_THROWS_AS(rig.runtime->execute(plan(), Clock::now() + 20ms), linkerbot::hand::TimeoutError);

    const auto health = rig.runtime->health();
    REQUIRE(health.consecutive_errors == 1);
    REQUIRE(health.recent_error.has_value());
    REQUIRE(test_support::require_value(health.recent_error).code == linkerbot::hand::ErrorCode::kTimeout);
    REQUIRE(test_support::require_value(health.recent_error).error_type == "TimeoutError");
    REQUIRE(test_support::require_value(health.recent_error).context.operation == "runtime.submit");
    REQUIRE(test_support::require_value(health.recent_error).context.model == "test/0x28");
    REQUIRE(test_support::require_value(health.recent_error).context.channel == "angle");
    REQUIRE(test_support::require_value(health.recent_error).context.generation == health.connection_generation);
    REQUIRE(test_support::require_value(health.recent_error).context.arbitration_id == 0x28);
}
TEST_CASE("comm: runtime classifies a full send queue at the hand boundary", "[comm][runtime][health][errors]") {
    FakeTransport *fake = nullptr;
    auto session = Session::create(
        [&] {
            auto transport = std::make_unique<FakeTransport>();
            fake = transport.get();
            return transport;
        },
        1);
    auto runtime = DeviceRuntime::acquire(session, "queue-health/0x28", {{0x28, false}});
    fake->arm_tx_gate();
    struct GateRelease {
        FakeTransport *fake = nullptr;

        void release() {
            if (fake == nullptr) {
                return;
            }
            fake->release_tx_gate();
            fake = nullptr;
        }

        ~GateRelease() { release(); }
    } gate_release{fake};

    linkerbot::hand::runtime::TransactionPlan command;
    command.domain = "configuration";
    command.command_family = "configuration";
    Message frame;
    frame.id = 0x28;
    frame.dlc = 1;
    frame.data[0] = 0x01;
    command.frames = {frame};

    auto claimed = runtime->submit(command, Clock::now() + 2s);
    REQUIRE(fake->wait_until_tx_entered(500ms));
    auto queued = runtime->submit(command, Clock::now() + 2s);
    auto rejected = runtime->submit(command, Clock::now() + 2s);
    REQUIRE_THROWS_AS(rejected.wait(), linkerbot::hand::QueueFullError);

    const auto health = runtime->health();
    REQUIRE(health.recent_error.has_value());
    REQUIRE(test_support::require_value(health.recent_error).code == linkerbot::hand::ErrorCode::kQueueFull);
    REQUIRE(test_support::require_value(health.recent_error).error_type == "QueueFullError");
    REQUIRE(test_support::require_value(health.recent_error).context.operation == "runtime.submit");
    REQUIRE(test_support::require_value(health.recent_error).context.model == "queue-health/0x28");
    REQUIRE(test_support::require_value(health.recent_error).context.channel == "configuration");
    REQUIRE(test_support::require_value(health.recent_error).context.arbitration_id == 0x28);
    REQUIRE(health.session.queue_high_water == 1);

    gate_release.release();
    (void)claimed.wait();
    (void)queued.wait();
    runtime->close();
    session->close();
}
TEST_CASE("comm: runtime classifies protocol decoder failures in health", "[comm][runtime][health][errors]") {
    Rig rig;
    linkerbot::hand::runtime::TransactionPlan malformed;
    malformed.domain = "protocol_probe";
    malformed.keys = {"protocol_probe"};
    Message request_frame;
    request_frame.id = 0x28;
    request_frame.dlc = 1;
    request_frame.data[0] = 0x7f;
    malformed.frames = {request_frame};
    malformed.consume = [](const Message &) {
        return test_util::throw_as<linkerbot::hand::protocol::Progress>(
            linkerbot::hand::ProtocolError("malformed protocol reply"));
    };

    auto request = rig.runtime->submit(std::move(malformed), Clock::now() + 1s);
    request.wait_sent_until(Clock::now() + 500ms);
    Message reply;
    reply.id = 0x28;
    reply.dlc = 1;
    reply.data[0] = 0xff;
    rig.fake->inject(reply);
    REQUIRE_THROWS_AS(request.wait(), linkerbot::hand::ProtocolError);

    const auto health = rig.runtime->health();
    REQUIRE(health.recent_error.has_value());
    REQUIRE(test_support::require_value(health.recent_error).code == linkerbot::hand::ErrorCode::kProtocol);
    REQUIRE(test_support::require_value(health.recent_error).error_type == "ProtocolError");
    REQUIRE(test_support::require_value(health.recent_error).context.operation == "runtime.submit");
    REQUIRE(test_support::require_value(health.recent_error).context.model == "test/0x28");
    REQUIRE(test_support::require_value(health.recent_error).context.channel == "protocol_probe");
    REQUIRE(test_support::require_value(health.recent_error).context.arbitration_id == 0x28);
}
TEST_CASE("comm: health does not wait for a pending device response", "[comm][runtime][health]") {
    Rig rig;
    auto request = rig.runtime->submit(plan(), Clock::now() + 5s);

    auto reading = std::async(std::launch::async, [&] { return rig.runtime->health(); });
    const auto status = reading.wait_for(200ms);
    request.cancel();

    REQUIRE(status == std::future_status::ready);
    const auto health = reading.get();
    REQUIRE(health.connection_state == linkerbot::hand::ConnectionState::kConnected);
}
TEST_CASE("comm: health exposes recovery state and generation transition", "[comm][runtime][health]") {
    std::atomic<size_t> opens{0};
    std::promise<void> recovery_entered;
    std::promise<void> allow_recovery;
    auto allow = allow_recovery.get_future().share();
    auto session = Session::create([&] {
        const auto open = ++opens;
        if (open == 2) {
            recovery_entered.set_value();
            allow.wait();
        }
        auto fake = std::make_unique<FakeTransport>();
        fake->echo_cmd(0x28, 1, 6, 42);
        return fake;
    });
    auto runtime = DeviceRuntime::acquire(session, "recovery-test/0x28", {{0x28, false}});
    const auto before = runtime->health();

    auto recovering = std::async(std::launch::async, [&] { runtime->recover(); });
    const bool entered = recovery_entered.get_future().wait_for(1s) == std::future_status::ready;
    std::optional<linkerbot::hand::HandHealth> during;
    if (entered) {
        during = runtime->health();
    }
    allow_recovery.set_value();
    recovering.get();
    const auto after = runtime->health();

    REQUIRE(entered);
    REQUIRE(during->lifecycle_state == linkerbot::hand::LifecycleState::kRecovering);
    REQUIRE(during->connection_state == linkerbot::hand::ConnectionState::kReconnecting);
    REQUIRE(during->connection_generation == before.connection_generation);
    REQUIRE(after.connection_state == linkerbot::hand::ConnectionState::kConnected);
    REQUIRE(after.connection_generation == before.connection_generation + 1);
    REQUIRE(after.reconnect_count == before.reconnect_count + 1);

    runtime->close();
    session->close();
}
TEST_CASE("comm: runtime health recording APIs update channel and command state", "[comm][runtime][health][metrics]") {
    Rig rig;
    const auto received_at = Clock::now() - 3ms;
    rig.runtime->register_channel("angles", 100.0);
    rig.runtime->register_command_family("angle_targets");
    rig.runtime->record_channel_success("angles", received_at, 2ms, 3);
    rig.runtime->record_channel_timeout("angles");
    rig.runtime->record_deadline_miss("angles");

    rig.runtime->record_command_accepted("angle_targets");
    rig.runtime->record_command_accepted("angle_targets");
    rig.runtime->record_command_dispatched("angle_targets");
    rig.runtime->record_command_dispatched("angle_targets");
    rig.runtime->record_command_acknowledged("angle_targets");
    rig.runtime->record_command_superseded("angle_targets");
    rig.runtime->record_command_accepted("angle_targets");
    rig.runtime->record_command_failed("angle_targets");

    const auto health = rig.runtime->health();
    const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                      [](const auto &item) { return item.channel == "angles"; });
    REQUIRE(channel != health.channels.end());
    REQUIRE(channel->requested_hz == 100.0);
    REQUIRE(channel->completed_reads == 1);
    REQUIRE(channel->feedback_samples == 3);
    REQUIRE(channel->timeouts == 1);
    REQUIRE(channel->missed_deadlines == 1);
    REQUIRE(channel->data_age.valid);
    REQUIRE_FALSE(channel->data_age.stale);
    REQUIRE(channel->data_age.age >= 3ms);
    REQUIRE(channel->data_age.sequence == 1);
    REQUIRE(channel->data_age.generation == health.connection_generation);
    REQUIRE(channel->latency.count == 1);
    REQUIRE(channel->latency.p50 == 2ms);
    REQUIRE(health.last_successful_receive == received_at);

    REQUIRE(health.command_families.size() == 1);
    const auto &family = health.command_families.front();
    REQUIRE(family.family == "angle_targets");
    REQUIRE(family.pending == 0);
    REQUIRE(family.accepted == 3);
    REQUIRE(family.dispatched == 2);
    REQUIRE(family.acknowledged == 1);
    REQUIRE(family.failed == 1);
    REQUIRE(family.superseded == 1);
    REQUIRE(family.queue.depth == 0);
    REQUIRE(family.queue.high_water == 2);
    REQUIRE(family.queue.accepted == 3);
    REQUIRE(family.queue.dropped == 1);
    REQUIRE(health.pending_commands() == 0);
    REQUIRE(health.superseded_commands() == 1);
}
TEST_CASE("comm: runtime health recording keeps bounded channel latency", "[comm][runtime][health][metrics]") {
    Rig rig;
    for (size_t i = 0; i < linkerbot::hand::kHealthLatencyWindowCapacity + 5; ++i) {
        rig.runtime->record_channel_success("angles", Clock::now(), 1us);
    }

    const auto health = rig.runtime->health();
    const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                      [](const auto &item) { return item.channel == "angles"; });
    REQUIRE(channel != health.channels.end());
    REQUIRE(channel->completed_reads == linkerbot::hand::kHealthLatencyWindowCapacity + 5);
    REQUIRE(channel->latency.count == linkerbot::hand::kHealthLatencyWindowCapacity);
    REQUIRE(channel->latency.p50 == 1us);
    REQUIRE(channel->latency.p95 == 1us);
    REQUIRE(channel->latency.p99 == 1us);
}
TEST_CASE("comm: channel latency reports exact nearest-rank percentiles", "[comm][runtime][health][metrics]") {
    Rig rig;
    for (size_t i = 1; i <= 101; ++i) {
        rig.runtime->record_channel_success("angles", Clock::now(), std::chrono::microseconds{i});
    }

    const auto health = rig.runtime->health();
    const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                      [](const auto &item) { return item.channel == "angles"; });
    REQUIRE(channel != health.channels.end());
    REQUIRE(channel->latency.count == 101);
    REQUIRE(channel->latency.p50 == 51us);
    REQUIRE(channel->latency.p95 == 96us);
    REQUIRE(channel->latency.p99 == 100us);
}
TEST_CASE("comm: recovery resets channel rate window but preserves counters", "[comm][runtime][health][metrics]") {
    auto session = Session::create([] { return std::make_unique<FakeTransport>(); });
    auto runtime = DeviceRuntime::acquire(session, "recovery-metrics/0x28", {{0x28, false}});
    const auto now = Clock::now();
    runtime->record_channel_success("angle", now - 2ms, 1ms);
    runtime->record_channel_success("angle", now - 1ms, 2ms);
    runtime->record_channel_timeout("angle");
    REQUIRE(runtime->health().channels.front().achieved_hz > 0.0);

    runtime->recover();

    const auto health = runtime->health();
    const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                      [](const auto &item) { return item.channel == "angles"; });
    REQUIRE(channel != health.channels.end());
    REQUIRE(channel->achieved_hz == 0.0);
    REQUIRE_FALSE(channel->data_age.valid);
    REQUIRE(channel->data_age.stale);
    REQUIRE(channel->completed_reads == 2);
    REQUIRE(channel->timeouts == 1);
    REQUIRE(channel->latency.count == 2);
    runtime->close();
    session->close();
}
TEST_CASE("comm: runtime health recording normalizes errors and resets their streak",
          "[comm][runtime][health][metrics]") {
    Rig rig;
    linkerbot::hand::ErrorContext context;
    context.operation = "poll.angles";
    context.channel = "angles";
    rig.runtime->record_error(std::make_exception_ptr(linkerbot::hand::TimeoutError("poll timed out")), context);

    const auto failed = rig.runtime->health();
    REQUIRE(failed.consecutive_errors == 1);
    REQUIRE(failed.recent_error.has_value());
    REQUIRE(test_support::require_value(failed.recent_error).code == linkerbot::hand::ErrorCode::kTimeout);
    REQUIRE(test_support::require_value(failed.recent_error).context.operation == "poll.angles");
    REQUIRE(test_support::require_value(failed.recent_error).context.model == "test/0x28");
    REQUIRE(test_support::require_value(failed.recent_error).context.channel == "angles");
    REQUIRE(test_support::require_value(failed.recent_error).context.generation == failed.connection_generation);

    rig.runtime->record_channel_success("angles");
    const auto recovered = rig.runtime->health();
    REQUIRE(recovered.consecutive_errors == 0);
    REQUIRE(recovered.recent_error.has_value());
    REQUIRE(test_support::require_value(recovered.recent_error).code ==
            test_support::require_value(failed.recent_error).code);
    REQUIRE(test_support::require_value(recovered.recent_error).message ==
            test_support::require_value(failed.recent_error).message);
}
TEST_CASE("comm: runtime health recording is thread safe", "[comm][runtime][health][metrics]") {
    Rig rig;
    constexpr size_t workers = 4;
    constexpr size_t iterations = 250;
    std::vector<std::future<void>> futures;
    for (size_t worker = 0; worker < workers; ++worker) {
        futures.push_back(std::async(std::launch::async, [&] {
            for (size_t i = 0; i < iterations; ++i) {
                rig.runtime->record_channel_success("angles");
                rig.runtime->record_command_accepted("angle_targets");
                rig.runtime->record_command_acknowledged("angle_targets");
            }
        }));
    }
    for (auto &future : futures) {
        future.get();
    }

    const auto health = rig.runtime->health();
    const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                      [](const auto &item) { return item.channel == "angles"; });
    REQUIRE(channel != health.channels.end());
    REQUIRE(channel->completed_reads == workers * iterations);
    REQUIRE(channel->feedback_samples == workers * iterations);
    REQUIRE(health.command_families.size() == 1);
    REQUIRE(health.command_families.front().accepted == workers * iterations);
    REQUIRE(health.command_families.front().acknowledged == workers * iterations);
    REQUIRE(health.command_families.front().pending == 0);
    REQUIRE(health.command_families.front().queue.depth == 0);
    REQUIRE(health.command_families.front().queue.high_water >= 1);
}
TEST_CASE("comm: concurrent health readers observe coherent cached snapshots", "[comm][runtime][health][metrics]") {
    Rig rig;
    constexpr size_t iterations = 2000;
    constexpr size_t reader_count = 4;
    rig.runtime->register_channel("angles");
    rig.runtime->register_command_family("angle_targets");
    std::atomic<bool> start{false};
    std::atomic<bool> finished{false};

    auto writer = std::async(std::launch::async, [&] {
        while (!start.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        for (size_t i = 0; i < iterations; ++i) {
            rig.runtime->record_channel_success("angles", Clock::now(), 1us);
            rig.runtime->record_command_accepted("angle_targets");
            rig.runtime->record_command_acknowledged("angle_targets");
            if ((i % 64) == 0) {
                std::this_thread::yield();
            }
        }
        finished.store(true, std::memory_order_release);
    });

    std::vector<std::future<size_t>> readers;
    for (size_t reader = 0; reader < reader_count; ++reader) {
        readers.push_back(std::async(std::launch::async, [&] {
            size_t reads = 0;
            uint64_t previous_samples = 0;
            while (!start.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            do {
                const auto snapshot = rig.runtime->health();
                const auto channel = std::find_if(snapshot.channels.begin(), snapshot.channels.end(),
                                                  [](const auto &item) { return item.channel == "angles"; });
                const auto family = std::find_if(snapshot.command_families.begin(), snapshot.command_families.end(),
                                                 [](const auto &item) { return item.family == "angle_targets"; });
                if (channel == snapshot.channels.end() || family == snapshot.command_families.end() ||
                    channel->completed_reads < previous_samples ||
                    channel->feedback_samples != channel->completed_reads ||
                    channel->latency.count > linkerbot::hand::kHealthLatencyWindowCapacity ||
                    family->accepted < family->acknowledged ||
                    family->pending != family->accepted - family->acknowledged ||
                    family->queue.depth != family->pending) {
                    throw std::runtime_error("incoherent health snapshot");
                }
                previous_samples = channel->completed_reads;
                ++reads;
            } while (!finished.load(std::memory_order_acquire));
            return reads;
        }));
    }

    start.store(true, std::memory_order_release);
    writer.get();
    for (auto &reader : readers) {
        REQUIRE(reader.get() > 0);
    }

    const auto health = rig.runtime->health();
    REQUIRE(health.channels.front().completed_reads == iterations);
    REQUIRE(health.command_families.front().accepted == iterations);
    REQUIRE(health.command_families.front().acknowledged == iterations);
    REQUIRE(health.command_families.front().pending == 0);
}
TEST_CASE("comm: polling automatically records rate samples and deadline misses", "[comm][runtime][health][polling]") {
    Rig rig;
    rig.fake->echo_cmd(0x28, 1, 6, 42);
    rig.runtime->start_polling({{"angle", 5ms, [&](auto token) {
                                     (void)rig.runtime->execute(plan(), Clock::now() + 200ms, token);
                                     std::this_thread::sleep_for(12ms);
                                 }}});

    const auto deadline = Clock::now() + 1s;
    linkerbot::hand::HandHealth health;
    do {
        std::this_thread::sleep_for(2ms);
        health = rig.runtime->health();
    } while (Clock::now() < deadline && (health.channels.empty() || health.channels.front().completed_reads < 3));

    const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                      [](const auto &item) { return item.channel == "angles"; });
    REQUIRE(channel != health.channels.end());
    REQUIRE(channel->supported);
    REQUIRE(channel->polling_active);
    REQUIRE(channel->requested_hz == 200.0);
    REQUIRE(channel->achieved_hz > 0.0);
    REQUIRE(channel->completed_reads >= 3);
    REQUIRE(channel->feedback_samples >= 3);
    REQUIRE(channel->missed_deadlines >= 1);
    REQUIRE(channel->latency.count >= 3);

    rig.runtime->stop_polling();
    const auto stopped = rig.runtime->health();
    const auto stopped_channel = std::find_if(stopped.channels.begin(), stopped.channels.end(),
                                              [](const auto &item) { return item.channel == "angles"; });
    REQUIRE(stopped_channel != stopped.channels.end());
    REQUIRE_FALSE(stopped_channel->polling_active);
    REQUIRE(stopped_channel->requested_hz == 0.0);
    REQUIRE(stopped_channel->achieved_hz == 0.0);
}
TEST_CASE("comm: polling timeout updates its canonical channel", "[comm][runtime][health][polling]") {
    Rig rig;
    rig.runtime->start_polling({{"angle",
                                 5ms,
                                 [&](auto token) { (void)rig.runtime->execute(plan(), Clock::now() + 15ms, token); },
                                 {},
                                 true}});

    REQUIRE(rig.runtime->wait_polling_stopped(Clock::now() + 1s));
    const auto health = rig.runtime->health();
    const auto channel = std::find_if(health.channels.begin(), health.channels.end(),
                                      [](const auto &item) { return item.channel == "angles"; });
    REQUIRE(channel != health.channels.end());
    REQUIRE(channel->timeouts == 1);
    REQUIRE_FALSE(channel->polling_active);
    REQUIRE(health.recent_error.has_value());
    REQUIRE(test_support::require_value(health.recent_error).code == linkerbot::hand::ErrorCode::kTimeout);
}
TEST_CASE("comm: an isolated poller keeps sibling rate on a shared channel", "[comm][runtime][health][polling]") {
    Rig rig;
    std::atomic<size_t> sibling_runs{0};
    rig.runtime->start_polling({
        {"force_sensor", 10ms, [](auto) { throw linkerbot::hand::TimeoutError("isolated force failure"); }, {}, true},
        {"tip_force", 20ms, [&](auto) { ++sibling_runs; }, {}, true},
    });

    const auto deadline = Clock::now() + 1s;
    linkerbot::hand::HandHealth health;
    const auto force = [&]() {
        return std::find_if(health.channels.begin(), health.channels.end(),
                            [](const auto &item) { return item.channel == "force"; });
    };
    do {
        std::this_thread::sleep_for(2ms);
        health = rig.runtime->health();
    } while (Clock::now() < deadline &&
             (sibling_runs.load() == 0 || force() == health.channels.end() || force()->requested_hz != 50.0));

    REQUIRE(health.polling_active);
    REQUIRE(sibling_runs.load() > 0);
    REQUIRE(force() != health.channels.end());
    REQUIRE(force()->polling_active);
    REQUIRE(force()->requested_hz == 50.0);
    rig.runtime->stop_polling();
}
TEST_CASE("comm: command transactions automatically update family outcomes", "[comm][runtime][health][commands]") {
    Rig rig;
    linkerbot::hand::runtime::TransactionPlan command;
    command.domain = "angle";
    command.command_family = "angle_targets";
    Message frame;
    frame.id = 0x28;
    frame.dlc = 1;
    frame.data[0] = 0x01;
    command.frames = {frame};
    rig.runtime->command(command, Clock::now() + 1s);

    auto timed = plan();
    timed.domain = "timeout_angle";
    timed.keys = {"timeout_angle"};
    timed.command_family = "angle_targets";
    REQUIRE_THROWS_AS(rig.runtime->execute(std::move(timed), Clock::now() + 20ms), linkerbot::hand::TimeoutError);

    linkerbot::hand::runtime::TransactionPlan cancelled;
    cancelled.domain = "cancelled_angle";
    cancelled.keys = {"cancelled_angle"};
    cancelled.command_family = "angle_targets";
    cancelled.frames = command.frames;
    cancelled.consume = [](const Message &) { return linkerbot::hand::protocol::Progress::unmatched(); };
    auto request = rig.runtime->submit(std::move(cancelled), Clock::now() + 1s);
    request.cancel();

    const auto health = rig.runtime->health();
    REQUIRE(health.command_families.size() == 1);
    const auto &family = health.command_families.front();
    REQUIRE(family.family == "angle_targets");
    REQUIRE(family.accepted == 3);
    REQUIRE(family.acknowledged == 1);
    REQUIRE(family.failed == 2);
    REQUIRE(family.timed_out == 1);
    REQUIRE(family.cancelled == 1);
    REQUIRE(family.pending == 0);
    REQUIRE(family.queue.accepted == 3);
}
TEST_CASE("comm: command queue counts logical jobs independently of frames", "[comm][runtime][health][commands]") {
    Rig rig;
    rig.fake->arm_tx_gate();
    struct Release {
        FakeTransport *fake;
        ~Release() { fake->release_tx_gate(); }
    } release{rig.fake};

    linkerbot::hand::runtime::TransactionPlan command;
    command.domain = "config";
    command.command_family = "configuration";
    Message frame;
    frame.id = 0x28;
    frame.dlc = 1;
    frame.data[0] = 0x01;
    command.frames = {frame, frame};
    auto first = rig.runtime->submit(command, Clock::now() + 2s);
    REQUIRE(rig.fake->wait_until_tx_entered(500ms));
    auto second = rig.runtime->submit(command, Clock::now() + 2s);
    auto third = rig.runtime->submit(command, Clock::now() + 2s);

    const auto queued = rig.runtime->health();
    REQUIRE(queued.command_families.size() == 1);
    REQUIRE(queued.command_families.front().accepted == 3);
    REQUIRE(queued.command_families.front().pending == 3);
    REQUIRE(queued.command_families.front().queue.depth == 3);
    REQUIRE(queued.command_families.front().queue.high_water == 3);
    REQUIRE(queued.session.queue_high_water >= 2);

    rig.fake->release_tx_gate();
    (void)first.wait();
    (void)second.wait();
    (void)third.wait();
    const auto completed = rig.runtime->health();
    REQUIRE(completed.command_families.front().acknowledged == 3);
    REQUIRE(completed.command_families.front().dispatched == 3);
    REQUIRE(completed.command_families.front().pending == 0);
    REQUIRE(completed.session.tx_frames >= 6);
}
TEST_CASE("comm: runtime close wakes waiter and clears pending cache", "[comm][runtime]") {
    Rig rig;
    auto request = rig.runtime->submit(plan(), Clock::now() + 30s);
    auto waiting = std::async(std::launch::async, [&] { REQUIRE_THROWS(request.wait()); });
    rig.runtime->close();
    REQUIRE(waiting.wait_for(500ms) == std::future_status::ready);
    waiting.get();
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
    REQUIRE(rig.runtime->health().lifecycle_state == linkerbot::hand::LifecycleState::kClosed);
    REQUIRE_FALSE(rig.runtime->health().connected());
    REQUIRE_THROWS(rig.runtime->submit(plan(), Clock::now() + 1s));
}
TEST_CASE("comm: same correlation key consumes the whole deadline while queued", "[comm][runtime]") {
    Rig rig;
    auto first = rig.runtime->submit(plan(), Clock::now() + 5s);
    const auto begin = Clock::now();
    REQUIRE_THROWS_AS(rig.runtime->execute(plan(), begin + 25ms), linkerbot::hand::TimeoutError);
    REQUIRE(Clock::now() - begin < 300ms);
    REQUIRE(rig.fake->sent().size() <= 1);
    first.cancel();
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
}
TEST_CASE("comm: runtime send failure leaves no pending transaction", "[comm][runtime]") {
    Rig rig;
    rig.fake->fail_writes("deterministic write failure");
    REQUIRE_THROWS(rig.runtime->execute(plan(), Clock::now() + 500ms));
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
    REQUIRE_FALSE((rig.runtime->snapshot("angle").has_value() &&
                   test_support::require_value(rig.runtime->snapshot("angle")).valid));
}
TEST_CASE("comm: cancellation of queued request prevents later backend write", "[comm][runtime]") {
    Rig rig;
    rig.fake->arm_tx_gate();
    linkerbot::transport::MessageBatch batch;
    Message command;
    command.id = 0x29;
    command.dlc = 1;
    command.data[0] = 1;
    batch.frames = {command};
    auto blocking = rig.session->submit(batch);
    REQUIRE(rig.fake->wait_until_tx_entered(500ms));
    auto request = rig.runtime->submit(plan(), Clock::now() + 1s);
    request.cancel();
    REQUIRE_THROWS_AS(request.wait(), linkerbot::hand::LifecycleError);
    rig.fake->release_tx_gate();
    blocking.wait();
    rig.session->send(batch);
    for (const auto &frame : rig.fake->sent()) {
        REQUIRE(frame.id != 0x28);
    }
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
}
TEST_CASE("comm: cancelled queued callbacks release a borrowed session", "[comm][runtime][lifecycle]") {
    // Hold the shared event executor on a different device so cancellation
    // must finish without waiting for queued callbacks to be dequeued.
    Rig blocker;
    blocker.fake->echo_cmd(0x28, 1, 6, 42);
    std::promise<void> entered, release;
    auto released = release.get_future();
    const auto blocked = blocker.runtime->subscribe("angle", [&](const auto &) {
        entered.set_value();
        released.wait();
    });
    auto fake = std::make_unique<FakeTransport>();
    fake->echo_cmd(0x28, 1, 6, 42);
    linkerbot::can::CANDispatcher dispatcher(std::move(fake));
    struct Release {
        std::promise<void> &promise;
        bool done = false;
        void open() {
            if (!done) {
                done = true;
                promise.set_value();
            }
        }
        ~Release() { open(); }
    } gate{release};

    (void)blocker.runtime->execute(plan(), Clock::now() + 1s);
    REQUIRE(entered.get_future().wait_for(1s) == std::future_status::ready);
    std::weak_ptr<Session> weak_session;
    std::weak_ptr<DeviceRuntime> weak_runtime;
    std::atomic<unsigned> callbacks{0};
    {
        auto session = Session::borrow(dispatcher);
        auto core = DeviceRuntime::acquire(session, "queued/borrowed", {{0x28, false}});
        weak_session = session;
        weak_runtime = core;
        core->subscribe("angle", [&](const auto &) { ++callbacks; });
        (void)core->execute(plan(), Clock::now() + 1s);
        core->close();
    }
    const bool runtime_released = weak_runtime.expired();
    const bool session_released = weak_session.expired();

    // Keep dispatcher alive while the old implementation drains its retained
    // session, so the failing regression reports ownership rather than UB.
    gate.open();
    blocker.runtime->unsubscribe(blocked);
    const auto deadline = Clock::now() + 1s;
    while (!weak_session.expired() && Clock::now() < deadline) {
        std::this_thread::yield();
    }
    REQUIRE(weak_session.expired());
    REQUIRE(runtime_released);
    REQUIRE(session_released);
    REQUIRE(callbacks == 0);
    REQUIRE(dispatcher.get_health().connected());
}

TEST_CASE("comm: an executing callback can release the final borrowed runtime", "[comm][runtime][lifecycle]") {
    Rig sentinel;
    sentinel.fake->echo_cmd(0x28, 1, 6, 42);
    std::promise<void> drained;
    const auto sentinel_id = sentinel.runtime->subscribe("angle", [&](const auto &) { drained.set_value(); });

    auto fake = std::make_unique<FakeTransport>();
    fake->echo_cmd(0x28, 1, 6, 42);
    linkerbot::can::CANDispatcher dispatcher(std::move(fake));
    std::promise<void> release, returned;
    auto released = release.get_future();
    auto session = Session::borrow(dispatcher);
    auto core = DeviceRuntime::acquire(session, "executing/borrowed", {{0x28, false}});
    const std::weak_ptr<DeviceRuntime> weak_runtime = core;
    const std::weak_ptr<Session> weak_session = session;
    core->subscribe("angle", [&](const auto &) {
        released.wait();
        core.reset();
        session.reset();
        returned.set_value();
    });

    // The synchronous call must finish before its owning references are reset
    // by the callback. Its executor reference then supplies the final lifetime.
    try {
        (void)core->execute(plan(), Clock::now() + 1s);
    } catch (...) {
        release.set_value();
        throw;
    }
    release.set_value();
    REQUIRE(returned.get_future().wait_for(1s) == std::future_status::ready);

    // This callback runs after the previous callback's executor has released
    // its owner, not merely after the previous callback body returned.
    (void)sentinel.runtime->execute(plan(), Clock::now() + 1s);
    REQUIRE(drained.get_future().wait_for(1s) == std::future_status::ready);
    sentinel.runtime->unsubscribe(sentinel_id);
    REQUIRE(weak_runtime.expired());
    REQUIRE(weak_session.expired());
    REQUIRE(dispatcher.get_health().connected());
}

TEST_CASE("comm: slow callback does not block bus or other transaction", "[comm][runtime]") {
    Rig rig;
    rig.fake->echo_cmd(0x28, 1, 6, 42);
    std::mutex mutex;
    std::condition_variable cv;
    bool entered = false;
    bool release = false;
    const auto subscription = rig.runtime->subscribe("angle", [&](const auto &) {
        std::unique_lock<std::mutex> lock(mutex);
        entered = true;
        cv.notify_all();
        cv.wait(lock, [&] { return release; });
    });
    REQUIRE_NOTHROW(rig.runtime->execute(plan(), Clock::now() + 500ms));
    bool observed = false;
    {
        std::unique_lock<std::mutex> lock(mutex);
        observed = cv.wait_for(lock, 500ms, [&] { return entered; });
    }
    auto request = rig.runtime->submit(plan(), Clock::now() + 500ms);
    const bool completed = [&] {
        try {
            (void)request.wait();
            return true;
        } catch (...) {
            return false;
        }
    }();
    {
        std::lock_guard<std::mutex> lock(mutex);
        release = true;
        cv.notify_all();
    }
    rig.runtime->unsubscribe(subscription);
    REQUIRE(observed);
    REQUIRE(completed);
}
TEST_CASE("comm: polling stop is interruptible and invalid plans rejected", "[comm][runtime]") {
    Rig rig;
    std::promise<void> ran;
    rig.runtime->start_polling({{"long", 30s, [&](auto) { ran.set_value(); }}});
    REQUIRE(ran.get_future().wait_for(500ms) == std::future_status::ready);
    auto before = Clock::now();
    rig.runtime->stop_polling();
    REQUIRE(Clock::now() - before < 300ms);
    REQUIRE_THROWS(rig.runtime->start_polling({{"zero", 0us, [](auto) {}}}));
    REQUIRE_THROWS(rig.runtime->start_polling({{"duplicate", 1s, [](auto) {}}, {"duplicate", 1s, [](auto) {}}}));
}

TEST_CASE("comm: callback lifecycle calls reject before an external close barrier", "[comm][runtime][lifecycle]") {
    Rig rig;
    rig.fake->echo_cmd(0x28, 1, 6, 42);
    std::promise<void> entered, try_close;
    auto release = try_close.get_future();
    std::promise<bool> rejected;
    rig.runtime->subscribe("angle", [&](const auto &) {
        entered.set_value();
        release.wait();
        try {
            rig.runtime->close();
            rejected.set_value(false);
        } catch (const linkerbot::hand::LifecycleError &) {
            rejected.set_value(true);
        }
    });
    (void)rig.runtime->execute(plan(), Clock::now() + 1s);
    REQUIRE(entered.get_future().wait_for(1s) == std::future_status::ready);
    auto closing = std::async(std::launch::async, [&] { rig.runtime->close(); });
    // The callback remains in flight while the external closer begins.
    try_close.set_value();
    REQUIRE(rejected.get_future().get());
    REQUIRE(closing.wait_for(1s) == std::future_status::ready);
    closing.get();
}

TEST_CASE("comm: concurrent callback unsubscribe callers share the execution barrier", "[comm][runtime][lifecycle]") {
    Rig rig;
    rig.fake->echo_cmd(0x28, 1, 6, 42);
    std::promise<void> entered, release;
    auto released = release.get_future();
    const auto id = rig.runtime->subscribe("angle", [&](const auto &) {
        entered.set_value();
        released.wait();
    });
    (void)rig.runtime->execute(plan(), Clock::now() + 1s);
    entered.get_future().wait();
    auto first = std::async(std::launch::async, [&] { rig.runtime->unsubscribe(id); });
    auto second = std::async(std::launch::async, [&] { rig.runtime->unsubscribe(id); });
    const auto a = first.wait_for(10ms);
    const auto b = second.wait_for(10ms);
    release.set_value();
    first.get();
    second.get();
    REQUIRE(a == std::future_status::timeout);
    REQUIRE(b == std::future_status::timeout);
}

TEST_CASE("comm: stopping polling interrupts admission behind a semantic operation", "[comm][runtime][lifecycle]") {
    Rig rig;
    auto operation = rig.runtime->operation(Clock::now() + 30s);
    std::promise<void> entered;
    rig.runtime->start_polling({{"blocked", 1s, [&](auto) {
                                     entered.set_value();
                                     auto nested = rig.runtime->operation(Clock::now() + 30s);
                                 }}});
    entered.get_future().wait();
    const auto begin = Clock::now();
    rig.runtime->stop_polling();
    REQUIRE(Clock::now() - begin < 1s);
    REQUIRE(rig.runtime->runtime_health().state == linkerbot::hand::runtime::DeviceState::ready);
}

TEST_CASE("comm: shared physical recovery invalidates both devices and reopens once", "[comm][runtime][lifecycle]") {
    size_t opens = 0;
    auto session = Session::create([&] {
        ++opens;
        auto fake = std::make_unique<FakeTransport>();
        fake->echo_cmd(0x28, 1, 6, 42);
        fake->echo_cmd(0x29, 1, 6, 51);
        return fake;
    });
    auto first = DeviceRuntime::acquire(session, "first", {{0x28, false}});
    auto second = DeviceRuntime::acquire(session, "second", {{0x29, false}});
    (void)first->execute(plan(), Clock::now() + 1s);
    auto other_plan = linkerbot::hand::o6::protocol::read_angle(0x29);
    (void)second->execute(other_plan, Clock::now() + 1s);
    first->recover();
    second->recover();
    REQUIRE(opens == 2);
    REQUIRE(first->runtime_health().physical_generation == second->runtime_health().physical_generation);
    REQUIRE(first->health().connection_generation == 2);
    REQUIRE(first->health().reconnect_count == 1);
    REQUIRE(first->health().connected());
    REQUIRE_FALSE(test_support::require_value(second->snapshot("angle")).valid);
    (void)second->execute(other_plan, Clock::now() + 1s);
    first->close();
    (void)second->execute(other_plan, Clock::now() + 1s);
    second->close();
    session->close();
}

TEST_CASE("comm: named composites preserve unrelated progress and bound nested requests", "[comm][runtime]") {
    Rig rig;
    rig.fake->echo_cmd(0x28, 1, 6, 42);
    const auto deadline = Clock::now() + 100ms;
    auto batch = rig.runtime->operation("tactile.batch", deadline);
    auto parallel =
        std::async(std::launch::async, [&] { return rig.runtime->execute(plan(), Clock::now() + 500ms).valid; });
    REQUIRE(parallel.get());
    auto competitor = std::async(std::launch::async, [&] {
        try {
            auto op = rig.runtime->operation("tactile.batch", Clock::now() + 10ms);
            return false;
        } catch (const linkerbot::hand::TimeoutError &) {
            return true;
        }
    });
    REQUIRE(competitor.get());
    auto exclusive = std::async(std::launch::async, [&] {
        try {
            auto op = rig.runtime->operation(Clock::now() + 10ms);
            return false;
        } catch (const linkerbot::hand::TimeoutError &) {
            return true;
        }
    });
    REQUIRE(exclusive.get());
    rig.fake->on_request(0x28, [](const Message &) { return std::vector<Message>{}; });
    const auto began = Clock::now();
    REQUIRE_THROWS_AS(rig.runtime->execute(plan(), began + 5s), linkerbot::hand::TimeoutError);
    REQUIRE(Clock::now() - began < 500ms);
}

TEST_CASE("comm: completed send-only requests retain their send receipt", "[comm][runtime]") {
    std::atomic<unsigned> observed{0};
    Rig rig;
    for (unsigned index = 0; index < 100; ++index) {
        auto send_only = plan();
        send_only.consume = {};
        send_only.optional_response_window.reset();
        linkerbot::hand::unified_detail::RuntimeAccess::execute_command(
            *rig.runtime, Clock::now() + 2s, true, [&](const auto &, bool) { ++observed; },
            [](const auto &) { return true; }, {},
            [&] {
                auto request = rig.runtime->submit(send_only, Clock::now() + 2s);
                (void)request.wait_sent_until(Clock::now() + 1s);
                REQUIRE(observed == index + 1);
                // Terminal completion must also retain the physical receipt.
                (void)request.wait_until(Clock::now() + 1s);
                const auto sent = request.wait_sent_until(Clock::now() + 1s);
                REQUIRE(sent.outcome == linkerbot::transport::SendOutcome::dispatched);
                REQUIRE(sent.sent_frames == 1);
                REQUIRE_FALSE(sent.error);
            });
    }
    REQUIRE(rig.runtime->runtime_health().sent_frames == 100);
}

TEST_CASE("comm: optional feedback waits after receipt and never claims an ACK", "[comm][runtime]") {
    Rig rig;
    auto optional = plan();
    optional.optional_response_window = 10ms;
    optional.poison_on_timeout = false;
    optional.late_guard = 2ms;
    rig.fake->arm_tx_gate();
    auto request = rig.runtime->submit(optional, Clock::now() + 1s);
    REQUIRE(rig.fake->wait_until_tx_entered(500ms));
    const auto before = rig.runtime->runtime_health().sent_frames;
    const bool completed_before_receipt = request.completed();
    rig.fake->release_tx_gate();
    const auto sample = request.wait();
    REQUIRE(before == 0);
    REQUIRE_FALSE(completed_before_receipt);
    REQUIRE_FALSE(sample.value.has_value());
    REQUIRE_FALSE(sample.valid);
    REQUIRE(rig.runtime->runtime_health().sent_frames == 1);
    REQUIRE(rig.runtime->sent_frames("angle") == 1);
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
    auto bounded = optional;
    bounded.optional_response_window = 10s;
    REQUIRE_THROWS_AS(rig.runtime->execute(bounded, Clock::now() + 20ms), linkerbot::hand::TimeoutError);
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
}

TEST_CASE("comm: optional feedback received after send acceptance updates the cache", "[comm][runtime]") {
    Rig rig;
    auto optional = plan();
    // Observation by the test thread is independent of the production 5ms
    // best-effort window. Keep this transaction open until its injected reply.
    optional.optional_response_window = 1s;
    optional.poison_on_timeout = false;
    auto request = rig.runtime->submit(optional, Clock::now() + 2s);
    const auto receipt = request.wait_sent_until(Clock::now() + 1s);
    REQUIRE(receipt.sent_frames == 1);
    REQUIRE_FALSE(request.completed());
    Message reply;
    reply.id = 0x28;
    reply.dlc = 7;
    reply.data = {1, 0, 51, 102, 153, 204, 255};
    rig.fake->inject(reply);
    const auto sample = request.wait();
    REQUIRE(sample.valid);
    const auto cached = rig.runtime->snapshot(optional.domain);
    REQUIRE(cached.has_value());
    REQUIRE(test_support::require_value(cached).valid);
    REQUIRE(test_support::require_value(cached).sequence == sample.sequence);
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
}

TEST_CASE("comm: dynamic polling waits and nested waits stop through runtime cancellation",
          "[comm][runtime][lifecycle]") {
    Rig rig;
    std::atomic<bool> due{false};
    std::promise<void> entered;
    rig.runtime->start_polling(
        {{"policy", 1us,
          [&](auto) {
              entered.set_value();
              rig.runtime->wait_until(Clock::now() + 30s);
          },
          [&]() { return std::optional<Clock::time_point>(due.load() ? Clock::now() : Clock::now() + 30s); }}});
    due = true;
    rig.runtime->wake_polling();
    REQUIRE(entered.get_future().wait_for(1s) == std::future_status::ready);
    const auto began = Clock::now();
    rig.runtime->stop_polling();
    REQUIRE(Clock::now() - began < 500ms);
    REQUIRE(rig.runtime->runtime_health().state == linkerbot::hand::runtime::DeviceState::ready);
}

TEST_CASE("comm: runtime close drains a claimed backend write before returning", "[comm][runtime][lifecycle]") {
    Rig rig;
    rig.fake->arm_tx_gate();
    auto request = rig.runtime->submit(plan(), Clock::now() + 5s);
    REQUIRE(rig.fake->wait_until_tx_entered(500ms));
    auto closing = std::async(std::launch::async, [&] { rig.runtime->close(); });
    bool cancelled = false;
    std::exception_ptr unexpected;
    try {
        (void)request.wait_until(Clock::now() + 500ms);
    } catch (const linkerbot::hand::LifecycleError &) {
        cancelled = true;
    } catch (...) {
        unexpected = std::current_exception();
    }
    const bool waiting_for_backend = closing.wait_for(10ms) == std::future_status::timeout;
    rig.fake->release_tx_gate();
    closing.get();
    if (unexpected) {
        std::rethrow_exception(unexpected);
    }
    REQUIRE(cancelled);
    REQUIRE(waiting_for_backend);
    REQUIRE(rig.runtime->runtime_health().state == linkerbot::hand::runtime::DeviceState::closed);
    REQUIRE(rig.runtime->runtime_health().sent_frames == 1);
}

TEST_CASE("comm: pre-recovery composite cannot resume with a new generation", "[comm][runtime][lifecycle]") {
    auto session = Session::create([] {
        auto fake = std::make_unique<FakeTransport>();
        fake->echo_cmd(0x28, 1, 6, 42);
        return fake;
    });
    auto core = DeviceRuntime::acquire(session, "old-operation", {{0x28, false}});
    for (const bool named : {false, true}) {
        {
            auto operation =
                named ? core->operation("composite", Clock::now() + 5s) : core->operation(Clock::now() + 5s);
            auto recovering = std::async(std::launch::async, [&] { core->recover(); });
            recovering.get();
            REQUIRE_THROWS_AS(core->execute(plan(), Clock::now() + 1s), linkerbot::hand::ClosedError);
        }
        REQUIRE(core->execute(plan(), Clock::now() + 1s).valid);
    }
    core->close();
    session->close();
}

TEST_CASE("comm: a shorter request wait deadline is a timeout with send uncertainty exposed", "[comm][runtime]") {
    Rig rig;
    rig.fake->arm_tx_gate();
    auto request = rig.runtime->submit(plan(), Clock::now() + 5s);
    REQUIRE(rig.fake->wait_until_tx_entered(500ms));
    bool timed_out = false;
    try {
        (void)request.wait_until(Clock::now() + 10ms);
    } catch (const linkerbot::hand::TimeoutError &) {
        timed_out = true;
    }
    const auto receipt = request.send_result();
    const auto health = rig.runtime->runtime_health();
    rig.fake->release_tx_gate();
    REQUIRE(timed_out);
    REQUIRE(receipt.has_value());
    REQUIRE(test_support::require_value(receipt).claimed_at.has_value());
    REQUIRE(test_support::require_value(receipt).outcome == linkerbot::transport::SendOutcome::pending);
    REQUIRE(health.timeouts == 1);
    REQUIRE(health.cancelled == 0);
    REQUIRE(health.in_flight == 0);
}

TEST_CASE("comm: timeout preserves claimed IO warning whichever thread expires the request",
          "[comm][runtime][regression]") {
    bool worker_expires = false;
    bool wait_for_send = false;
    SECTION("worker expires before response wait") { worker_expires = true; }
    SECTION("worker expires before send wait") {
        worker_expires = true;
        wait_for_send = true;
    }
    SECTION("caller expires response wait") {}
    SECTION("caller expires send wait") { wait_for_send = true; }

    Rig rig;
    rig.fake->arm_tx_gate();
    // Release even if an assertion fails, so Rig can drain the pending send.
    struct ReleaseGate {
        FakeTransport *fake;
        ~ReleaseGate() { fake->release_tx_gate(); }
    } release{rig.fake};
    auto request = rig.runtime->submit(plan(), Clock::now() + (worker_expires ? 200ms : 5s));
    REQUIRE(rig.fake->wait_until_tx_entered(1s));
    if (worker_expires) {
        // No wait API participates until the receive worker has stored its error.
        const auto limit = Clock::now() + 2s;
        while (!request.completed() && Clock::now() < limit) {
            std::this_thread::sleep_for(1ms);
        }
        REQUIRE(request.completed());
    }
    std::string error;
    try {
        if (wait_for_send) {
            (void)request.wait_sent_until(Clock::now() + 10ms);
        } else {
            (void)request.wait_until(Clock::now() + 10ms);
        }
    } catch (const linkerbot::hand::TimeoutError &e) {
        error = e.what();
    }
    INFO(error);
    REQUIRE(error.find("claimed backend I/O") != std::string::npos);
    // Subsequent waiters and health diagnostics must retain the same warning.
    REQUIRE_THROWS_WITH(request.wait(), Catch::Matchers::ContainsSubstring("claimed backend I/O"));
    REQUIRE(rig.runtime->runtime_health().last_error.find("claimed backend I/O") != std::string::npos);
    REQUIRE(rig.runtime->runtime_health().timeouts == 1);
    REQUIRE(rig.runtime->runtime_health().cancelled == 0);
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
    const auto receipt = request.send_result();
    REQUIRE(receipt.has_value());
    REQUIRE(test_support::require_value(receipt).claimed_at.has_value());
    REQUIRE(test_support::require_value(receipt).outcome == linkerbot::transport::SendOutcome::pending);
}

namespace {
    struct ReportProbe {
        std::mutex mutex;
        std::condition_variable cv;
        unsigned builds = 0;
        unsigned consumed = 0;
        bool blocked = false;
        bool release = false;
        DeviceRuntime::DecoderFactory factory(bool gate = false) {
            return [this, gate] {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    ++builds;
                    cv.notify_all();
                }
                return [this, gate, first = std::optional<int>{}](const Message &m) mutable {
                    using Progress = linkerbot::hand::protocol::Progress;
                    std::unique_lock<std::mutex> lock(mutex);
                    ++consumed;
                    cv.notify_all();
                    if (gate && m.data[0] == 0x7f) {
                        blocked = true;
                        cv.notify_all();
                        cv.wait(lock, [&] { return release; });
                        return Progress::pending();
                    }
                    if (m.data[0] == 0xee) {
                        auto result = Progress::pending();
                        result.kind = linkerbot::hand::protocol::ProgressKind::failed;
                        result.conflicting_duplicates = 1;
                        return result;
                    }
                    if (m.data[0] == 1) {
                        first = m.data[1];
                    }
                    if (m.data[0] == 2 && first) {
                        auto value = *first * 256 + m.data[1];
                        first.reset();
                        return Progress::complete(value, linkerbot::hand::protocol::Origin::report);
                    }
                    return Progress::pending();
                };
            };
        }
        bool wait(unsigned count, unsigned generations = 1) {
            std::unique_lock<std::mutex> lock(mutex);
            return cv.wait_for(lock, 1s, [&] { return consumed >= count && builds >= generations; });
        }
        void unblock() {
            std::lock_guard<std::mutex> lock(mutex);
            release = true;
            cv.notify_all();
        }
    };
    struct FragmentArguments {
        uint8_t part;
        uint8_t value = 0;
    };
    Message fragment(const FragmentArguments &arguments) {
        Message m;
        m.id = 0x28;
        m.dlc = 2;
        m.data[0] = arguments.part;
        m.data[1] = arguments.value;
        return m;
    }
}
TEST_CASE("comm: report factories discard partial data across physical recovery", "[comm][runtime][lifecycle]") {
    ReportProbe probe;
    FakeTransport *fake = nullptr;
    auto session = Session::create([&] {
        auto transport = std::make_unique<FakeTransport>();
        fake = transport.get();
        return transport;
    });
    auto core = DeviceRuntime::acquire(session, "reports", {{0x28, false}});
    core->observe_factory("reports", probe.factory());
    fake->inject(fragment({1, 9}));
    REQUIRE(probe.wait(1));
    core->recover();
    fake->inject(fragment({2, 7}));
    REQUIRE(probe.wait(2, 2));
    REQUIRE_FALSE(core->snapshot("reports").has_value());
    fake->inject(fragment({1, 3}));
    fake->inject(fragment({2, 4}));
    REQUIRE(probe.wait(4, 2));
    REQUIRE(core->snapshot_as<int>("reports") == 0x304);
    REQUIRE(test_support::require_value(core->snapshot("reports")).device_generation == 2);
    core->close();
    session->close();
}
TEST_CASE("comm: failed report resets assembler and exposes conflict health", "[comm][runtime]") {
    ReportProbe probe;
    Rig rig;
    rig.runtime->observe_factory("reports", probe.factory());
    rig.fake->inject(fragment({1, 9}));
    rig.fake->inject(fragment({0xee}));
    REQUIRE(probe.wait(2, 2));
    rig.fake->inject(fragment({2, 7}));
    REQUIRE(probe.wait(3, 2));
    REQUIRE_FALSE(test_support::require_value(rig.runtime->snapshot("reports")).valid);
    REQUIRE(rig.runtime->runtime_health().conflicting_duplicate_frames == 1);
    REQUIRE(rig.runtime->runtime_health().protocol_errors == 1);
    rig.fake->inject(fragment({1, 3}));
    rig.fake->inject(fragment({2, 4}));
    REQUIRE(probe.wait(5, 2));
    REQUIRE(rig.runtime->snapshot_as<int>("reports") == 0x304);
}
TEST_CASE("comm: report assembly is reset after bounded inbox overflow", "[comm][runtime]") {
    ReportProbe probe;
    auto transport = std::make_unique<FakeTransport>();
    auto *fake = transport.get();
    auto session = Session::own(std::move(transport));
    auto core = DeviceRuntime::acquire(session, "reports", {{0x28, false}}, 1);
    core->observe_factory("reports", probe.factory(true));
    fake->inject(fragment({1, 9}));
    REQUIRE(probe.wait(1));
    fake->inject(fragment({0x7f}));
    bool entered;
    {
        std::unique_lock<std::mutex> lock(probe.mutex);
        entered = probe.cv.wait_for(lock, 1s, [&] { return probe.blocked; });
    }
    // Always release the decoder before assertions/unwinding.
    fake->inject(fragment({2, 7}));
    fake->inject(fragment({2, 8}));
    const auto limit = Clock::now() + 1s;
    while (!session->health().overflows && Clock::now() < limit) {
        std::this_thread::yield();
    }
    const auto overflow = session->health().overflows;
    probe.unblock();
    REQUIRE(entered);
    REQUIRE(overflow > 0);
    REQUIRE(probe.wait(2, 2));
    (void)core->runtime_health(); // overflow reset and queued-fragment drain have completed
    fake->inject(fragment({2, 7}));
    REQUIRE(probe.wait(3, 2));
    REQUIRE_FALSE(core->snapshot_as<int>("reports").has_value());
    fake->inject(fragment({1, 3}));
    REQUIRE(probe.wait(4, 2));
    fake->inject(fragment({2, 4}));
    REQUIRE(probe.wait(5, 2));
    REQUIRE(core->snapshot_as<int>("reports") == 0x304);
    core->close();
    session->close();
}

TEST_CASE("comm: facade sink replacement retains self cancellation barrier", "[comm][runtime][lifecycle]") {
    Rig rig;
    rig.fake->echo_cmd(0x28, 1, 6, 42);
    linkerbot::hand::runtime::Cache<int> cache(rig.runtime, "projected");
    std::promise<void> entered, release;
    auto released = release.get_future();
    cache.set_sink([&](int) {
        cache.set_sink({});
        entered.set_value();
        released.wait();
    });
    (void)rig.runtime->execute(plan(), Clock::now() + 1s);
    cache.push(1);
    entered.get_future().wait();
    auto first = std::async(std::launch::async, [&] { cache.set_sink({}); });
    auto second = std::async(std::launch::async, [&] { cache.set_sink({}); });
    const auto a = first.wait_for(20ms);
    const auto b = second.wait_for(20ms);
    release.set_value();
    first.get();
    second.get();
    REQUIRE(a == std::future_status::timeout);
    REQUIRE(b == std::future_status::timeout);
}

TEST_CASE("comm: report factory reset failure faults runtime instead of using stale state", "[comm][runtime]") {
    Rig rig;
    unsigned generations = 0;
    rig.runtime->observe_factory("reports", [&]() {
        if (++generations == 2) {
            throw std::runtime_error("injected assembler allocation failure");
        }
        return DeviceRuntime::Decoder([](const Message &) { return linkerbot::hand::protocol::Progress::complete(1); });
    });
    rig.fake->inject(fragment({1}));
    REQUIRE(rig.runtime->wait_for_fault(Clock::now() + 1s));
    REQUIRE(rig.runtime->runtime_health().state == linkerbot::hand::runtime::DeviceState::faulted);
    REQUIRE_FALSE(test_support::require_value(rig.runtime->snapshot("reports")).valid);
    REQUIRE_THROWS(rig.runtime->submit(plan(), Clock::now() + 1s));
}

TEST_CASE("comm: recovery waits for active callbacks and discards queued old samples", "[comm][runtime][lifecycle]") {
    auto session = Session::create([] {
        auto fake = std::make_unique<FakeTransport>();
        fake->echo_cmd(0x28, 1, 6, 42);
        return fake;
    });
    auto core = DeviceRuntime::acquire(session, "recover-events", {{0x28, false}});
    std::promise<void> entered, release, next_generation;
    auto released = release.get_future();
    std::atomic<unsigned> called{0};
    core->subscribe("angle", [&](const auto &sample) {
        ++called;
        if (sample.device_generation == 1) {
            entered.set_value();
            released.wait();
        } else {
            next_generation.set_value();
        }
    });
    (void)core->execute(plan(), Clock::now() + 1s);
    entered.get_future().wait();
    (void)core->execute(plan(), Clock::now() + 1s); // queued behind blocked handler
    auto recovery = std::async(std::launch::async, [&] { core->recover(); });
    const auto state = recovery.wait_for(20ms);
    release.set_value();
    recovery.get();
    REQUIRE(state == std::future_status::timeout);
    REQUIRE_FALSE(test_support::require_value(core->snapshot("angle")).valid);
    (void)core->execute(plan(), Clock::now() + 1s);
    REQUIRE(next_generation.get_future().wait_for(1s) == std::future_status::ready);
    REQUIRE(called == 2);
    core->close();
    session->close();
}

TEST_CASE("comm: request-only API waits for actual send but leaves response pending", "[comm][runtime]") {
    Rig rig;
    rig.fake->arm_tx_gate();
    auto request = rig.runtime->submit(plan(), Clock::now() + 2s);
    REQUIRE(rig.fake->wait_until_tx_entered(500ms));
    auto sent = std::async(std::launch::async, [&] { return request.wait_sent_until(Clock::now() + 1s); });
    const auto before_release = sent.wait_for(20ms);
    rig.fake->release_tx_gate();
    const auto receipt = sent.get();
    REQUIRE(before_release == std::future_status::timeout);
    REQUIRE(receipt.outcome == linkerbot::transport::SendOutcome::dispatched);
    REQUIRE(receipt.sent_frames == 1);
    REQUIRE_FALSE(request.completed());
    request.cancel();
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
}

TEST_CASE("comm: request-only send wait reports backend errors and cancels on deadline", "[comm][runtime]") {
    Rig rig;
    SECTION("backend failure") {
        rig.fake->fail_writes("injected send rejection");
        auto request = rig.runtime->submit(plan(), Clock::now() + 1s);
        REQUIRE_THROWS(request.wait_sent_until(Clock::now() + 1s));
        REQUIRE(rig.runtime->runtime_health().in_flight == 0);
    }
    SECTION("shorter send deadline") {
        rig.fake->arm_tx_gate();
        auto request = rig.runtime->submit(plan(), Clock::now() + 1s);
        REQUIRE(rig.fake->wait_until_tx_entered(500ms));
        bool timed_out = false;
        try {
            (void)request.wait_sent_until(Clock::now() + 20ms);
        } catch (const linkerbot::hand::TimeoutError &) {
            timed_out = true;
        }
        rig.fake->release_tx_gate();
        REQUIRE(timed_out);
        REQUIRE(rig.runtime->runtime_health().in_flight == 0);
        REQUIRE(rig.runtime->runtime_health().timeouts == 1);
        REQUIRE(rig.runtime->runtime_health().cancelled == 0);
    }
}

TEST_CASE("comm: device registry rejects incompatible bindings and stale route reuse", "[comm][runtime]") {
    Rig rig;
    REQUIRE(DeviceRuntime::acquire(rig.session, "test/0x28", {{0x28, false}}) == rig.runtime);
    REQUIRE_THROWS_AS(DeviceRuntime::acquire(rig.session, "test/0x28", {{0x29, false}}),
                      linkerbot::hand::ValidationError);
    REQUIRE_THROWS_AS(DeviceRuntime::acquire(rig.session, "test/0x28", {{0x28, false}}, 1),
                      linkerbot::hand::ValidationError);
    REQUIRE_THROWS_AS(DeviceRuntime::acquire(rig.session, "test/0x28", {{0x28, false}, {0x28, false}}),
                      linkerbot::hand::ValidationError);
    rig.runtime->set_routes({{0x29, false}});
    REQUIRE_THROWS_AS(DeviceRuntime::acquire(rig.session, "test/0x28", {{0x28, false}}),
                      linkerbot::hand::ValidationError);
    REQUIRE(DeviceRuntime::acquire(rig.session, "test/0x28", {{0x29, false}}) == rig.runtime);
    REQUIRE_THROWS(DeviceRuntime::acquire(rig.session, "other", {{0x29, false}}));
}

TEST_CASE("comm: deferred batch publication retains the last success on send failure", "[comm][runtime][regression]") {
    Rig rig;
    rig.fake->echo_cmd(0x28, 1, 6, 42);
    const auto previous = rig.runtime->execute(plan(), Clock::now() + 1s);
    rig.fake->arm_tx_gate();
    struct Release {
        FakeTransport *fake;
        ~Release() { fake->release_tx_gate(); }
    } release{rig.fake};
    auto batch = plan();
    batch.preserve_last_success = true;
    batch.publish_after_send = true;
    auto request = rig.runtime->submit(std::move(batch), Clock::now() + 1s);
    REQUIRE(rig.fake->wait_until_tx_entered(500ms));
    std::promise<void> parsed;
    const auto observer = rig.runtime->observe("marker", [&](const Message &m) {
        if (m.data[0] == 0x7f) {
            parsed.set_value();
        }
        return linkerbot::hand::protocol::Progress::unmatched();
    });
    Message reply;
    reply.id = 0x28;
    reply.dlc = 7;
    reply.data[0] = 1;
    reply.data[1] = 99;
    rig.fake->inject(reply);
    reply.data[0] = 0x7f;
    rig.fake->inject(reply);
    REQUIRE(parsed.get_future().wait_for(500ms) == std::future_status::ready);
    REQUIRE_FALSE(request.completed());
    REQUIRE(test_support::require_value(rig.runtime->snapshot("angle")).sequence == previous.sequence);
    rig.fake->fail_writes("failed after early reply");
    rig.fake->release_tx_gate();
    REQUIRE_THROWS(request.wait());
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
    const auto cached = rig.runtime->snapshot("angle");
    REQUIRE(test_support::require_value(cached).valid);
    REQUIRE(test_support::require_value(cached).sequence == previous.sequence);
    rig.runtime->unobserve(observer);
}

TEST_CASE("comm: optional control supersession isolates reads while permitting further targets",
          "[comm][runtime][l20]") {
    FakeTransport *fake = nullptr;
    auto session = Session::create([&] {
        auto transport = std::make_unique<FakeTransport>();
        fake = transport.get();
        return transport;
    });
    auto core = DeviceRuntime::acquire(session, "optional-controls", {{0x28, false}});
    auto control = [] {
        auto value = plan();
        value.command_family = "angle_targets";
        value.optional_response_window = 5s;
        value.supersede_optional_feedback = true;
        value.publish_after_send = true;
        return value;
    };
    auto first = core->submit(control(), Clock::now() + 2s);
    first.wait_sent_until(Clock::now() + 1s);
    REQUIRE(core->runtime_health().in_flight == 1);
    auto second = core->submit(control(), Clock::now() + 2s);
    second.wait_until(Clock::now() + 1s);
    REQUIRE(first.completed());
    REQUIRE(fake->sent().size() == 2);
    REQUIRE(core->runtime_health().in_flight == 0);
    const auto superseded = core->health().command_families.front();
    REQUIRE(superseded.accepted == 2);
    REQUIRE(superseded.acknowledged == 1);
    REQUIRE(superseded.superseded == 1);
    REQUIRE(superseded.queue.dropped == 1);
    REQUIRE(superseded.pending == 0);
    REQUIRE_THROWS(core->execute(plan(), Clock::now() + 20ms));
    REQUIRE(fake->sent().size() == 2);
    core->command(control(), Clock::now() + 1s);
    REQUIRE(fake->sent().size() == 3);
    Message late;
    late.id = 0x28;
    late.dlc = 7;
    late.data[0] = 1;
    fake->inject(late);
    core->recover();
    fake->echo_cmd(0x28, 1, 6, 42);
    REQUIRE_NOTHROW(core->execute(plan(), Clock::now() + 500ms));
}

TEST_CASE("comm: optional control cannot supersede an ordinary query", "[comm][runtime][l20]") {
    Rig rig;
    auto active = rig.runtime->submit(plan(), Clock::now() + 2s);
    active.wait_sent_until(Clock::now() + 1s);
    auto control = plan();
    control.optional_response_window = 200ms;
    control.supersede_optional_feedback = true;
    REQUIRE_THROWS_AS(rig.runtime->submit(control, Clock::now() + 20ms), linkerbot::hand::TimeoutError);
    REQUIRE_FALSE(active.completed());
    REQUIRE(rig.fake->sent().size() == 1);
    active.cancel();
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
}

TEST_CASE("comm: runtime nested semantic operations expose the inherited absolute deadline",
          "[comm][runtime][operation-deadline]") {
    Rig rig;
    const auto deadline = Clock::now() + 500ms;
    auto outer = rig.runtime->operation(deadline);
    auto ordinary = rig.runtime->operation("ordinary", Clock::now() + 5s);
    REQUIRE(outer.deadline() == deadline);
    REQUIRE(ordinary.deadline() == deadline);
    auto nested = rig.runtime->operation(Clock::now() + 10s);
    REQUIRE(nested.deadline() == deadline);
    auto moved = std::move(nested);
    REQUIRE(moved.deadline() == deadline);
    DeviceRuntime::Operation assigned;
    assigned = std::move(moved);
    REQUIRE(assigned.deadline() == deadline);
}

TEST_CASE("comm: recovery preserves old send evidence without reusing old response state",
          "[comm][runtime][lifecycle][regression]") {
    auto session = Session::create([] {
        auto fake = std::make_unique<FakeTransport>();
        fake->echo_cmd(0x28, 1, 6, 42);
        return fake;
    });
    auto core = DeviceRuntime::acquire(session, "receipt-recovery", {{0x28, false}});
    auto old = core->submit(plan(), Clock::now() + 1s);
    const auto previous = old.wait();
    const auto before = old.send_result();
    core->recover();
    const auto after = old.send_result();
    auto next = core->execute(plan(), Clock::now() + 1s);
    core->close();
    session->close();
    REQUIRE(before.has_value());
    REQUIRE(after.has_value());
    REQUIRE(test_support::require_value(after).outcome == linkerbot::transport::SendOutcome::dispatched);
    REQUIRE(test_support::require_value(after).generation == test_support::require_value(before).generation);
    REQUIRE(test_support::require_value(after).sent_frames == test_support::require_value(before).sent_frames);
    REQUIRE(next.physical_generation != previous.physical_generation);
    REQUIRE(next.device_generation != previous.device_generation);
    REQUIRE(old.wait().physical_generation == previous.physical_generation);
}

TEST_CASE("comm: failed recovery rejects work and a retry installs a fresh generation",
          "[comm][runtime][lifecycle][regression]") {
    bool reject_open = false;
    auto session = Session::create([&]() -> std::unique_ptr<linkerbot::can::CANTransport> {
        if (reject_open) {
            throw std::runtime_error("injected reopen failure");
        }
        auto fake = std::make_unique<FakeTransport>();
        fake->echo_cmd(0x28, 1, 6, 42);
        return fake;
    });
    auto core = DeviceRuntime::acquire(session, "retry-recovery", {{0x28, false}});
    const auto previous = core->execute(plan(), Clock::now() + 1s);
    reject_open = true;
    REQUIRE_THROWS(core->recover());
    REQUIRE(core->runtime_health().state == linkerbot::hand::runtime::DeviceState::faulted);
    REQUIRE_FALSE(test_support::require_value(core->snapshot("angle")).valid);
    REQUIRE_THROWS(core->execute(plan(), Clock::now() + 1s));
    reject_open = false;
    REQUIRE_NOTHROW(core->recover());
    const auto next = core->execute(plan(), Clock::now() + 1s);
    core->close();
    session->close();
    REQUIRE(next.valid);
    REQUIRE(next.physical_generation != previous.physical_generation);
}

TEST_CASE("comm: receipt preparation failure occurs before physical admission", "[comm][runtime][regression]") {
    Rig rig;
    using Access = linkerbot::hand::unified_detail::RuntimeAccess;
    REQUIRE_THROWS_AS(Access::execute_command(
                          *rig.runtime, Clock::now() + 1s, true, {}, {}, {},
                          [&] { rig.runtime->execute(plan(), Clock::now() + 1s); }, [] { throw std::bad_alloc(); }),
                      std::bad_alloc);
    REQUIRE(rig.fake->sent().empty());
    REQUIRE(rig.runtime->runtime_health().in_flight == 0);
    REQUIRE(rig.runtime->runtime_health().state == linkerbot::hand::runtime::DeviceState::ready);
    rig.fake->echo_cmd(0x28, 1, 6, 42);
    REQUIRE(rig.runtime->execute(plan(), Clock::now() + 1s).valid);
}

TEST_CASE("comm: a throwing receipt observer cannot replay send accounting", "[comm][runtime][regression]") {
    Rig rig;
    std::atomic<unsigned> calls{0};
    std::string observer_error;
    using Access = linkerbot::hand::unified_detail::RuntimeAccess;
    try {
        Access::execute_command(
            *rig.runtime, Clock::now() + 1s, true,
            [&](const linkerbot::transport::SendResult &, bool) {
                ++calls;
                throw std::bad_alloc();
            },
            [](const auto &) { return true; }, {},
            [&] {
                auto request = rig.runtime->submit(plan(), Clock::now() + 1s);
                (void)request.wait_sent_until(Clock::now() + 1s);
            });
    } catch (const std::exception &error) {
        observer_error = error.what();
    }
    rig.runtime->close();
    const auto first = rig.runtime->runtime_health();
    const auto second = rig.runtime->runtime_health();
    INFO(observer_error);
    REQUIRE(calls == 1);
    REQUIRE(first.sent_frames == 1);
    REQUIRE(second.sent_frames == first.sent_frames);
}

TEST_CASE("comm: terminal receipt publishes completed session accounting", "[comm][runtime][regression]") {
    Rig rig;
    const auto before = rig.session->health();
    auto request = rig.runtime->submit(plan(), Clock::now() + 1s);
    const auto receipt = request.wait_sent_until(Clock::now() + 1s);
    const auto after = rig.session->health();
    REQUIRE(receipt.outcome == linkerbot::transport::SendOutcome::dispatched);
    REQUIRE(after.tx_frames == before.tx_frames + receipt.sent_frames);
    request.cancel();
}

TEST_CASE("unbounded legacy timeout conversions saturate instead of wrapping", "[comm][python-phase2]") {
    using namespace linkerbot::hand::runtime;
    REQUIRE((deadline_after(std::chrono::milliseconds::max()) == Deadline::max()));
    REQUIRE((deadline_after(remaining(Deadline::max())) == Deadline::max()));
}

TEST_CASE("required response windows buffer an early protocol failure until delivery is known",
          "[comm][python-phase2]") {
    Rig rig;
    auto plan = linkerbot::hand::o6::protocol::read_angle(0x28);
    plan.response_window = 40ms;
    plan.defer_errors_until_send = true;
    plan.consume = [](const Message &) -> linkerbot::hand::protocol::Progress {
        throw linkerbot::hand::ProtocolError("injected malformed response");
    };
    rig.fake->arm_tx_gate();
    const auto request = rig.runtime->submit(plan, Clock::now() + 2s);
    const bool entered = rig.fake->wait_until_tx_entered(500ms);
    Message reply{};
    reply.id = 0x28;
    reply.dlc = 2;
    reply.data[0] = 1;
    rig.fake->inject(reply);
    std::this_thread::sleep_for(10ms);
    const bool early_done = request.completed();
    rig.fake->release_tx_gate();
    REQUIRE((entered));
    REQUIRE_FALSE(early_done);
    REQUIRE_THROWS_AS(request.wait(), linkerbot::hand::ProtocolError);
    REQUIRE(
        (test_support::require_value(request.send_result()).outcome == linkerbot::transport::SendOutcome::dispatched));
}

TEST_CASE("required response decoding cannot overrun its response window", "[comm][python-phase2]") {
    Rig rig;
    auto transaction = plan();
    transaction.response_window = 20ms;
    transaction.defer_errors_until_send = true;
    const auto decode = transaction.consume;
    transaction.consume = [decode](const Message &message) {
        std::this_thread::sleep_for(50ms);
        return decode(message);
    };
    rig.fake->on_request(0x28, [](const Message &request) {
        auto response = request;
        response.dlc = 7;
        return std::vector<Message>{response};
    });
    REQUIRE_THROWS_AS(rig.runtime->execute(transaction, Clock::now() + 1s), linkerbot::hand::ResponseTimeoutError);
}

TEST_CASE("recovery quiet waits can be interrupted without waiting for the quiet interval", "[comm][python-phase2]") {
    Rig rig;
    std::atomic<bool> cancelled{false};
    auto quiet = std::async(std::launch::async, [&] {
        try {
            linkerbot::hand::unified_detail::RuntimeAccess::wait_endpoint_quiet(*rig.runtime, 1s, Clock::now() + 2s,
                                                                                [&] { return cancelled.load(); });
            return false;
        } catch (const linkerbot::hand::ClosedError &) {
            return true;
        }
    });
    cancelled.store(true);
    const auto interrupted = quiet.wait_for(200ms);
    const bool closed = quiet.get();
    REQUIRE((interrupted == std::future_status::ready));
    REQUIRE((closed));
}

TEST_CASE("recovery quiet windows do not include another device route", "[comm][python-phase2]") {
    Rig rig;
    std::atomic<bool> stop{false};
    auto peer = std::thread([&] {
        Message message{};
        message.id = 0x27;
        message.dlc = 1;
        while (!stop.load()) {
            rig.fake->inject(message);
            std::this_thread::sleep_for(2ms);
        }
    });
    std::exception_ptr error;
    try {
        linkerbot::hand::unified_detail::RuntimeAccess::wait_endpoint_quiet(*rig.runtime, 20ms, Clock::now() + 500ms,
                                                                            [] { return false; });
    } catch (...) {
        error = std::current_exception();
    }
    stop.store(true);
    peer.join();
    REQUIRE_FALSE(error);
}

TEST_CASE("buffered response failures keep matching frames out of report decoders", "[comm][python-phase2]") {
    const auto barrier = std::make_shared<std::promise<void>>();
    auto processed = barrier->get_future();
    Rig rig;
    (void)rig.runtime->observe("angle", [](const Message &message) {
        if (message.id == 0x28 && message.dlc == 7 && message.data[0] == 1) {
            return linkerbot::hand::protocol::Progress::complete(static_cast<int>(message.data[1]));
        }
        return linkerbot::hand::protocol::Progress::unmatched();
    });
    (void)rig.runtime->observe("barrier", [barrier](const Message &message) {
        if (message.id == 0x28 && message.dlc == 1 && message.data[0] == 0xFE) {
            barrier->set_value();
            return linkerbot::hand::protocol::Progress::complete(true);
        }
        return linkerbot::hand::protocol::Progress::unmatched();
    });
    Message baseline{};
    baseline.id = 0x28;
    baseline.dlc = 7;
    baseline.data[0] = 1;
    baseline.data[1] = 64;
    rig.fake->inject(baseline);
    const auto baseline_deadline = Clock::now() + 1s;
    while (!rig.runtime->snapshot_as<int>("angle") && Clock::now() < baseline_deadline) {
        std::this_thread::sleep_for(1ms);
    }
    REQUIRE((test_support::require_value(rig.runtime->snapshot_as<int>("angle")) == 64));
    auto transaction = plan();
    transaction.response_window = 100ms;
    transaction.defer_errors_until_send = true;
    transaction.publish_after_send = true;
    transaction.preserve_last_success = true;
    transaction.consume = [](const Message &message) {
        if (message.id != 0x28 || message.dlc == 0 || message.data[0] != 1) {
            return linkerbot::hand::protocol::Progress::unmatched();
        }
        if (message.dlc != 7) {
            throw linkerbot::hand::ProtocolError("injected malformed vector");
        }
        return linkerbot::hand::protocol::Progress::complete(static_cast<int>(message.data[1]));
    };
    rig.fake->arm_tx_gate();
    const auto request = rig.runtime->submit(transaction, Clock::now() + 2s);
    const bool entered = rig.fake->wait_until_tx_entered(1s);
    Message response{};
    response.id = 0x28;
    response.dlc = 2;
    response.data[0] = 1;
    rig.fake->inject(response);
    response.dlc = 7;
    response.data[1] = 99;
    rig.fake->inject(response);
    response.dlc = 1;
    response.data[0] = 0xFE;
    rig.fake->inject(response);
    const auto observed = processed.wait_for(1s);
    const auto before = rig.runtime->snapshot_as<int>("angle");
    const bool completed_early = request.completed();
    rig.fake->release_tx_gate();
    REQUIRE((entered));
    REQUIRE((observed == std::future_status::ready));
    REQUIRE_FALSE(completed_early);
    REQUIRE((test_support::require_value(before) == 64));
    REQUIRE_THROWS_AS(request.wait(), linkerbot::hand::ProtocolError);
    REQUIRE((test_support::require_value(rig.runtime->snapshot_as<int>("angle")) == 64));
}

TEST_CASE("preserved claimed dispatch batches finish every frame during runtime close", "[comm][python-phase2]") {
    for (const bool dispatch_only : {false, true}) {
        Rig rig;
        const auto delivery = std::make_shared<linkerbot::hand::unified_detail::CommandDelivery>();
        auto transaction = plan();
        transaction.consume = {};
        transaction.frames = {transaction.frames.front(), transaction.frames.front(), transaction.frames.front()};
        rig.fake->arm_tx_gate();
        auto sending = std::async(std::launch::async, [&] {
            try {
                using Access = linkerbot::hand::unified_detail::RuntimeAccess;
                Access::with_response_windows(
                    *rig.runtime, [](auto plan) { return plan; },
                    [&] {
                        Access::execute_command(
                            *rig.runtime, Clock::now() + 2s, dispatch_only, {}, [](const auto &) { return true; }, {},
                            [&] { (void)rig.runtime->execute(transaction, Clock::now() + 2s); }, {}, {}, {},
                            [delivery](const Access::Submission &submit, bool final) {
                                return delivery->admit(submit, final);
                            });
                    });
            } catch (const linkerbot::hand::HandError &) {
                // The native wait ends on close; the retained physical receipt owns
                // the dispatch-only command's final result at the facade boundary.
                return true;
            }
            return false;
        });
        const bool claimed = rig.fake->wait_until_tx_entered(1s);
        const bool preserved = delivery->preserve_claimed(dispatch_only);
        auto closing = std::async(std::launch::async, [&] { rig.runtime->close(); });
        const auto deadline = Clock::now() + 1s;
        while (rig.runtime->runtime_health().in_flight != 0 && Clock::now() < deadline) {
            std::this_thread::sleep_for(1ms);
        }
        const bool ended = rig.runtime->runtime_health().in_flight == 0;
        rig.fake->release_tx_gate();
        const bool native_wait_ended = sending.get();
        closing.get();
        REQUIRE((claimed));
        REQUIRE((preserved == dispatch_only));
        REQUIRE((ended));
        REQUIRE((native_wait_ended));
        REQUIRE((delivery->draining_claimed()));
        // Required-ACK commands fail closed but their claimed physical batch drains.
        const auto result = test_support::require_value(delivery->receipt).result();
        REQUIRE((result.outcome == linkerbot::transport::SendOutcome::dispatched));
        REQUIRE((result.sent_frames == 3));
        REQUIRE((rig.fake->sent().size() == 3));
    }
}

TEST_CASE("Python read batches also finish every claimed frame during close", "[comm][python-phase2]") {
    Rig rig;
    auto transaction = plan();
    transaction.frames = {transaction.frames.front(), transaction.frames.front(), transaction.frames.front()};
    rig.fake->arm_tx_gate();
    auto reading = std::async(std::launch::async, [&] {
        try {
            linkerbot::hand::unified_detail::RuntimeAccess::with_response_windows(
                *rig.runtime, [](auto plan) { return plan; },
                [&] { (void)rig.runtime->execute(transaction, Clock::now() + 2s); });
        } catch (const linkerbot::hand::HandError &) {
            return true;
        }
        return false;
    });
    const bool claimed = rig.fake->wait_until_tx_entered(1s);
    auto closing = std::async(std::launch::async, [&] { rig.runtime->close(); });
    const auto deadline = Clock::now() + 1s;
    while (rig.runtime->runtime_health().in_flight != 0 && Clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    const bool ended = rig.runtime->runtime_health().in_flight == 0;
    rig.fake->release_tx_gate();
    const bool waiter_closed = reading.get();
    closing.get();
    REQUIRE((claimed));
    REQUIRE((ended));
    REQUIRE((waiter_closed));
    REQUIRE((rig.fake->sent().size() == 3));
    REQUIRE((rig.session->health().tx_frames == 3));
}

TEST_CASE("closed Python runtimes cannot cancel a reopened endpoint", "[comm][python-phase2]") {
    Rig rig;
    auto transaction = plan();
    transaction.consume = {};
    using Access = linkerbot::hand::unified_detail::RuntimeAccess;
    Access::with_response_windows(
        *rig.runtime, [](auto next) { return next; },
        [&] { (void)rig.runtime->execute(transaction, Clock::now() + 1s); });
    rig.runtime->close();
    Access::retire_closed_profiles(rig.session);
    const auto next = DeviceRuntime::acquire(rig.session, "test/0x28", {{0x28, false}});
    Access::resume_endpoint(*next);
    rig.runtime->cancel_all();
    const auto result = next->execute(transaction, Clock::now() + 1s);
    const auto frames = rig.session->health().tx_frames;
    next->close();
    REQUIRE((rig.runtime->runtime_health().state == linkerbot::hand::runtime::DeviceState::closed));
    REQUIRE((result.valid));
    REQUIRE((frames == 2));
}

namespace {
    struct PostAcceptanceRace {
        std::atomic<bool> observed{false};
        std::promise<void> entered;
        std::promise<void> release;
        std::shared_future<void> gate = release.get_future().share();
    };
    class PostAcceptanceError : public linkerbot::hand::ProtocolError {
    public:
        explicit PostAcceptanceError(std::shared_ptr<PostAcceptanceRace> state)
            : ProtocolError("injected post-acceptance failure"), state_(std::move(state)) {}
        const char *what() const noexcept override {
            try {
                if (!state_->observed.exchange(true)) {
                    state_->entered.set_value();
                    state_->gate.wait();
                }
            } catch (...) {
                return "post-acceptance fixture synchronization failed";
            }
            return ProtocolError::what();
        }

    private:
        std::shared_ptr<PostAcceptanceRace> state_;
    };
}

TEST_CASE("late physical completion cannot publish profile success before its send observer", "[comm][python-phase2]") {
    Rig rig;
    const auto state = std::make_shared<PostAcceptanceRace>();
    auto error_recording = state->entered.get_future();
    std::promise<linkerbot::transport::SendReceipt> accepted;
    auto receipt_future = accepted.get_future();
    std::atomic<bool> observed{false};
    rig.fake->arm_tx_gate();
    auto sending = std::async(std::launch::async, [&] {
        try {
            using Access = linkerbot::hand::unified_detail::RuntimeAccess;
            Access::with_response_windows(
                *rig.runtime, [](auto next) { return next; },
                [&] {
                    Access::execute_command(
                        *rig.runtime, Clock::now() + 5s, true, [&](const auto &, bool) { observed.store(true); },
                        [](const auto &) { return true; }, {},
                        [&] { (void)rig.runtime->execute(plan(), Clock::now() + 5s); }, {}, {},
                        [&](const auto &receipt, bool) {
                            accepted.set_value(receipt);
                            throw PostAcceptanceError(state);
                        });
                });
        } catch (const linkerbot::hand::HandError &) {
            return true;
        }
        return false;
    });
    const auto captured = receipt_future.wait_for(1s);
    const auto recording = error_recording.wait_for(1s);
    const bool claimed = rig.fake->wait_until_tx_entered(1s);
    std::optional<linkerbot::transport::SendReceipt> receipt;
    if (captured == std::future_status::ready) {
        receipt = receipt_future.get();
    }
    rig.fake->release_tx_gate();
    bool completed = false;
    if (receipt) {
        try {
            (void)receipt->wait_result_until(Clock::now() + 1s);
            completed = true;
        } catch (const linkerbot::can::TimeoutError &) {
            completed = false;
        }
    }
    state->release.set_value();
    const bool error_preserved = sending.get();
    linkerbot::hand::unified_detail::RuntimeAccess::flush_sends(*rig.runtime);
    REQUIRE((captured == std::future_status::ready));
    REQUIRE((recording == std::future_status::ready));
    REQUIRE((claimed));
    REQUIRE((completed));
    REQUIRE((error_preserved));
    REQUIRE((observed.load()));
    REQUIRE((rig.session->health().tx_frames == 1));
}

TEST_CASE("Python construction capacity bounds the real receive inbox", "[comm][python-phase4]") {
    ReportProbe probe;
    auto transport = std::make_unique<FakeTransport>();
    auto *fake = transport.get();
    auto session = Session::own(std::move(transport));
    std::shared_ptr<DeviceRuntime> core;
    {
        linkerbot::hand::unified_detail::RuntimeAccess::InboxScope scope(1);
        core = DeviceRuntime::acquire(session, "reports", {{0x28, false}});
    }
    REQUIRE((linkerbot::hand::unified_detail::RuntimeAccess::inbox_capacity(*core) == 1));
    core->observe_factory("reports", probe.factory(true));
    fake->inject(fragment({1, 9}));
    REQUIRE(probe.wait(1));
    fake->inject(fragment({0x7f}));
    bool entered;
    {
        std::unique_lock<std::mutex> lock(probe.mutex);
        entered = probe.cv.wait_for(lock, 1s, [&] { return probe.blocked; });
    }
    // Always release the decoder before assertions/unwinding.
    fake->inject(fragment({2, 7}));
    fake->inject(fragment({2, 8}));
    const auto limit = Clock::now() + 1s;
    while (!session->health().overflows && Clock::now() < limit) {
        std::this_thread::yield();
    }
    const auto overflow = session->health().overflows;
    probe.unblock();
    REQUIRE(entered);
    REQUIRE(overflow > 0);
    REQUIRE(probe.wait(2, 2));
    (void)core->runtime_health(); // overflow reset and queued-fragment drain have completed
    fake->inject(fragment({2, 7}));
    REQUIRE(probe.wait(3, 2));
    REQUIRE_FALSE(core->snapshot_as<int>("reports").has_value());
    fake->inject(fragment({1, 3}));
    REQUIRE(probe.wait(4, 2));
    fake->inject(fragment({2, 4}));
    REQUIRE(probe.wait(5, 2));
    REQUIRE(core->snapshot_as<int>("reports") == 0x304);
    core->close();
    session->close();
}
