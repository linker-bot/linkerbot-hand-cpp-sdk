#include "../../../src/hand/unified/detail/read_coordinator.hpp"
#include "../../../src/hand/unified/detail/runtime_access.hpp"
#include "../../../validation/fake_classic_hand.hpp"
#include "hand/support/optional_value.hpp"
#include <catch2/catch_test_macros.hpp>
#include <future>
#include <thread>

using namespace linkerbot::hand;
using namespace std::chrono_literals;
using unified_detail::RuntimeAccess;
namespace {
    HandDeviceConfig python_config() {
        auto c = validation::config();
        c.runtime_behavior = RuntimeBehavior::kPython;
        c.identity_timeout = 300ms;
        c.response_timeout = 800ms;
        c.transaction_timeout = 10ms;
        return c;
    }
    linkerbot::can::Message angle_reply() {
        linkerbot::can::Message m{};
        m.id = 0x28;
        m.dlc = 7;
        m.data[0] = 1;
        for (size_t i = 1; i < 7; ++i) {
            m.data[i] = 64;
        }
        return m;
    }
    void hold_angles(validation::FakeHand &d, std::promise<void> &sent) {
        d.transport->on_request(0x28, [&sent](const auto &r) {
            if (r.data[0] == 1 && r.dlc == 1) {
                sent.set_value();
                return std::vector<linkerbot::can::Message>{};
            }
            return validation::l6_reply(r);
        });
    }
}

TEST_CASE("Python stop polling drains the claimed read after its poll observer times out", "[comm][python-phase4]") {
    auto c = python_config();
    c.shutdown_timeout = 2s;
    std::promise<void> sent;
    validation::FakeHand d(c);
    hold_angles(d, sent);
    d.hand.start_polling({{TelemetrySemantic::kAngles, 2s, AngleFeedbackPolicy::kQueryOnly}});
    REQUIRE((sent.get_future().wait_for(1s) == std::future_status::ready));
    std::this_thread::sleep_for(30ms);
    auto stop = std::async(std::launch::async, [&] { d.hand.stop_polling(); });
    const auto before_reply = stop.wait_for(60ms);
    d.transport->inject(angle_reply());
    stop.get();
    REQUIRE((before_reply == std::future_status::timeout));
    REQUIRE((d.hand.get_faults(300ms).meta.valid));
    REQUIRE_FALSE(d.hand.get_health().polling_active);
    d.transport->on_request(0x28, validation::l6_reply);
    d.hand.start_polling({{TelemetrySemantic::kAngles, 20ms}});
    d.hand.stop_polling();
}

TEST_CASE("Python stop polling reports its bound without ending the device generation", "[comm][python-phase4]") {
    auto c = python_config();
    c.shutdown_timeout = 75ms;
    std::promise<void> sent;
    validation::FakeHand d(c);
    hold_angles(d, sent);
    const auto generation = d.hand.get_cached_snapshot().connection_generation;
    d.hand.start_polling({{TelemetrySemantic::kAngles, 2s}});
    REQUIRE((sent.get_future().wait_for(1s) == std::future_status::ready));
    const auto start = runtime::Clock::now();
    bool timed_out = false;
    try {
        d.hand.stop_polling();
    } catch (const TimeoutError &) {
        timed_out = true;
    }
    const auto elapsed = runtime::Clock::now() - start;
    d.transport->inject(angle_reply());
    REQUIRE((timed_out));
    REQUIRE((elapsed < 500ms));
    REQUIRE((d.hand.is_open()));
    REQUIRE((d.hand.get_faults(1s).meta.valid));
    REQUIRE((d.hand.get_cached_snapshot().connection_generation == generation));
    REQUIRE_NOTHROW(d.hand.stop_polling());
    d.hand.recover_for(2s);
    REQUIRE_FALSE(d.hand.get_health().polling_active);
}

TEST_CASE("poll cancellation preserves a shared reader and drains only the claimed poll", "[comm][python-phase4]") {
    unified_detail::ReadCoordinator<int> reads(4);
    std::promise<void> entered, release;
    auto gate = release.get_future().share();
    auto poll = std::async(std::launch::async, [&] {
        try {
            (void)reads.read(
                1, runtime::Clock::now() + 2s,
                [&] {
                    entered.set_value();
                    gate.wait();
                    return 42;
                },
                true);
            return false;
        } catch (const ClosedError &) {
            return true;
        }
    });
    entered.get_future().wait();
    auto reader =
        std::async(std::launch::async, [&] { return reads.read(1, runtime::Clock::now() + 2s, [] { return 99; }); });
    const auto joined_deadline = runtime::Clock::now() + 1s;
    while (reads.waiter_count() == 0 && runtime::Clock::now() < joined_deadline) {
        std::this_thread::yield();
    }
    const bool joined = reads.waiter_count() == 1;
    reads.cancel_polls();
    const auto cancelled = poll.wait_for(200ms);
    const bool idle_before = reads.wait_polls_idle(runtime::Clock::now() + 20ms);
    release.set_value();
    const bool stopped = poll.get();
    const auto value = reader.get();
    REQUIRE((joined));
    REQUIRE((cancelled == std::future_status::ready));
    REQUIRE((stopped));
    REQUIRE_FALSE(idle_before);
    REQUIRE((value == 42));
    REQUIRE((reads.wait_polls_idle(runtime::Clock::now() + 1s)));
}

TEST_CASE("poll cancellation does not wait for a read claimed without poll demand", "[comm][python-phase4]") {
    unified_detail::ReadCoordinator<int> reads(4);
    std::promise<void> entered, release;
    auto gate = release.get_future().share();
    auto reader = std::async(std::launch::async, [&] {
        return reads.read(1, runtime::Clock::now() + 2s, [&] {
            entered.set_value();
            gate.wait();
            return 42;
        });
    });
    entered.get_future().wait();
    reads.cancel_polls();
    const bool idle = reads.wait_polls_idle(runtime::Clock::now() + 20ms);
    release.set_value();
    REQUIRE((reader.get() == 42));
    REQUIRE((idle));
}

TEST_CASE("Python health uses the successful facade generation across reopen and recovery", "[comm][python-phase4]") {
    validation::FakeHand d(python_config());
    for (uint64_t generation = 1; generation <= 3; ++generation) {
        const auto snapshot = d.hand.get_cached_snapshot();
        const auto health = d.hand.get_health();
        REQUIRE((snapshot.connection_generation == generation));
        REQUIRE((health.connection_generation == generation));
        REQUIRE_FALSE(health.channels.empty());
        for (const auto &channel : health.channels) {
            REQUIRE((channel.data_age.generation == generation));
        }
        if (generation == 1) {
            d.hand.close();
            REQUIRE((d.hand.get_health().connection_generation == generation));
            d.hand.open();
        } else if (generation == 2) {
            const auto physical = health.session.generation;
            d.hand.recover_for(2s);
            REQUIRE((d.hand.get_health().session.generation > physical));
        }
    }
}

TEST_CASE("Native health retains its original generation behavior on reopen", "[comm][python-phase4]") {
    validation::FakeHand d;
    const auto native = d.hand.get_health().connection_generation;
    d.hand.close();
    d.hand.open();
    REQUIRE((d.hand.get_health().connection_generation == native));
    REQUIRE((d.hand.get_cached_snapshot().connection_generation == 2));
}

TEST_CASE("Python firmware downgrade records the final error once and preserves later errors",
          "[comm][python-phase4]") {
    auto c = python_config();
    c.response_timeout = 40ms;
    validation::FakeHand d(c);
    d.transport->on_request(0x28, [](const auto &r) {
        if (r.data[0] == 0x36) {
            return std::vector<linkerbot::can::Message>{};
        }
        return validation::l6_reply(r);
    });
    REQUIRE_THROWS_AS(d.hand.get_currents(1s), TimeoutError);
    REQUIRE_THROWS_AS(d.hand.get_currents(1s), TimeoutError);
    REQUIRE_THROWS_AS(d.hand.get_currents(1s), UnsupportedFirmwareError);
    auto health = d.hand.get_health();
    REQUIRE((test_support::require_value(health.recent_error).code == ErrorCode::kUnsupportedFirmware));
    REQUIRE((test_support::require_value(health.recent_error).error_type == "UnsupportedFirmwareError"));
    REQUIRE((health.consecutive_errors == 3));
    REQUIRE((d.hand.get_angles(1s).meta.valid));
    REQUIRE((d.hand.get_health().consecutive_errors == 0));
    d.transport->on_request(0x28, [](const auto &r) {
        auto replies = validation::l6_reply(r);
        replies.front().dlc = 1;
        return replies;
    });
    REQUIRE_THROWS_AS(d.hand.get_angles(1s), ProtocolError);
    REQUIRE((test_support::require_value(d.hand.get_health().recent_error).code == ErrorCode::kProtocol));
    d.hand.recover_for(2s);
    REQUIRE_FALSE(d.hand.get_health().recent_error.has_value());
}

TEST_CASE("an older query downgrade cannot replace a newer native error", "[comm][python-phase4]") {
    auto transport = std::make_unique<linkerbot::can::FakeTransport>();
    auto session = linkerbot::transport::Session::own(std::move(transport));
    auto core = runtime::DeviceRuntime::acquire(session, "phase4", {{0x28, false}});
    auto capture = std::make_shared<std::atomic<uint64_t>>(0);
    runtime::TransactionPlan plan;
    plan.domain = "test";
    plan.keys = {"test"};
    plan.frames = {angle_reply()};
    plan.response_window = 20ms;
    plan.poison_on_timeout = false;
    plan.consume = [](const auto &) { return protocol::Progress::unmatched(); };
    REQUIRE_THROWS_AS(RuntimeAccess::with_response_windows(
                          *core, [](auto p) { return p; }, [&] { (void)core->execute(plan, runtime::Deadline::max()); },
                          runtime::Deadline::max(), {}, capture),
                      TimeoutError);
    const auto revision = capture->load();
    REQUIRE((revision > 0));
    core->record_error(std::make_exception_ptr(TransportError("newer driver error")));
    RuntimeAccess::promote_error(*core, revision, UnsupportedFirmwareError("old downgrade"));
    RuntimeAccess::initialized_health(*core);
    const auto health = core->health();
    REQUIRE((test_support::require_value(health.recent_error).code == ErrorCode::kTransport));
    REQUIRE((health.consecutive_errors == 2));
    core->close();
}

TEST_CASE("Python receive capacity is applied to its routed inbox and survives recovery", "[comm][python-phase4]") {
    for (const auto capacity : {size_t{256}, size_t{16}}) {
        auto c = python_config();
        if (capacity == 16) {
            c.route_queue_capacity = capacity;
        }
        auto session = linkerbot::transport::Session::create([] {
            auto t = std::make_unique<linkerbot::can::FakeTransport>();
            t->on_request(0x28, validation::l6_reply);
            return t;
        });
        HandDevice hand(c, session);
        hand.open();
        auto core = runtime::DeviceRuntime::acquire(session, "l6/40", {{0x28, false}}, capacity);
        REQUIRE((RuntimeAccess::inbox_capacity(*core) == capacity));
        REQUIRE_THROWS_AS(runtime::DeviceRuntime::acquire(session, "l6/40", {{0x28, false}}, capacity + 1),
                          ValidationError);
        REQUIRE((hand.get_angles(1s).meta.valid));
        hand.recover_for(2s);
        REQUIRE((RuntimeAccess::inbox_capacity(*core) == capacity));
        hand.close();
    }
}

TEST_CASE("Native receive capacity and construction scope survive Python construction failure",
          "[comm][python-phase4]") {
    auto session = linkerbot::transport::Session::own(std::make_unique<linkerbot::can::FakeTransport>());
    bool threw = false;
    try {
        RuntimeAccess::InboxScope scope(3);
        throw std::runtime_error("construction failed");
    } catch (const std::runtime_error &) {
        threw = true;
    }
    REQUIRE((threw));
    auto native = runtime::DeviceRuntime::acquire(session, "native", {{0x28, false}});
    REQUIRE((RuntimeAccess::inbox_capacity(*native) == 512));
    native->close();
}

TEST_CASE("receive capacity validation rejects before creating a transport", "[comm][python-phase4]") {
    for (const auto python : {false, true}) {
        auto c = python_config();
        if (!python) {
            c.runtime_behavior = RuntimeBehavior::kNative;
        }
        c.route_queue_capacity = python ? 0 : 256;
        int factories = 0;
        REQUIRE_THROWS_AS(HandDevice(c,
                                     [&] {
                                         ++factories;
                                         return std::make_unique<linkerbot::can::FakeTransport>();
                                     }),
                          ValidationError);
        REQUIRE((factories == 0));
    }
}

TEST_CASE("Python stop polling leaves another device on the same Session responsive", "[comm][python-phase4]") {
    linkerbot::can::FakeTransport *raw = nullptr;
    std::promise<void> sent;
    auto session = linkerbot::transport::Session::create([&] {
        auto t = std::make_unique<linkerbot::can::FakeTransport>();
        raw = t.get();
        t->on_request(0x28, validation::l6_reply);
        t->on_request(0x27, [](const auto &r) {
            auto replies = validation::l6_reply(r);
            if (r.data[0] == 0xC0) {
                for (auto &reply : replies) {
                    if (reply.data[1] == 1) {
                        reply.data[7] = 'R';
                    }
                }
            }
            return replies;
        });
        return t;
    });
    auto c = python_config();
    c.shutdown_timeout = 2s;
    HandDevice left(c, session);
    c.side = HandSide::kRight;
    HandDevice right(c, session);
    left.open();
    right.open();
    raw->on_request(0x28, [&sent](const auto &r) {
        if (r.data[0] == 1 && r.dlc == 1) {
            sent.set_value();
            return std::vector<linkerbot::can::Message>{};
        }
        return validation::l6_reply(r);
    });
    left.start_polling({{TelemetrySemantic::kAngles, 2s}});
    REQUIRE((sent.get_future().wait_for(1s) == std::future_status::ready));
    auto stop = std::async(std::launch::async, [&] { left.stop_polling(); });
    const auto before_reply = stop.wait_for(40ms);
    bool peer_ok = false;
    try {
        peer_ok = right.get_faults(300ms).meta.valid;
    } catch (const HandError &) {
        peer_ok = false;
    }
    raw->inject(angle_reply());
    stop.get();
    REQUIRE((before_reply == std::future_status::timeout));
    REQUIRE((peer_ok));
    REQUIRE((right.is_open()));
    left.close();
    right.close();
}

TEST_CASE("unclaimed poll work is discarded while an active reader continues", "[comm][python-phase4]") {
    unified_detail::ReadCoordinator<int> reads(4);
    std::promise<void> entered, release, admitted;
    auto gate = release.get_future().share();
    auto reader = std::async(std::launch::async, [&] {
        return reads.read(1, runtime::Clock::now() + 3s, [&] {
            entered.set_value();
            gate.wait();
            return 42;
        });
    });
    entered.get_future().wait();
    std::atomic<int> executions{0};
    auto poll = std::async(std::launch::async, [&] {
        try {
            (void)reads.read_prepared(
                2, runtime::Clock::now() + 3s,
                [&](const auto &) {
                    ++executions;
                    return 99;
                },
                {}, true, {},
                [&] {
                    admitted.set_value();
                    return true;
                });
            return false;
        } catch (const ClosedError &) {
            return true;
        }
    });
    admitted.get_future().wait();
    reads.cancel_polls();
    const bool idle = reads.wait_polls_idle(runtime::Clock::now() + 20ms);
    release.set_value();
    REQUIRE((reader.get() == 42));
    REQUIRE((poll.get()));
    reads.drain();
    REQUIRE((idle));
    REQUIRE((executions.load() == 0));
}
