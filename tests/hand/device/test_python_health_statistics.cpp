#include "../../../src/hand/unified/detail/python_health.hpp"
#include "../../../src/hand/unified/detail/python_polling.hpp"
#include "../../../src/hand/unified/detail/python_state.hpp"
#include "../../../src/hand/unified/detail/python_transactions.hpp"
#include "../../../validation/fake_classic_hand.hpp"
#include "hand/support/optional_value.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <future>
#include <linkerbot/hand/o6/protocol_backend.hpp>
#include <thread>

using namespace linkerbot::hand;
using namespace std::chrono_literals;
namespace {
    using S = TelemetrySemantic;
    auto config(Model model = Model::kL6) {
        auto c = validation::config();
        c.model = model;
        c.runtime_behavior = RuntimeBehavior::kPython;
        c.identity_timeout = 500ms;
        c.response_timeout = 300ms;
        if (model == Model::kO30) {
            c.o30_layout = O30Layout::kPythonV25;
            c.angle_mapping_path.reset();
        }
        return c;
    }
    ChannelMetrics channel(const HandHealth &h, const std::string &name) {
        for (const auto &c : h.channels) {
            if (c.channel == name) {
                return c;
            }
        }
        throw std::runtime_error("missing health channel: " + name);
    }
    ForceFieldMetrics field(const HandHealth &h, const std::string &name) {
        for (const auto &f : h.force_fields) {
            if (f.field == name) {
                return f;
            }
        }
        throw std::runtime_error("missing health force field: " + name);
    }
    HandHealth health(const unified_detail::PythonState &state) {
        HandHealth result;
        state.project_health(result);
        return result;
    }
    PollingChannel force(std::initializer_list<ForceFieldPolling> fields) {
        PollingChannel c{S::kForce, 100ms};
        c.force_fields = fields;
        return c;
    }
}

TEST_CASE("Python health counts only a configured polling measurement window", "[comm][python-phase5]") {
    validation::FakeHand d(config());
    const auto initial = d.hand.get_health();
    REQUIRE(channel(initial, "angles").completed_reads == 0);
    REQUIRE(channel(initial, "angles").feedback_samples == 0);
    REQUIRE(initial.transaction_latency.count == 4);
    const auto before = d.transport->sent().size();
    for (int i = 0; i < 10; ++i) {
        (void)d.hand.get_health();
    }
    REQUIRE(d.transport->sent().size() == before);
    REQUIRE(d.hand.get_angles(1s).meta.valid);
    const auto read = d.hand.get_health();
    REQUIRE(channel(read, "angles").completed_reads == 0);
    REQUIRE(channel(read, "angles").data_age.sequence == 2);
    REQUIRE(read.transaction_latency.count == initial.transaction_latency.count + 1);
    d.hand.start_polling({{S::kAngles, 20ms, AngleFeedbackPolicy::kQueryOnly}});
    const auto deadline = runtime::Clock::now() + 2s;
    while (channel(d.hand.get_health(), "angles").completed_reads < 2 && runtime::Clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    REQUIRE(channel(d.hand.get_health(), "angles").completed_reads >= 2);
    d.hand.stop_polling();
    const auto stopped = d.hand.get_health();
    REQUIRE(channel(stopped, "angles").completed_reads == 0);
    REQUIRE(channel(stopped, "angles").feedback_samples == 0);
    REQUIRE(channel(stopped, "angles").requested_hz == 0);
    REQUIRE(stopped.transaction_latency.count >= read.transaction_latency.count + 2);
    const auto count = stopped.transaction_latency.count;
    d.hand.close();
    REQUIRE(d.hand.get_health().transaction_latency.count == count);
    REQUIRE(channel(d.hand.get_health(), "angles").completed_reads == 0);
    d.hand.open();
    REQUIRE(d.hand.get_health().transaction_latency.count == count + 4);
    d.hand.recover_for(3s);
    REQUIRE(d.hand.get_health().transaction_latency.count == count + 8);
}

TEST_CASE("Native health retains native counters and leaves Python field metrics absent", "[comm][python-phase5]") {
    validation::FakeHand d;
    const auto h = d.hand.get_health();
    REQUIRE(channel(h, "angles").completed_reads == 1);
    REQUIRE(h.transaction_latency.count == 7);
    REQUIRE(h.force_fields.empty());
    REQUIRE_THROWS_AS(d.hand.start_polling({force({{"fn", 100ms}})}), PollingPlanError);
    REQUIRE_FALSE(d.hand.get_health().polling_active);
}

TEST_CASE("Python health percentile selection uses ties to even and retains only 2048 completions",
          "[comm][python-phase5]") {
    unified_detail::PythonHealthMetrics m;
    const auto start = HealthTimePoint{};
    HandHealth h;
    m.project(h);
    REQUIRE(h.transaction_latency.count == 0);
    REQUIRE_FALSE(h.last_successful_transaction);
    m.completed(start, start + 1ns);
    m.completed(start, start + 3ns);
    m.project(h);
    REQUIRE(test_support::require_value(h.transaction_latency.p50) == 1ns);
    m.completed(start, start + 5ns);
    m.completed(start, start + 7ns);
    m.project(h);
    REQUIRE(test_support::require_value(h.transaction_latency.p50) == 5ns);
    for (int i = 0; i < 2048; ++i) {
        m.completed(start, start + 10ns);
    }
    m.project(h);
    REQUIRE(h.transaction_latency.count == 2048);
    REQUIRE(test_support::require_value(h.transaction_latency.p50) == 10ns);
    REQUIRE(test_support::require_value(h.transaction_latency.p95) == 10ns);
    REQUIRE(test_support::require_value(h.transaction_latency.p99) == 10ns);
    REQUIRE_THROWS_AS(m.completed(start + 1ns, start), ValidationError);
    m.project(h);
    REQUIRE(h.transaction_latency.count == 2048);
    m.clear();
    m.project(h);
    REQUIRE(h.transaction_latency.count == 0);
    REQUIRE_FALSE(h.last_successful_transaction);
}

TEST_CASE("Python poll reads and command feedback use distinct counters", "[comm][python-phase5]") {
    unified_detail::PythonState state(config());
    state.replace({{S::kAngles, 100ms, AngleFeedbackPolicy::kQueryOnly}});
    runtime::Sample sample;
    sample.valid = true;
    sample.device_generation = 1;
    sample.received_at = runtime::Clock::now();
    sample.origin = protocol::Origin::report;
    state.publish("angle", sample);
    REQUIRE(channel(health(state), "angles").feedback_samples == 0);
    state.read_succeeded(S::kAngles, false);
    REQUIRE(channel(health(state), "angles").completed_reads == 1);
    REQUIRE(channel(health(state), "angles").feedback_samples == 1);
    sample.origin = protocol::Origin::command_feedback;
    state.publish("angle", sample);
    auto h = health(state);
    REQUIRE(channel(h, "angles").command_feedback_samples == 1);
    REQUIRE(channel(h, "angles").feedback_samples == 1);
    state.replace({{S::kAngles, 100ms, AngleFeedbackPolicy::kCommandFeedbackOnly}});
    state.read_succeeded(S::kAngles, false);
    state.publish("angle", sample);
    h = health(state);
    REQUIRE(channel(h, "angles").completed_reads == 1);
    REQUIRE(channel(h, "angles").feedback_samples == 1);
    REQUIRE(channel(h, "angles").command_feedback_samples == 1);
    REQUIRE_FALSE(state.replace({{S::kAngles, 100ms, AngleFeedbackPolicy::kCommandFeedbackOnly}}));
    REQUIRE(channel(health(state), "angles").feedback_samples == 1);
    state.stop_polling();
    REQUIRE(channel(health(state), "angles").feedback_samples == 0);
}

TEST_CASE("Python force field intent shares one descriptor query at the fastest rate", "[comm][python-phase5]") {
    unified_detail::PythonState state(config(Model::kO30));
    const auto plan = PollingPlan{force({{"fn", 20ms}, {"fns", 50ms}})};
    REQUIRE(state.replace(plan));
    state.read_succeeded(S::kForce, true);
    auto h = health(state);
    REQUIRE(h.force_fields.size() == 7);
    REQUIRE(field(h, "fn").requested_hz == 50);
    REQUIRE(field(h, "fns").requested_hz == 20);
    REQUIRE(field(h, "fn").query_hz == 50);
    REQUIRE(field(h, "fns").query_hz == 50);
    REQUIRE_FALSE(field(h, "fn").supported.has_value());
    REQUIRE(field(h, "fn").achieved_hz == 0);
    REQUIRE(channel(h, "force").completed_reads == 1);
    REQUIRE_FALSE(state.replace(plan));
    REQUIRE(channel(health(state), "force").completed_reads == 1);
    ForceSample observed;
    observed.sensors.resize(5);
    ForceFieldSample f;
    f.name = "fns";
    observed.fields.push_back(f);
    state.observe_force(observed);
    h = health(state);
    REQUIRE(field(h, "fn").supported == false);
    REQUIRE(field(h, "fns").supported == true);
    REQUIRE(field(h, "fn").requested_hz == 50);
    REQUIRE(field(h, "fn").query_hz == 20);
    REQUIRE(field(h, "fn").achieved_hz == 0);
    REQUIRE(field(h, "fns").achieved_hz == Catch::Approx(channel(h, "force").achieved_hz));
    observed.fields.clear();
    state.observe_force(observed);
    h = health(state);
    REQUIRE_FALSE(channel(h, "force").polling_active);
    REQUIRE(channel(h, "force").completed_reads == 1);
    REQUIRE(field(h, "fns").requested_hz == 20);
    REQUIRE(field(h, "fns").query_hz == 0);
    REQUIRE(field(h, "fns").achieved_hz == 0);
    state.stop_polling();
    REQUIRE(field(health(state), "fns").requested_hz == 0);
}

TEST_CASE("Python L20Lite scalar force fields retain independent query rates", "[comm][python-phase5]") {
    unified_detail::PythonState state(config(Model::kL20Lite));
    state.replace({force({{"fns", 100ms}, {"fts", 50ms}, {"px", 200ms}})});
    state.read_succeeded(S::kForceFts, true);
    const auto h = health(state);
    REQUIRE(field(h, "fns").query_channel == S::kForceFns);
    REQUIRE(field(h, "fts").query_channel == S::kForceFts);
    REQUIRE(field(h, "fts").query_hz == 20);
    REQUIRE(field(h, "fns").query_hz == 10);
    REQUIRE(channel(h, "force_fts").completed_reads == 1);
    REQUIRE(channel(h, "force_fns").completed_reads == 0);
    REQUIRE(field(h, "fts").achieved_hz > 0);
    REQUIRE(field(h, "fns").achieved_hz == 0);
}

TEST_CASE("Python field polling rejects invalid intent without replacing the current plan", "[comm][python-phase5]") {
    unified_detail::PythonState state(config(Model::kO30));
    const auto plan = PollingPlan{force({{"fn", 100ms}, {"fns", 50ms}})};
    state.replace(plan);
    state.read_succeeded(S::kForce, true);
    REQUIRE_THROWS_AS(state.replace({force({{"Fn", 100ms}})}), ValidationError);
    REQUIRE_THROWS_AS(state.replace({force({{"fn", 0ms}})}), PollingPlanError);
    REQUIRE_THROWS_AS(state.replace({force({{"fn", 100ms}, {"fn", 50ms}})}), PollingPlanError);
    REQUIRE_THROWS_AS(state.replace({force({{"fn", std::chrono::milliseconds::max()}})}), PollingPlanError);
    REQUIRE(channel(health(state), "force").completed_reads == 1);
    REQUIRE(field(health(state), "fn").requested_hz == 10);
    unified_detail::PythonState classic(config());
    REQUIRE_THROWS_AS(classic.replace({force({{"fts", 100ms}})}), UnsupportedOperationError);
}

TEST_CASE("Python composite transaction latency counts success once and never partial stages",
          "[comm][python-phase5]") {
    auto t = std::make_unique<validation::FakeTransport>();
    auto *raw = t.get();
    auto session = linkerbot::transport::Session::own(std::move(t));
    auto runtime = runtime::DeviceRuntime::acquire(session, "health-composite", {{0x28, false}});
    raw->on_request(0x28, validation::l6_reply);
    unified_detail::PythonHealthMetrics metrics;
    auto c = config();
    c.response_timeout = 100ms;
    const auto execute = [&] {
        unified_detail::execute_python_transactions(
            *runtime, c,
            [&] {
                for (int i = 0; i < 3; ++i) {
                    (void)runtime->execute(o6::protocol::read_angle(0x28), runtime::deadline_after(1s));
                }
            },
            runtime::Deadline::max(), {}, false, metrics);
    };
    execute();
    HandHealth h;
    metrics.project(h);
    REQUIRE(h.transaction_latency.count == 1);
    REQUIRE(runtime->health().transaction_latency.count == 3);
    const auto last = h.last_successful_transaction;
    for (int failure = 1; failure <= 2; ++failure) {
        int requests = 0;
        raw->on_request(0x28, [&](const auto &request) {
            if (++requests == 2) {
                return std::vector<linkerbot::can::Message>{};
            }
            return validation::l6_reply(request);
        });
        REQUIRE_THROWS_AS(execute(), ResponseTimeoutError);
        metrics.project(h);
        REQUIRE(h.transaction_latency.count == 1);
        REQUIRE(h.last_successful_transaction == last);
        REQUIRE(runtime->health().consecutive_errors == static_cast<uint64_t>(failure));
    }
    raw->on_request(0x28, validation::l6_reply);
    execute();
    metrics.project(h);
    REQUIRE(h.transaction_latency.count == 2);
    REQUIRE(runtime->health().consecutive_errors == 0);
    runtime->close();
    session->close();
}

TEST_CASE("Python logical latency includes admitted transport waiting and physical completion",
          "[comm][python-phase5]") {
    auto t = std::make_unique<validation::FakeTransport>();
    auto *raw = t.get();
    auto session = linkerbot::transport::Session::own(std::move(t));
    auto runtime = runtime::DeviceRuntime::acquire(session, "health-queue", {{0x28, false}});
    raw->on_request(0x28, validation::l6_reply);
    raw->arm_tx_gate();
    unified_detail::PythonHealthMetrics metrics;
    const auto c = config();
    auto pending = std::async(std::launch::async, [&] {
        unified_detail::execute_python_transactions(
            *runtime, c, [&] { (void)runtime->execute(o6::protocol::read_angle(0x28), runtime::deadline_after(1s)); },
            runtime::Deadline::max(), {}, false, metrics);
    });
    const bool entered = raw->wait_until_tx_entered(1s);
    const auto waited = pending.wait_for(50ms);
    raw->release_tx_gate();
    pending.get();
    REQUIRE(entered);
    REQUIRE(waited == std::future_status::timeout);
    HandHealth h;
    metrics.project(h);
    REQUIRE(h.transaction_latency.count == 1);
    REQUIRE(test_support::require_value(h.transaction_latency.p50) >= 50ms);
    runtime->close();
    session->close();
}

TEST_CASE("Python caller timeout does not discard a later shared logical completion", "[comm][python-phase5]") {
    std::promise<void> sent;
    validation::FakeHand d(config());
    const auto count = d.hand.get_health().transaction_latency.count;
    d.transport->on_request(0x28, [&](const auto &request) {
        if (request.data[0] == 1 && request.dlc == 1) {
            sent.set_value();
            return std::vector<linkerbot::can::Message>{};
        }
        return validation::l6_reply(request);
    });
    auto wait = std::async(std::launch::async, [&] {
        try {
            (void)d.hand.get_angles(20ms);
            return false;
        } catch (const TimeoutError &) {
            return true;
        }
    });
    REQUIRE(sent.get_future().wait_for(1s) == std::future_status::ready);
    const bool timed_out = wait.get();
    linkerbot::can::Message request{};
    request.id = 0x28;
    request.dlc = 1;
    request.data[0] = 1;
    d.transport->inject(validation::l6_reply(request).front());
    const auto deadline = runtime::Clock::now() + 1s;
    while (d.hand.get_health().transaction_latency.count == count && runtime::Clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    REQUIRE(timed_out);
    REQUIRE(d.hand.get_health().transaction_latency.count == count + 1);
}

TEST_CASE("Python logical health is device-local on a shared Session", "[comm][python-phase5]") {
    auto session = linkerbot::transport::Session::create([] {
        auto t = std::make_unique<linkerbot::can::FakeTransport>();
        t->on_request(0x28, validation::l6_reply);
        t->on_request(0x27, [](const auto &request) {
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
        return t;
    });
    auto c = config();
    HandDevice left(c, session);
    c.side = HandSide::kRight;
    HandDevice right(c, session);
    left.open();
    right.open();
    REQUIRE(left.get_health().transaction_latency.count == 4);
    REQUIRE(right.get_health().transaction_latency.count == 4);
    (void)left.get_faults(1s);
    REQUIRE(left.get_health().transaction_latency.count == 5);
    REQUIRE(right.get_health().transaction_latency.count == 4);
    right.close();
    REQUIRE(right.get_health().transaction_latency.count == 4);
    (void)left.get_angles(1s);
    REQUIRE(left.get_health().transaction_latency.count == 6);
    right.open();
    REQUIRE(right.get_health().transaction_latency.count == 8);
    REQUIRE(left.get_health().transaction_latency.count == 6);
    right.close();
    left.close();
    session->close();
}

TEST_CASE("Python identity wire completion precedes fingerprint rejection", "[comm][python-phase5]") {
    validation::FakeHand device(config());
    const auto baseline = device.hand.get_health().transaction_latency.count;
    device.transport->on_request(0x28, [](const auto &request) {
        auto replies = validation::l6_reply(request);
        if (request.data[0] == 0xC0) {
            for (auto &reply : replies) {
                if (reply.data[1] == 1) {
                    reply.data[4] = '9';
                }
            }
        }
        return replies;
    });
    REQUIRE_THROWS_AS(device.hand.refresh_device_info(1s), IdentityMismatchError);
    REQUIRE(device.hand.get_health().transaction_latency.count == baseline + 1);
    REQUIRE_FALSE(device.hand.is_open());
}

TEST_CASE("Python invalid field plan preserves active feedback plan without sending", "[comm][python-phase5]") {
    validation::FakeHand device(config());
    device.hand.start_polling({{S::kAngles, 100ms, AngleFeedbackPolicy::kCommandFeedbackOnly}});
    const auto count = device.transport->sent().size();
    REQUIRE_THROWS_AS(device.hand.start_polling({force({{"Fn", 10ms}})}), ValidationError);
    REQUIRE_THROWS_AS(device.hand.start_polling({force({{"fts", 10ms}})}), UnsupportedOperationError);
    REQUIRE(device.transport->sent().size() == count);
    REQUIRE(device.hand.get_health().polling_active);
    REQUIRE(channel(device.hand.get_health(), "angles").feedback_policy == AngleFeedbackPolicy::kCommandFeedbackOnly);
    device.hand.stop_polling();
}

namespace {
    template <typename Predicate> bool await_health(HandDevice &device, Predicate predicate) {
        const auto deadline = runtime::Clock::now() + 2s;
        do {
            if (predicate(device.get_health())) {
                return true;
            }
            std::this_thread::sleep_for(1ms);
        } while (runtime::Clock::now() < deadline);
        return false;
    }
    auto drop_angle_feedback(const linkerbot::can::Message &request) {
        return request.data[0] == 1 && request.dlc > 1 ? std::vector<linkerbot::can::Message>{}
                                                       : validation::l6_reply(request);
    }
}

TEST_CASE("Python missing command feedback retains dispatch without successful latency", "[comm][python-phase5]") {
    auto c = config();
    c.response_timeout = 20ms;
    validation::FakeHand device(c);
    device.hand.start_polling({{S::kAngles, 1s, AngleFeedbackPolicy::kCommandFeedbackOnly}});
    const auto before = device.hand.get_health();
    device.transport->on_request(0x28, drop_angle_feedback);
    for (uint64_t attempt = 1; attempt <= 2; ++attempt) {
        auto command = device.hand.set_angles(std::vector<int32_t>(6, 10));
        validation::complete(command);
        REQUIRE(await_health(device.hand, [&](const auto &h) {
            return channel(h, "angles").timeouts == attempt && h.consecutive_errors == attempt;
        }));
        const auto health = device.hand.get_health();
        REQUIRE(health.transaction_latency.count == before.transaction_latency.count);
        REQUIRE(health.last_successful_transaction == before.last_successful_transaction);
        REQUIRE(test_support::require_value(health.recent_error).code == ErrorCode::kTimeout);
        REQUIRE(channel(health, "angles").command_feedback_samples == 0);
        REQUIRE(channel(health, "angles").feedback_samples == 0);
        REQUIRE(command.stage() == CommandStage::kDispatched);
        REQUIRE_FALSE(command.error());
        REQUIRE(command.send_results().size() == 1);
    }
    device.transport->on_request(0x28, validation::l6_reply);
    auto successful = device.hand.set_angles(std::vector<int32_t>(6, 11));
    validation::complete(successful);
    REQUIRE(await_health(device.hand, [&](const auto &h) {
        return h.transaction_latency.count == before.transaction_latency.count + 1;
    }));
    const auto recovered = device.hand.get_health();
    REQUIRE(recovered.consecutive_errors == 0);
    REQUIRE(channel(recovered, "angles").command_feedback_samples == 1);
    REQUIRE(successful.stage() == CommandStage::kDispatched);
    device.hand.stop_polling();
}

TEST_CASE("Python missing command feedback records a logical error without polling", "[comm][python-phase5]") {
    auto c = config();
    c.response_timeout = 20ms;
    validation::FakeHand device(c);
    const auto before = device.hand.get_health();
    const auto sample = test_support::require_value(device.hand.get_cached_snapshot().angles);
    device.transport->on_request(0x28, drop_angle_feedback);
    auto command = device.hand.set_angles(std::vector<int32_t>(6, 10));
    validation::complete(command);
    REQUIRE(await_health(device.hand, [](const auto &h) { return h.consecutive_errors == 1; }));
    const auto health = device.hand.get_health();
    REQUIRE_FALSE(health.polling_active);
    REQUIRE(channel(health, "angles").timeouts == 0);
    REQUIRE(health.transaction_latency.count == before.transaction_latency.count);
    REQUIRE(health.last_successful_transaction == before.last_successful_transaction);
    REQUIRE(test_support::require_value(device.hand.get_cached_snapshot().angles).meta.sequence ==
            sample.meta.sequence);
    REQUIRE(command.stage() == CommandStage::kDispatched);
    REQUIRE_FALSE(command.error());
}

TEST_CASE("Native missing optional feedback retains its existing success accounting", "[comm][python-phase5]") {
    validation::FakeHand device;
    const auto before = device.hand.get_health();
    device.transport->on_request(0x28, drop_angle_feedback);
    auto command = device.hand.set_angles(std::vector<int32_t>(6, 10));
    validation::complete(command);
    REQUIRE(await_health(device.hand, [&](const auto &h) {
        return h.transaction_latency.count == before.transaction_latency.count + 1;
    }));
    const auto health = device.hand.get_health();
    REQUIRE(health.consecutive_errors == before.consecutive_errors);
    REQUIRE_FALSE(health.recent_error);
    REQUIRE(command.stage() == CommandStage::kDispatched);
    REQUIRE_FALSE(command.error());
}
