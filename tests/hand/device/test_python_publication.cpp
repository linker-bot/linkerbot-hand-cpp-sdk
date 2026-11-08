#include "../../../src/hand/unified/detail/force.hpp"
#include "../../../src/hand/unified/detail/python_state.hpp"
#include "../../../src/hand/unified/detail/python_transactions.hpp"
#include "../../../validation/fake_classic_hand.hpp"
#include "hand/support/optional_value.hpp"
#include <catch2/catch_test_macros.hpp>
#include <future>
#include <linkerbot/hand/l20/protocol_backend.hpp>
#include <linkerbot/hand/l20lite/protocol_backend.hpp>
#include <linkerbot/hand/l6/protocol_backend.hpp>
#include <linkerbot/hand/o6/protocol_backend.hpp>
#include <thread>
#include <type_traits>

using namespace linkerbot::hand;
using namespace std::chrono_literals;
using unified_detail::PythonState;
namespace {
    HandDeviceConfig profile(Model model = Model::kL6) {
        auto config = validation::config();
        config.model = model;
        config.runtime_behavior = RuntimeBehavior::kPython;
        config.identity_timeout = 100ms;
        config.response_timeout = 50ms;
        return config;
    }
    runtime::Sample publication(int value, protocol::Origin origin = protocol::Origin::response) {
        return {value, true, runtime::Clock::now(), 1, 1, 1, origin};
    }
    template <class Predicate> bool eventually(Predicate predicate, std::chrono::milliseconds budget = 1s) {
        const auto end = runtime::Clock::now() + budget;
        while (!predicate()) {
            if (runtime::Clock::now() >= end) {
                return false;
            }
            std::this_thread::sleep_for(1ms);
        }
        return true;
    }
}
static_assert(std::is_same_v<decltype(&HandDevice::wait_for_snapshot),
                             HandSnapshot (HandDevice::*)(const HandSnapshot &, std::chrono::milliseconds) const>);

TEST_CASE("Python channel sequences count publications independently of cache readers", "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    const auto first = device.hand.get_cached_snapshot();
    const auto angle = test_support::require_value(first.angles).meta.sequence;
    const auto fault = test_support::require_value(first.faults).meta.sequence;
    REQUIRE((angle == 1));
    for (int i = 0; i < 3; ++i) {
        (void)device.hand.get_angles(1s);
    }
    const auto current = device.hand.get_cached_snapshot();
    REQUIRE((test_support::require_value(current.angles).meta.sequence == angle + 3));
    REQUIRE((test_support::require_value(current.faults).meta.sequence == fault));
    REQUIRE((current.snapshot_sequence >= first.snapshot_sequence + 3));
    const auto repeated = device.hand.get_cached_snapshot();
    REQUIRE((repeated.snapshot_sequence == current.snapshot_sequence));
    REQUIRE((repeated.angles->meta.sequence == current.angles->meta.sequence));
}

TEST_CASE("Python identity has its own sample sequence and retained invalid evidence", "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    const auto before = device.hand.get_cached_snapshot();
    REQUIRE((before.device_info.has_value()));
    REQUIRE((before.device_info->meta.valid));
    REQUIRE((before.device_info->meta.sequence == 1));
    const auto refreshed = device.hand.refresh_device_info(1s);
    REQUIRE((refreshed.meta.sequence == 2));
    REQUIRE((refreshed.meta.age.has_value()));
    REQUIRE((refreshed.meta.collected_at == refreshed.timestamp));
    REQUIRE((refreshed.meta.connection_generation > 0));
    const auto after = device.hand.get_cached_snapshot();
    REQUIRE((after.angles->meta.sequence == before.angles->meta.sequence));
    REQUIRE((after.snapshot_sequence > before.snapshot_sequence));
    device.hand.close();
    const auto closed = device.hand.get_cached_snapshot();
    REQUIRE((closed.device_info.has_value()));
    REQUIRE_FALSE(closed.device_info->meta.valid);
    REQUIRE((closed.device_info->serial_number == refreshed.serial_number));
}

TEST_CASE("Python publication filtering retains exact non-command revisions", "[comm][python-phase3]") {
    PythonState state(profile());
    state.publish("angle", publication(1));
    const auto before = state.view();
    state.publish("angle", publication(2, protocol::Origin::command_feedback));
    REQUIRE((state.view().sequence == before.sequence + 1));
    REQUIRE((state.view(false).sequence == before.sequence));
    REQUIRE((std::any_cast<int>(state.view(false).samples.at("angle").value) == 1));
    state.publish("fault", publication(3));
    const auto non_command = state.view(false);
    REQUIRE((non_command.sequence == before.sequence + 2));
    REQUIRE((std::any_cast<int>(non_command.samples.at("angle").value) == 2));
}

TEST_CASE("Python filtered waiter does not busy-return for a newer command-only snapshot", "[comm][python-phase3]") {
    PythonState state(profile());
    state.publish("angle", publication(1));
    state.publish("angle", publication(2, protocol::Origin::command_feedback));
    auto waited =
        std::async(std::launch::async, [&] { state.wait(state.view().sequence, runtime::Clock::now() + 1s, false); });
    REQUIRE((waited.wait_for(20ms) == std::future_status::timeout));
    state.stop();
    REQUIRE((waited.wait_for(1s) == std::future_status::ready));
    waited.get();
}

TEST_CASE("Python retained-cache waiter wakes on close without device I/O", "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    const auto previous = device.hand.get_cached_snapshot();
    const auto count = device.transport->sent().size();
    auto waiter =
        std::async(std::launch::async, [&] { return device.hand.wait_for_cached_snapshot(previous, 2s, false); });
    REQUIRE((waiter.wait_for(20ms) == std::future_status::timeout));
    device.hand.close();
    REQUIRE((waiter.wait_for(1s) == std::future_status::ready));
    const auto stopped = waiter.get();
    REQUIRE_FALSE(stopped.angles->meta.valid);
    REQUIRE((count > 0));
}

TEST_CASE("Python optional failure degradation counts sent transactions and keeps peers ready",
          "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    device.transport->on_request(0x28, [](const auto &request) {
        if (request.data[0] == 0x36) {
            return std::vector<linkerbot::can::Message>{};
        }
        return validation::l6_reply(request);
    });
    REQUIRE_THROWS_AS(device.hand.get_currents(1s), TimeoutError);
    REQUIRE_THROWS_AS(device.hand.get_currents(1s), TimeoutError);
    REQUIRE_THROWS_AS(device.hand.get_currents(1s), UnsupportedFirmwareError);
    const auto count = device.transport->sent().size();
    REQUIRE_THROWS_AS(device.hand.get_currents(1s), UnsupportedFirmwareError);
    REQUIRE((device.transport->sent().size() == count));
    REQUIRE((device.hand.is_open()));
    REQUIRE((device.hand.get_angles(1s).meta.valid));
    const auto caps = device.hand.get_capabilities();
    REQUIRE_FALSE(caps.supports(TelemetrySemantic::kCurrents));
    REQUIRE((caps.supports(TelemetrySemantic::kAngles)));
}

TEST_CASE("Python verified optional channels are not disabled by later response loss", "[comm][python-phase3]") {
    PythonState state(profile());
    state.initialized();
    state.read_succeeded(TelemetrySemantic::kCurrents, false);
    for (int i = 0; i < 5; ++i) {
        state.read_failed(TelemetrySemantic::kCurrents, true, TimeoutError("lost"), false);
    }
    REQUIRE_FALSE(state.disabled(TelemetrySemantic::kCurrents));
    for (int i = 0; i < 5; ++i) {
        state.read_failed(TelemetrySemantic::kTemperatures, false, TimeoutError("not sent"), false);
    }
    REQUIRE_FALSE(state.disabled(TelemetrySemantic::kTemperatures));
    for (int i = 0; i < 5; ++i) {
        state.read_failed(TelemetrySemantic::kTemperatures, true, TransportError("offline"), false);
    }
    REQUIRE_FALSE(state.disabled(TelemetrySemantic::kTemperatures));
}

TEST_CASE("Python optional degradation is scoped to a connection generation", "[comm][python-phase3]") {
    PythonState state(profile());
    state.initialized();
    state.read_failed(TelemetrySemantic::kCurrents, true, TimeoutError("lost"), false);
    state.read_failed(TelemetrySemantic::kCurrents, true, ProtocolError("bad"), false);
    REQUIRE_THROWS_AS(state.read_failed(TelemetrySemantic::kCurrents, true, TimeoutError("lost"), false),
                      UnsupportedFirmwareError);
    REQUIRE((state.disabled(TelemetrySemantic::kCurrents)));
    state.reset();
    state.initialized();
    REQUIRE_FALSE(state.disabled(TelemetrySemantic::kCurrents));
}

TEST_CASE("Python adaptive angle feedback defers queries but query-only does not", "[comm][python-phase3]") {
    for (const auto policy : {AngleFeedbackPolicy::kAdaptive, AngleFeedbackPolicy::kQueryOnly}) {
        PythonState state(profile());
        REQUIRE((state.replace({{TelemetrySemantic::kAngles, 200ms, policy}})));
        const auto due = *state.next_poll();
        state.publish("angle", publication(1, protocol::Origin::command_feedback));
        REQUIRE(((*state.next_poll() > due) == (policy == AngleFeedbackPolicy::kAdaptive)));
        HandHealth health;
        state.project_health(health);
        REQUIRE((health.channels.at(0).command_feedback_samples == 1));
        REQUIRE((health.channels.at(0).feedback_samples == (policy == AngleFeedbackPolicy::kAdaptive ? 1 : 0)));
    }
}

TEST_CASE("Python identical polling plans preserve deadlines and metrics", "[comm][python-phase3]") {
    PythonState state(profile());
    const PollingPlan plan{{TelemetrySemantic::kAngles, 100ms, AngleFeedbackPolicy::kAdaptive}};
    REQUIRE((state.replace(plan)));
    state.publish("angle", publication(1));
    const auto next = state.next_poll();
    REQUIRE_FALSE(state.replace(plan));
    REQUIRE((state.next_poll() == next));
    HandHealth health;
    state.project_health(health);
    REQUIRE((health.channels.at(0).feedback_samples == 1));
}

TEST_CASE("Python command-only polling emits no angle queries and rejects illegal policies", "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    const auto count = device.transport->sent().size();
    device.hand.start_polling({{TelemetrySemantic::kAngles, 10ms, AngleFeedbackPolicy::kCommandFeedbackOnly}});
    std::this_thread::sleep_for(30ms);
    REQUIRE((device.transport->sent().size() == count));
    REQUIRE_THROWS_AS(
        device.hand.start_polling({{TelemetrySemantic::kAngles, 10ms, static_cast<AngleFeedbackPolicy>(255)}}),
        PollingPlanError);
    REQUIRE((device.hand.get_health().polling_active));
    device.hand.stop_polling();
}

TEST_CASE("Python filtered facade wait retains command feedback until a query wakes it", "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    const auto before = device.hand.get_cached_snapshot();
    auto waiter =
        std::async(std::launch::async, [&] { return device.hand.wait_for_cached_snapshot(before, 2s, false); });
    auto command = device.hand.set_angles(std::vector<int32_t>(6, 11));
    validation::complete(command);
    REQUIRE((eventually([&] {
        return device.hand.get_cached_snapshot().angles->meta.delivery == TelemetryDelivery::kCommandResponse;
    })));
    REQUIRE((waiter.wait_for(20ms) == std::future_status::timeout));
    (void)device.hand.get_faults(1s);
    REQUIRE((waiter.wait_for(1s) == std::future_status::ready));
    const auto value = waiter.get();
    REQUIRE((value.snapshot_sequence > before.snapshot_sequence));
    REQUIRE((value.angles->raw == std::vector<double>(6, 11)));
}

TEST_CASE("force aggregation replaces one query and updates scalar raw evidence", "[comm][python-phase3]") {
    ForceSample previous, next;
    ForceFieldSample fns;
    fns.name = "fns";
    fns.query_channel = TelemetrySemantic::kForceFns;
    fns.raw = {std::vector<double>{1}};
    previous.raw = {1};
    previous.fields = {fns};
    fns.raw = {std::vector<double>{2}};
    next.raw = {2};
    next.fields = {fns};
    unified_detail::merge_python_force_samples(previous, next);
    REQUIRE((previous.raw == std::vector<uint8_t>{2}));
    REQUIRE((previous.fields.front().raw.front() == std::vector<double>{2}));
    ForceSample descriptor;
    ForceFieldSample fn;
    fn.name = "fn";
    fn.query_channel = TelemetrySemantic::kForce;
    fns.query_channel = TelemetrySemantic::kForce;
    previous.fields = {fn, fns};
    descriptor.fields = {fn};
    descriptor.unsupported_fields = {"fns"};
    unified_detail::merge_python_force_samples(previous, descriptor);
    REQUIRE((previous.field("fns") == nullptr));
    REQUIRE((previous.status("fns") == "unsupported"));
}

TEST_CASE("Python tactile wrapper rejects matching malformed rows and ignores unrelated frames",
          "[comm][python-phase3]") {
    for (const auto model : {Model::kL6, Model::kO6, Model::kL20Lite, Model::kL20}) {
        runtime::TransactionPlan native;
        native.domain = "force";
        linkerbot::can::Message request{};
        request.id = 0x28;
        request.dlc = 2;
        request.data[0] = 0xB1;
        native.frames = {request};
        native.consume = [](const auto &) { return protocol::Progress::pending(); };
        auto plan = unified_detail::python_transaction(native, profile(model));
        auto short_row = request;
        short_row.dlc = 3;
        REQUIRE_THROWS_AS(plan.consume(short_row), ProtocolError);
        short_row.id = 0x27;
        REQUIRE((plan.consume(short_row).kind == protocol::ProgressKind::unmatched));
        auto bad_row = request;
        bad_row.dlc = model == Model::kO6 ? 6 : 8;
        bad_row.data[1] = 0xFF;
        REQUIRE_THROWS_AS(plan.consume(bad_row), ProtocolError);
        REQUIRE((native.consume(bad_row).kind == protocol::ProgressKind::pending));
    }
}

TEST_CASE("Python command-feedback timeout retains the successful sample and sequence", "[comm][python-phase3]") {
    PythonState state(profile());
    state.replace({{TelemetrySemantic::kAngles, 100ms, AngleFeedbackPolicy::kAdaptive}});
    state.publish("angle", publication(1));
    const auto before = state.view();
    state.publish("angle", {});
    const auto after = state.view();
    REQUIRE((after.sequence == before.sequence));
    REQUIRE((after.metadata.at(TelemetrySemantic::kAngles).sequence == 1));
    REQUIRE((std::any_cast<int>(after.samples.at("angle").value) == 1));
    HandHealth health;
    state.project_health(health);
    REQUIRE((health.channels.at(0).timeouts == 1));
}

TEST_CASE("Python production tactile assemblers preserve complete data and reject conflicting rows",
          "[comm][python-phase3]") {
    for (const auto model : {Model::kL6, Model::kO6, Model::kL20Lite, Model::kL20}) {
        CAPTURE(model);
        runtime::TransactionPlan native;
        if (model == Model::kL6) {
            native = l6::protocol::read_force(0x28, 0xB1, 100ms);
        }
        if (model == Model::kO6) {
            native = o6::protocol::read_force_sensor(0x28, 0xB1, "force_sensor_thumb");
        }
        if (model == Model::kL20Lite) {
            native = l20lite::protocol::read_force_sensor(0x28, 0xB1, "force_sensor_thumb");
        }
        if (model == Model::kL20) {
            native = l20::protocol::read_force_sensor(0x28, 0xB1, "force_sensor_thumb");
        }
        auto plan = unified_detail::python_transaction(native, profile(model));
        linkerbot::can::Message row{};
        row.id = 0x28;
        row.dlc = model == Model::kO6 ? 6 : 8;
        row.data[0] = 0xB1;
        REQUIRE((plan.consume(row).kind == protocol::ProgressKind::pending));
        REQUIRE((plan.consume(row).kind == protocol::ProgressKind::pending));
        row.data[2] = 1;
        REQUIRE((plan.consume(row).kind == protocol::ProgressKind::failed));
        auto complete = unified_detail::python_transaction(native, profile(model));
        const int rows = model == Model::kO6 ? 10 : 12;
        for (int index = 0; index < rows; ++index) {
            row.data[1] = static_cast<uint8_t>(index); // Python compact row representation.
            const auto result = complete.consume(row);
            REQUIRE((result.kind ==
                     (index + 1 == rows ? protocol::ProgressKind::complete : protocol::ProgressKind::pending)));
        }
    }
}

TEST_CASE("Python O30 force absence degrades immediately without changing other capabilities",
          "[comm][python-phase3]") {
    auto config = profile(Model::kO30);
    config.o30_layout = O30Layout::kPythonV25;
    PythonState state(config);
    state.initialized();
    ErrorContext context;
    context.native_status = 0x21;
    REQUIRE_THROWS_AS(
        state.read_failed(TelemetrySemantic::kForce, true, ProtocolError("object unavailable", context), false),
        UnsupportedFirmwareError);
    const auto caps = state.capabilities(declared_capabilities(config));
    REQUIRE_FALSE(caps.supports(TelemetrySemantic::kForce));
    REQUIRE((caps.supports(TelemetrySemantic::kAngles)));
    HandSnapshot snapshot;
    REQUIRE_NOTHROW(state.project(snapshot, state.view(), caps));
}

TEST_CASE("Python fulfilled feedback cancels only the still-unclaimed poll ticket", "[comm][python-phase3]") {
    for (const auto policy : {AngleFeedbackPolicy::kAdaptive, AngleFeedbackPolicy::kQueryOnly}) {
        PythonState state(profile());
        state.replace({{TelemetrySemantic::kAngles, 1s, policy}});
        const auto ticket = state.take_poll();
        REQUIRE((ticket.has_value()));
        REQUIRE((state.poll_needed(*ticket)));
        state.publish("angle", publication(2, protocol::Origin::command_feedback));
        REQUIRE((state.poll_needed(*ticket) == (policy == AngleFeedbackPolicy::kQueryOnly)));
        state.stop_polling();
        REQUIRE_FALSE(state.poll_needed(*ticket));
    }
}

TEST_CASE("Python retained publication view rejects undeclared and partial domains", "[comm][python-phase3]") {
    PythonState state(profile());
    state.publish("external/domain", publication(1));
    state.publish("tactile/177", publication(2));
    REQUIRE((state.view().samples.empty()));
    REQUIRE((state.view().sequence == 0));
    state.publish("angle", publication(3));
    REQUIRE((state.view().samples.size() == 1));
    REQUIRE((state.view().sequence == 1));
}

TEST_CASE("Python response-cache reads preserve publication metadata without sending", "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    const auto initial = device.transport->sent().size();
    REQUIRE_THROWS_AS(device.hand.get_configured_speeds(1s), UnsupportedOperationError);
    REQUIRE((device.transport->sent().size() == initial));
    validation::complete(device.hand.set_speeds(std::vector<double>(6, 12)));
    const auto count = device.transport->sent().size();
    const auto before = device.hand.get_cached_snapshot();
    const auto first = device.hand.get_configured_speeds(1s);
    const auto second = device.hand.get_configured_speeds(1s);
    REQUIRE((first.raw == std::vector<double>(6, 12)));
    REQUIRE((first.meta.sequence == 1));
    REQUIRE((second.meta.sequence == first.meta.sequence));
    REQUIRE((device.hand.get_cached_snapshot().snapshot_sequence == before.snapshot_sequence));
    REQUIRE((device.transport->sent().size() == count));
}

TEST_CASE("Python angle dispatch is observable while response observation retains its device lane",
          "[comm][python-phase3]") {
    auto config = profile();
    config.response_timeout = 2s;
    validation::FakeHand device(config);
    device.transport->on_request(0x28, [](const auto &request) {
        if (request.data[0] == 1 && request.dlc > 1) {
            return std::vector<linkerbot::can::Message>{};
        }
        return validation::l6_reply(request);
    });
    auto command = device.hand.set_angles(std::vector<int32_t>(6, 19));
    validation::complete(command);
    const auto sent = device.transport->sent().size();
    auto read = std::async(std::launch::async, [&] { return device.hand.get_angles(2s); });
    const auto blocked = read.wait_for(20ms);
    const auto count = device.transport->sent().size();
    linkerbot::can::Message response{};
    response.id = 0x28;
    response.dlc = 7;
    response.data[0] = 1;
    std::fill_n(response.data.begin() + 1, 6, uint8_t{19});
    device.transport->inject(response);
    const auto value = read.get();
    REQUIRE((blocked == std::future_status::timeout));
    REQUIRE((count == sent));
    REQUIRE((value.raw == std::vector<double>(6, 64)));
    REQUIRE((command.stage() == CommandStage::kDispatched));
}

TEST_CASE("Python adaptive feedback discards a redundant poll queued behind a command", "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    auto command = device.block();
    device.hand.start_polling({{TelemetrySemantic::kAngles, 1s, AngleFeedbackPolicy::kAdaptive}});
    std::this_thread::sleep_for(20ms);
    device.transport->release_tx_gate();
    validation::complete(command);
    REQUIRE((eventually([&] {
        return device.hand.get_cached_snapshot().angles->meta.delivery == TelemetryDelivery::kCommandResponse;
    })));
    std::this_thread::sleep_for(20ms);
    device.hand.stop_polling();
    const auto frames = device.transport->sent();
    const auto queries = std::count_if(frames.begin(), frames.end(),
                                       [](const auto &frame) { return frame.data[0] == 1 && frame.dlc == 1; });
    REQUIRE((queries == 1)); // Only the initialization baseline.
}

TEST_CASE("Python default force reads preserve firmware-unsupported errors without I/O", "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    device.transport->on_request(0x28, [](const auto &request) {
        if (request.data[0] == 0xB1) {
            return std::vector<linkerbot::can::Message>{};
        }
        return validation::l6_reply(request);
    });
    REQUIRE_THROWS_AS(device.hand.get_force(1s), TimeoutError);
    REQUIRE_THROWS_AS(device.hand.get_force(1s), TimeoutError);
    REQUIRE_THROWS_AS(device.hand.get_force(1s), UnsupportedFirmwareError);
    const auto sent = device.transport->sent().size();
    REQUIRE_THROWS_AS(device.hand.get_force(1s), UnsupportedFirmwareError);
    REQUIRE((device.transport->sent().size() == sent));
    REQUIRE((device.hand.is_open()));
}

TEST_CASE("Python rejects a degraded polling channel before replacing an existing plan", "[comm][python-phase3]") {
    validation::FakeHand device(profile());
    device.transport->on_request(0x28, [](const auto &request) {
        if (request.data[0] == 0x36) {
            return std::vector<linkerbot::can::Message>{};
        }
        return validation::l6_reply(request);
    });
    REQUIRE_THROWS_AS(device.hand.get_currents(1s), TimeoutError);
    REQUIRE_THROWS_AS(device.hand.get_currents(1s), TimeoutError);
    REQUIRE_THROWS_AS(device.hand.get_currents(1s), UnsupportedFirmwareError);
    device.hand.start_polling({{TelemetrySemantic::kAngles, 10ms, AngleFeedbackPolicy::kQueryOnly}});
    REQUIRE_THROWS_AS(device.hand.start_polling({{TelemetrySemantic::kCurrents, 10ms}}), PollingPlanError);
    REQUIRE((eventually([&] { return device.hand.get_cached_snapshot().angles->meta.sequence >= 2; })));
    device.hand.stop_polling();
}
