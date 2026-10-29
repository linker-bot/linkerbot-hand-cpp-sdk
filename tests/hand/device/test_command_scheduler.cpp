#include "hand/support/throw_as.hpp"
#include <catch2/catch_test_macros.hpp>

#include "../../../src/hand/unified/detail/command_handle_access.hpp"
#include "../../../src/hand/unified/detail/command_scheduler.hpp"
#include "../../../src/hand/unified/detail/execution_gate.hpp"

#include <atomic>
#include <future>
#include <set>

using namespace linkerbot::hand;
using namespace linkerbot::hand::unified_detail;

namespace {
    CommandRequest request(CommandSemantic semantic, double value,
                           std::chrono::milliseconds timeout = std::chrono::seconds{5}) {
        return {semantic, {value}, timeout, AckExpectation::kNone};
    }

    CommandHandle delivered() {
        CommandHandle result(1, CommandFamily::kAngleTarget, AckSupport::kUnsupported);
        command_detail::CommandHandleAccess::mark_dispatched(result, 1);
        return result;
    }

    struct Barrier {
        std::promise<void> entered;
        std::promise<void> release;
        std::shared_future<void> gate = release.get_future().share();
        bool released = false;
        void open() {
            if (!released) {
                released = true;
                release.set_value();
            }
        }
        ~Barrier() { open(); }
    };

    CommandFamilyMetrics totals(const std::vector<CommandFamilyMetrics> &metrics) {
        CommandFamilyMetrics total;
        for (const auto &m : metrics) {
            total.accepted += m.accepted;
            total.dispatched += m.dispatched;
            total.failed += m.failed;
            total.superseded += m.superseded;
            total.pending += m.pending;
            total.queue.depth += m.queue.depth;
        }
        return total;
    }
}

TEST_CASE("M2 latest slots preserve new sequence and FIFO semantic order", "[comm][m2][scheduler]") {
    CommandScheduler scheduler(250, 16);
    Barrier barrier; // Destroyed before scheduler even if an assertion fails.
    std::vector<std::pair<CommandSemantic, double>> executed;
    auto execute = [&](const CommandRequest &r) {
        executed.emplace_back(r.semantic, r.raw_values.front());
        return delivered();
    };
    auto first = scheduler.accept(request(CommandSemantic::kAngles, 1), AckSupport::kUnsupported, {},
                                  [&, gate = barrier.gate](const auto &r) {
                                      barrier.entered.set_value();
                                      gate.wait();
                                      return execute(r);
                                  });
    const auto entered = barrier.entered.get_future().wait_for(std::chrono::seconds{2}) == std::future_status::ready;
    if (!entered) {
        barrier.open();
    }
    REQUIRE(entered);
    auto old = scheduler.accept(request(CommandSemantic::kAngles, 2), AckSupport::kUnsupported, {}, execute);
    std::vector<CommandHandle> retained;
    for (const auto semantic :
         {CommandSemantic::kAccelerations, CommandSemantic::kAccelerations, CommandSemantic::kStallThresholds,
          CommandSemantic::kStallHoldCurrents, CommandSemantic::kStallDetectionTimes, CommandSemantic::kStallThresholds,
          CommandSemantic::kStallHoldCurrents}) {
        retained.push_back(scheduler.accept(request(semantic, retained.size()), AckSupport::kUnsupported, {}, execute));
    }
    auto newest = scheduler.accept(request(CommandSemantic::kAngles, 3), AckSupport::kUnsupported, {}, execute);
    for (const auto semantic : {CommandSemantic::kSpeeds, CommandSemantic::kCurrentLimits}) {
        auto previous = scheduler.accept(request(semantic, 4), AckSupport::kUnsupported, {}, execute);
        retained.push_back(scheduler.accept(request(semantic, 5), AckSupport::kUnsupported, {}, execute));
        REQUIRE(previous.stage() == CommandStage::kSuperseded);
        REQUIRE(previous.sent_frames() == 0);
    }
    REQUIRE_FALSE(first.done()); // Claimed requests cannot be superseded.
    REQUIRE(old.stage() == CommandStage::kSuperseded);
    REQUIRE(old.sent_frames() == 0);
    REQUIRE_FALSE(old.error());
    const auto queued = totals(scheduler.metrics());
    REQUIRE(queued.pending == 11);
    REQUIRE(queued.queue.depth == 10);
    barrier.open();
    REQUIRE(first.wait_for(std::chrono::seconds{2}));
    for (auto &handle : retained) {
        REQUIRE(handle.wait_for(std::chrono::seconds{2}));
    }
    REQUIRE(newest.wait_for(std::chrono::seconds{2}));
    REQUIRE(executed.size() == 11);
    REQUIRE(executed[1] == std::make_pair(CommandSemantic::kAccelerations, 0.0));
    REQUIRE(executed[2] == std::make_pair(CommandSemantic::kAccelerations, 1.0));
    REQUIRE(executed[3].first == CommandSemantic::kStallThresholds);
    REQUIRE(executed[4].first == CommandSemantic::kStallHoldCurrents);
    REQUIRE(executed[8] == std::make_pair(CommandSemantic::kAngles, 3.0));
    const auto done = totals(scheduler.metrics());
    REQUIRE(done.accepted == 14);
    REQUIRE(done.dispatched == 11);
    REQUIRE(done.superseded == 3);
    REQUIRE(done.pending == 0);
    REQUIRE(done.queue.depth == 0);
    for (const auto &m : scheduler.metrics()) {
        if (m.dispatched) {
            REQUIRE(m.execution_latency.count == m.dispatched);
        }
    }
}

TEST_CASE("M2 FIFO capacity excludes latest and executing requests", "[comm][m2][scheduler]") {
    CommandScheduler scheduler(250, 16);
    Barrier barrier;
    auto first = scheduler.accept(request(CommandSemantic::kAccelerations, 0), AckSupport::kUnsupported, {},
                                  [&, gate = barrier.gate](const auto &) {
                                      barrier.entered.set_value();
                                      gate.wait();
                                      return delivered();
                                  });
    REQUIRE(barrier.entered.get_future().wait_for(std::chrono::seconds{2}) == std::future_status::ready);
    std::vector<CommandHandle> handles;
    for (size_t i = 0; i < 16; ++i) {
        handles.push_back(scheduler.accept(request(CommandSemantic::kAccelerations, i), AckSupport::kUnsupported, {},
                                           [](const auto &) { return delivered(); }));
    }
    REQUIRE_THROWS_AS(scheduler.accept(request(CommandSemantic::kStallHoldCurrents, 20), AckSupport::kUnsupported, {},
                                       [](const auto &) { return delivered(); }),
                      QueueFullError);
    for (const auto semantic : {CommandSemantic::kAngles, CommandSemantic::kSpeeds, CommandSemantic::kCurrentLimits}) {
        handles.push_back(scheduler.accept(request(semantic, 1), AckSupport::kUnsupported, {},
                                           [](const auto &) { return delivered(); }));
    }
    const auto metrics = totals(scheduler.metrics());
    REQUIRE(metrics.accepted == 20);
    REQUIRE(metrics.pending == 20);
    REQUIRE(metrics.queue.depth == 19);
    barrier.open();
    REQUIRE(first.wait_for(std::chrono::seconds{2}));
    for (auto &handle : handles) {
        REQUIRE(handle.wait_for(std::chrono::seconds{2}));
    }
}

TEST_CASE("M2 pending deadlines expire independently of executor and throttle", "[comm][m2][scheduler]") {
    CommandScheduler scheduler(0.5, 1); // Throttle must not delay expiry.
    Barrier barrier;
    auto first = scheduler.accept(request(CommandSemantic::kAngles, 0), AckSupport::kUnsupported, {},
                                  [&, gate = barrier.gate](const auto &) {
                                      barrier.entered.set_value();
                                      gate.wait();
                                      return delivered();
                                  });
    REQUIRE(barrier.entered.get_future().wait_for(std::chrono::seconds{2}) == std::future_status::ready);
    std::atomic<size_t> executed{0};
    auto old = scheduler.accept(request(CommandSemantic::kAccelerations, 1, std::chrono::milliseconds{20}),
                                AckSupport::kUnsupported, {}, [&](const auto &) {
                                    ++executed;
                                    return delivered();
                                });
    REQUIRE(old.wait_for(std::chrono::seconds{2}));
    REQUIRE(old.stage() == CommandStage::kFailed);
    REQUIRE_THROWS_AS(std::rethrow_exception(old.error()), TimeoutError);
    REQUIRE(old.sent_frames() == 0);
    REQUIRE(executed.load() == 0);
    auto replacement = scheduler.accept(request(CommandSemantic::kAccelerations, 2), AckSupport::kUnsupported, {},
                                        [](const auto &) { return delivered(); });
    REQUIRE(replacement.command_id() > old.command_id());
    scheduler.invalidate(std::make_exception_ptr(ClosedError("generation ended")));
    REQUIRE(replacement.wait_for(std::chrono::seconds{1}));
    REQUIRE_THROWS_AS(std::rethrow_exception(replacement.error()), ClosedError);
    barrier.open();
    REQUIRE(first.wait_for(std::chrono::seconds{2}));
}

TEST_CASE("M2 concurrent admission executes once with bounded batch rate", "[comm][m2][scheduler]") {
    CommandScheduler scheduler(250, 64);
    std::mutex mutex;
    std::vector<HealthTimePoint> starts;
    std::vector<double> executed;
    std::vector<std::future<CommandHandle>> submitted;
    for (size_t i = 0; i < 40; ++i) {
        submitted.push_back(std::async(std::launch::async, [&, i] {
            return scheduler.accept(request(CommandSemantic::kAccelerations, i), AckSupport::kUnsupported, {},
                                    [&](const auto &r) {
                                        const std::lock_guard<std::mutex> lock(mutex);
                                        starts.push_back(HealthClock::now());
                                        executed.push_back(r.raw_values[0]);
                                        return delivered();
                                    });
        }));
    }
    std::set<uint64_t> ids;
    for (auto &future : submitted) {
        auto handle = future.get();
        ids.insert(handle.command_id());
        REQUIRE(handle.wait_for(std::chrono::seconds{3}));
        REQUIRE(handle.stage() == CommandStage::kDispatched);
    }
    REQUIRE(ids.size() == 40);
    const std::lock_guard<std::mutex> lock(mutex);
    REQUIRE(std::set<double>(executed.begin(), executed.end()).size() == 40);
    for (size_t i = 1; i < starts.size(); ++i) {
        REQUIRE(starts[i] - starts[i - 1] >= std::chrono::milliseconds{4});
    }
    REQUIRE(totals(scheduler.metrics()).accepted == 40);
    REQUIRE(totals(scheduler.metrics()).dispatched == 40);
}

TEST_CASE("M2 execution gate alternates telemetry and commands", "[comm][m2][scheduler]") {
    for (const bool initial_command : {false, true}) {
        auto gate = std::make_shared<ExecutionGate>();
        auto incumbent = gate->acquire(initial_command, runtime::Deadline::max());
        std::promise<void> command_queued, read_queued;
        std::atomic<bool> command_seen{false}, read_seen{false};
        std::mutex order_mutex;
        std::vector<bool> order;
        const auto waiter = [&](bool command, std::promise<void> &queued, std::atomic<bool> &seen) {
            auto lease = gate->acquire(command, runtime::deadline_after(std::chrono::seconds{3}), {}, [&] {
                if (!seen.exchange(true)) {
                    queued.set_value();
                }
                return true;
            });
            const std::lock_guard<std::mutex> lock(order_mutex);
            order.push_back(command);
        };
        auto command = std::async(std::launch::async, [&] { waiter(true, command_queued, command_seen); });
        auto read = std::async(std::launch::async, [&] { waiter(false, read_queued, read_seen); });
        const auto cq = command_queued.get_future().wait_for(std::chrono::seconds{1}) == std::future_status::ready;
        const auto rq = read_queued.get_future().wait_for(std::chrono::seconds{1}) == std::future_status::ready;
        incumbent.reset();
        command.get();
        read.get();
        REQUIRE(cq);
        REQUIRE(rq);
        REQUIRE(order == std::vector<bool>{!initial_command, initial_command});
    }
}

TEST_CASE("M2 background failures wake every waiter and leave executor usable", "[comm][m2][scheduler]") {
    CommandScheduler scheduler(250, 16);
    Barrier barrier;
    auto failed = scheduler.accept(
        request(CommandSemantic::kAccelerations, 0), AckSupport::kUnsupported,
        {"set_accelerations", "O30", "acceleration_targets", 7}, [&, gate = barrier.gate](const auto &) {
            barrier.entered.set_value();
            gate.wait();
            return test_util::throw_as<CommandHandle>(TransportError("injected send failure"));
        });
    REQUIRE(barrier.entered.get_future().wait_for(std::chrono::seconds{2}) == std::future_status::ready);
    REQUIRE_FALSE(failed.wait_for(std::chrono::milliseconds{1}));
    std::vector<std::future<CommandStage>> waiters;
    for (size_t i = 0; i < 8; ++i) {
        waiters.push_back(std::async(std::launch::async, [failed] { return failed.wait(); }));
    }
    barrier.open();
    for (auto &waiter : waiters) {
        REQUIRE(waiter.get() == CommandStage::kFailed);
    }
    REQUIRE_THROWS_AS(std::rethrow_exception(failed.error()), TransportError);
    try {
        std::rethrow_exception(failed.error());
    } catch (const HandError &e) {
        REQUIRE(e.context().operation == "set_accelerations");
        REQUIRE(e.context().generation == 7);
    }
    auto success = scheduler.accept(request(CommandSemantic::kAccelerations, 1), AckSupport::kUnsupported, {},
                                    [](const auto &) { return delivered(); });
    REQUIRE(success.wait_for(std::chrono::seconds{2}));
    REQUIRE(success.stage() == CommandStage::kDispatched);
    REQUIRE(totals(scheduler.metrics()).failed == 1);
    REQUIRE(totals(scheduler.metrics()).dispatched == 1);
}
