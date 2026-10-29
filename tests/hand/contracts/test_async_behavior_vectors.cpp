#include "../../../src/hand/unified/detail/command_handle_access.hpp"
#include "../../../src/hand/unified/detail/command_scheduler.hpp"
#include "hand/support/throw_as.hpp"
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <future>

using namespace linkerbot::hand;
using namespace linkerbot::hand::unified_detail;
using namespace std::chrono_literals;

TEST_CASE("admission deadline boundary is independent of caller waits and a blocked executor",
          "[comm][contracts][async][clock]") {
    const auto origin = CommandScheduler::Clock::now();
    std::atomic<int> elapsed_ms{0};
    CommandScheduler scheduler(250, 3, [&] { return origin + std::chrono::milliseconds{elapsed_ms.load()}; });
    // Gate lifetime precedes scheduler destruction even on an assertion failure.
    std::promise<void> entered, release;
    const auto gate = release.get_future().share();
    struct Release {
        std::promise<void> &promise;
        ~Release() { promise.set_value(); }
    } cleanup{release};
    auto blocked =
        scheduler.accept({CommandSemantic::kAngles, {10}, 5s, AckExpectation::kNone}, AckSupport::kUnsupported, {},
                         [&, gate](const CommandRequest &) {
                             entered.set_value();
                             gate.wait();
                             CommandHandle delivered(1, CommandFamily::kAngleTarget, AckSupport::kUnsupported);
                             command_detail::CommandHandleAccess::mark_dispatched(delivered, 1);
                             return delivered;
                         });
    REQUIRE(entered.get_future().wait_for(2s) == std::future_status::ready);
    std::atomic<int> executed{0};
    auto pending = scheduler.accept({CommandSemantic::kStallThresholds, {10}, 20ms, AckExpectation::kRequired},
                                    AckSupport::kRequired, {}, [&](const auto &) {
                                        ++executed;
                                        return test_util::throw_as<CommandHandle>(
                                            std::runtime_error("expired work must never execute"));
                                    });
    REQUIRE_FALSE(pending.wait_for(1ms));
    REQUIRE_FALSE(pending.done());
    elapsed_ms = 19;
    scheduler.notify_clock_advanced();
    REQUIRE_FALSE(pending.done());
    elapsed_ms = 20;
    scheduler.notify_clock_advanced();
    REQUIRE(pending.done());
    REQUIRE(pending.stage() == CommandStage::kFailed);
    REQUIRE_THROWS_AS(std::rethrow_exception(pending.error()), TimeoutError);
    REQUIRE(pending.sent_frames() == 0);
    REQUIRE(executed == 0);
    REQUIRE_FALSE(blocked.done());
    // Repeated clock notifications cannot reset budget or mutate terminal history.
    const auto history_size = pending.history().size();
    elapsed_ms = 100;
    scheduler.notify_clock_advanced();
    REQUIRE(pending.history().size() == history_size);
    scheduler.invalidate(std::make_exception_ptr(ClosedError("test generation ended")));
    REQUIRE(blocked.stage() == CommandStage::kFailed);
}
