#include "../../../src/hand/unified/detail/read_coordinator.hpp"
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <future>

using namespace linkerbot::hand;
using namespace std::chrono_literals;

TEST_CASE("read coordinator enforces capacity without cancelling its existing reader", "[comm][python-phase2]") {
    unified_detail::ReadCoordinator<int> reads(1);
    std::promise<void> entered;
    std::promise<void> release;
    auto gate = release.get_future().share();
    std::atomic<int> executions{0};
    auto first = std::async(std::launch::async, [&] {
        return reads.read(1, std::chrono::steady_clock::now() + 2s, [&] {
            ++executions;
            entered.set_value();
            gate.wait();
            return 42;
        });
    });
    entered.get_future().wait();
    bool rejected = false;
    try {
        (void)reads.read(1, std::chrono::steady_clock::now() + 1s, [] { return 99; });
    } catch (const QueueFullError &) {
        rejected = true;
    }
    release.set_value();
    const auto value = first.get();
    REQUIRE((rejected));
    REQUIRE((value == 42));
    REQUIRE((executions.load() == 1));
}

TEST_CASE("read coordinator discards late results at invalidation", "[comm][python-phase2]") {
    unified_detail::ReadCoordinator<int> reads(1);
    std::promise<void> entered;
    std::promise<void> release;
    auto gate = release.get_future().share();
    auto first = std::async(std::launch::async, [&] {
        try {
            (void)reads.read(1, std::chrono::steady_clock::now() + 2s, [&] {
                entered.set_value();
                gate.wait();
                return 42;
            });
            return false;
        } catch (const ClosedError &) {
            return true;
        }
    });
    entered.get_future().wait();
    reads.invalidate();
    const bool closed = first.get();
    release.set_value();
    reads.drain();
    reads.activate();
    const auto next = reads.read(1, std::chrono::steady_clock::now() + 1s, [] { return 99; });
    REQUIRE((closed));
    REQUIRE((next == 99));
}

TEST_CASE("read errors keep caller context independent when a source exception is shared", "[comm][python-phase2]") {
    unified_detail::ReadCoordinator<int> reads(2);
    const auto source = std::make_exception_ptr(ProtocolError("shared malformed response"));
    for (const std::string operation : {"first_reader", "second_reader"}) {
        std::exception_ptr classified;
        try {
            (void)reads.read(1, std::chrono::steady_clock::now() + 1s,
                             [&]() -> int { std::rethrow_exception(source); });
        } catch (const ProtocolError &) {
            classified = classify_exception(std::current_exception(), {operation});
        }
        REQUIRE((classified));
        try {
            std::rethrow_exception(classified);
        } catch (const ProtocolError &error) {
            REQUIRE((error.context().operation == operation));
        }
    }
}
