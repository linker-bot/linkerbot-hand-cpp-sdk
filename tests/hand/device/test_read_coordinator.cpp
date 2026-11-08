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

namespace {
    struct PublicationBarrier {
        std::promise<void> entered;
        std::promise<void> release;
        std::shared_future<void> gate = release.get_future().share();
        bool fail = false;
    };
    struct PublicationValue {
        explicit PublicationValue(std::shared_ptr<PublicationBarrier> barrier) : state(std::move(barrier)) {}
        PublicationValue(const PublicationValue &) = default;
        PublicationValue &operator=(const PublicationValue &) = default;
        PublicationValue &operator=(PublicationValue &&) = default;
        // Intentionally throwing move: simulate failure while publishing a value.
        PublicationValue(PublicationValue &&other) noexcept(false) : state(other.state), stage(other.stage + 1) {
            // The first move installs the worker's local result. The second
            // publishes it under the coordinator lock, delaying the waiter's
            // wakeup until after its deadline without a scheduler race.
            if (other.stage == 1) {
                state->entered.set_value();
                state->gate.wait();
                if (state->fail) {
                    throw ClosedError("publication failed after the caller deadline");
                }
            }
        }
        std::shared_ptr<PublicationBarrier> state;
        unsigned stage = 0;
    };
}

TEST_CASE("read deadlines precede results and errors published after their wait window", "[comm][python-phase2]") {
    for (const bool fail : {false, true}) {
        unified_detail::ReadCoordinator<PublicationValue> reads(1);
        const auto barrier = std::make_shared<PublicationBarrier>();
        barrier->fail = fail;
        auto entered = barrier->entered.get_future();
        const auto deadline = std::chrono::steady_clock::now() + 300ms;
        auto reader = std::async(std::launch::async, [&] {
            try {
                (void)reads.read(1, deadline, [barrier] { return PublicationValue{barrier}; });
                return ErrorCode::kInternal;
            } catch (const HandError &error) {
                return error.code();
            }
        });
        const auto publishing = entered.wait_for(2s);
        std::this_thread::sleep_until(deadline + 20ms);
        barrier->release.set_value();
        const auto result = reader.get();
        REQUIRE((publishing == std::future_status::ready));
        REQUIRE((result == ErrorCode::kTimeout));
    }
}

TEST_CASE("fulfilled poll demand does not revoke an explicit shared reader", "[comm][python-phase3]") {
    for (const bool explicit_reader : {false, true}) {
        unified_detail::ReadCoordinator<int> reads(1);
        std::atomic<bool> needed{true};
        std::atomic<unsigned> executions{0};
        std::promise<void> preparing, release;
        auto gate = release.get_future().share();
        auto poll = std::async(std::launch::async, [&] {
            try {
                return reads.read_prepared(
                    1, std::chrono::steady_clock::now() + 2s,
                    [&](const auto &) {
                        ++executions;
                        return 7;
                    },
                    [&](const auto &) {
                        preparing.set_value();
                        gate.wait();
                        return std::shared_ptr<void>{};
                    },
                    true, {}, [&] { return needed.load(); });
            } catch (const ClosedError &) {
                return 0;
            }
        });
        preparing.get_future().wait();
        std::future<int> reader;
        if (explicit_reader) {
            reader = std::async(std::launch::async, [&] {
                return reads.read(1, std::chrono::steady_clock::now() + 2s, [&] {
                    ++executions;
                    return 7;
                });
            });
            REQUIRE((reader.wait_for(20ms) == std::future_status::timeout));
        }
        needed = false;
        release.set_value();
        const auto polled = poll.get();
        if (explicit_reader) {
            REQUIRE((reader.get() == 7));
            REQUIRE((executions == 1));
        } else {
            REQUIRE((polled == 0));
            REQUIRE((executions == 0));
        }
    }
}
