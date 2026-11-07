#include "../../../src/hand/unified/detail/command_delivery.hpp"
#include <catch2/catch_test_macros.hpp>
#include <future>
#include <linkerbot/can/fake_transport.hpp>

using namespace linkerbot::hand;
using namespace std::chrono_literals;

TEST_CASE("close waits for accepted receipt publication before deciding a claimed outcome", "[comm][python-phase2]") {
    for (const bool allow_dispatched : {false, true}) {
        auto transport = std::make_unique<linkerbot::can::FakeTransport>();
        auto *raw = transport.get();
        auto session = linkerbot::transport::Session::own(std::move(transport));
        unified_detail::CommandDelivery delivery;
        std::promise<void> submitted;
        auto entered = submitted.get_future();
        std::promise<void> publish;
        auto gate = publish.get_future().share();
        raw->arm_tx_gate();
        auto sender = std::async(std::launch::async, [&] {
            return delivery.admit(
                [&] {
                    linkerbot::can::Message frame{};
                    frame.id = 0x28;
                    frame.dlc = 1;
                    auto receipt = session->submit({{frame}});
                    submitted.set_value();
                    gate.wait();
                    return receipt;
                },
                true);
        });
        const auto accepted = entered.wait_for(1s);
        const bool claimed = raw->wait_until_tx_entered(1s);
        std::promise<void> closing;
        auto close_started = closing.get_future();
        auto closer = std::async(std::launch::async, [&] {
            closing.set_value();
            return delivery.preserve_claimed(allow_dispatched);
        });
        close_started.wait();
        const auto pending = closer.wait_for(20ms);
        publish.set_value();
        const auto receipt = sender.get();
        const bool preserved = closer.get();
        raw->release_tx_gate();
        receipt.wait();
        session->close();
        REQUIRE((accepted == std::future_status::ready));
        REQUIRE((claimed));
        REQUIRE((pending == std::future_status::timeout));
        REQUIRE((preserved == allow_dispatched));
        REQUIRE((delivery.draining_claimed()));
        REQUIRE((delivery.retained().has_value() == allow_dispatched));
        REQUIRE((receipt.result().outcome == linkerbot::transport::SendOutcome::dispatched));
        int later_submissions = 0;
        REQUIRE_THROWS_AS(delivery.admit(
                              [&]() -> linkerbot::transport::SendReceipt {
                                  ++later_submissions;
                                  return {};
                              },
                              true),
                          ClosedError);
        REQUIRE((later_submissions == 0));
    }
}

TEST_CASE("close before admission rejects the batch without invoking Session submission", "[comm][python-phase2]") {
    unified_detail::CommandDelivery delivery;
    REQUIRE_FALSE(delivery.preserve_claimed());
    int submissions = 0;
    REQUIRE_THROWS_AS(delivery.admit(
                          [&]() -> linkerbot::transport::SendReceipt {
                              ++submissions;
                              return {};
                          },
                          true),
                      ClosedError);
    REQUIRE((submissions == 0));
}
