#include "fake_classic_hand.hpp"
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>

using namespace validation;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    try {
        require(argc == 5 || (argc == 6 && std::string(argv[5]) == "python"),
                "usage: probe scenario capacity targets timeout_ms [python]");
        const bool python = argc == 6;
        const std::string scenario = argv[1];
        const auto capacity = static_cast<size_t>(std::stoul(argv[2]));
        const int targets = std::stoi(argv[3]);
        const int timeout = std::stoi(argv[4]);
        auto settings = config(capacity, scenario == "execution_timeout" ? timeout : 3000);
        if (python) {
            settings.runtime_behavior = RuntimeBehavior::kPython;
            settings.response_timeout = std::chrono::milliseconds{scenario == "execution_timeout" ? timeout : 1000};
        }
        FakeHand device(settings);
        auto &hand = device.hand;
        auto *transport = device.transport;
        if (scenario == "admission_invalid") {
            const auto before = transport->sent().size();
            bool invalid = false, unsupported = false;
            try {
                (void)hand.set_angles({1});
            } catch (const ValidationError &) {
                invalid = true;
            }
            try {
                (void)hand.set_accelerations(std::vector<double>(6, 1));
            } catch (const UnsupportedOperationError &) {
                unsupported = true;
            }
            require(invalid && unsupported && transport->sent().size() == before, "admission performed I/O");
            hand.close();
            bool closed = false;
            try {
                (void)hand.set_angles(std::vector<int32_t>(6, 10));
            } catch (const DeviceNotReadyError &) {
                closed = true;
            }
            require(closed, "closed facade accepted a setter");
            std::cout << R"({"rejected":true,"frames":0})";
        } else if (scenario == "dispatch_without_ack") {
            const auto handle = hand.set_angles(std::vector<int32_t>(6, targets));
            complete(handle);
            require(handle.stage() == CommandStage::kDispatched, "no-ACK stage changed");
            std::cout << R"({"stage":"dispatched","error":null})";
        } else if (scenario == "ack_after_complete_response" || scenario == "execution_timeout") {
            const auto request_seen = std::make_shared<std::promise<void>>();
            transport->on_request(0x28, [request_seen, python, scenario](const Message &request) {
                if (request.data[0] == 0xC6 && request.dlc > 1) {
                    request_seen->set_value();
                    if (python && scenario == "execution_timeout") {
                        return std::vector<Message>{};
                    }
                    auto partial = request;
                    partial.dlc = 2;
                    return std::vector<Message>{partial};
                }
                return l6_reply(request);
            });
            const auto handle = hand.set_stall_thresholds(std::vector<double>(6, targets));
            require(request_seen->get_future().wait_for(2s) == std::future_status::ready, "ACK request not sent");
            if (scenario == "execution_timeout") {
                require(handle.wait_for(2s), "execution budget did not expire");
                require(handle.stage() == CommandStage::kFailed, "execution timeout did not fail");
                bool timed_out = false;
                try {
                    std::rethrow_exception(handle.error());
                } catch (const TimeoutError &) {
                    timed_out = true;
                }
                require(timed_out, "wrong execution error");
                std::cout << (python
                                  ? R"({"stage":"failed","exception":"timeout","budget":"response_window"})"
                                  : R"({"stage":"failed","exception":"timeout","budget":"admission_to_completion"})");
            } else if (python) {
                require(handle.wait_for(2s) && handle.stage() == CommandStage::kFailed,
                        "malformed response did not fail");
                bool malformed = false;
                try {
                    std::rethrow_exception(handle.error());
                } catch (const ProtocolError &) {
                    malformed = true;
                }
                require(malformed, "malformed response has the wrong error category");
                transport->on_request(0x28, l6_reply);
                complete(hand.set_stall_thresholds(std::vector<double>(6, targets)));
                std::cout
                    << R"({"stage":"acknowledged","partial_terminal":true,"partial_stage":"failed+protocol","error":null})";
            } else {
                require(!handle.wait_for(1ms), "partial response acknowledged command");
                Message full{};
                full.id = 0x28;
                full.dlc = 7;
                full.data[0] = 0xC6;
                // Returned settings deliberately differ from the requested values.
                std::fill(full.data.begin() + 1, full.data.begin() + 7, uint8_t{1});
                transport->inject(full);
                complete(handle);
                require(handle.stage() == CommandStage::kAcknowledged, "complete response not acknowledged");
                std::cout
                    << R"({"stage":"acknowledged","partial_terminal":false,"partial_stage":"pending","error":null})";
            }
        } else if (scenario == "latest_pending_replaced" || scenario == "claimed_request_not_replaced") {
            auto claimed = device.block();
            auto old = hand.set_angles(std::vector<int32_t>(6, targets));
            auto newest = hand.set_percent_angles(std::vector<float>(6, 50));
            require(old.stage() == CommandStage::kSuperseded && old.sent_frames() == 0, "latest slot not replaced");
            require(!claimed.done(), "claimed request was replaced");
            transport->release_tx_gate();
            complete(claimed);
            complete(newest);
            if (scenario == "latest_pending_replaced") {
                std::cout << R"({"stage":"superseded","sent_frames":0})";
            } else {
                std::cout << R"({"stage":"dispatched","superseded":false})";
            }
        } else if (scenario == "fifo_full") {
            const auto claimed = device.block();
            std::vector<CommandHandle> pending;
            for (size_t i = 0; i < capacity; ++i) {
                pending.push_back(hand.set_stall_thresholds(std::vector<double>(6, static_cast<double>(i + 1))));
            }
            const auto accepted = [&] {
                uint64_t count = 0;
                for (const auto &family : hand.get_health().command_families) {
                    count += family.accepted;
                }
                return count;
            };
            const auto before = accepted();
            bool full = false;
            try {
                (void)hand.set_stall_thresholds(std::vector<double>(6, 20));
            } catch (const QueueFullError &) {
                full = true;
            }
            require(full && accepted() == before, "FIFO overflow changed accepted count");
            transport->release_tx_gate();
            complete(claimed);
            for (auto &handle : pending) {
                complete(handle);
            }
            std::vector<int> values;
            for (const auto &frame : transport->sent()) {
                if (frame.data[0] == 0xC6 && frame.dlc > 1) {
                    values.push_back(frame.data[1]);
                }
            }
            require(values.size() == capacity, "rejected FIFO command was sent");
            for (size_t i = 0; i < capacity; ++i) {
                require(values[i] == int(i + 1), "FIFO order changed");
            }
            std::cout << R"({"exception":"queue_full","accepted_count_unchanged":true,"ordered":true})";
        } else if (scenario == "wait_timeout") {
            const auto handle = device.block();
            require(!handle.wait_for(std::chrono::milliseconds{timeout}), "caller wait unexpectedly completed");
            require(!handle.done() && !handle.error(), "wait timeout cancelled command");
            transport->release_tx_gate();
            complete(handle);
            std::cout << R"({"wait":"false","command_cancelled":false,"stage":"dispatched"})";
        } else if (scenario == "lifecycle_invalidation") {
            const auto claimed = device.block();
            const auto pending = hand.set_stall_thresholds(std::vector<double>(6, targets));
            auto closer = std::async(std::launch::async, [&] { hand.close(); });
            const auto ended = pending.wait_for(1s) && (python || claimed.wait_for(1s));
            transport->release_tx_gate();
            closer.get();
            if (python) {
                complete(claimed);
            }
            require(ended && pending.stage() == CommandStage::kFailed &&
                        claimed.stage() == (python ? CommandStage::kDispatched : CommandStage::kFailed),
                    "close has the wrong command terminal states");
            bool closed = false;
            try {
                std::rethrow_exception(pending.error());
            } catch (const ClosedError &) {
                closed = true;
            }
            require(closed, "wrong lifecycle error");
            hand.open();
            for (const auto &frame : device.transport->sent()) {
                require(!(frame.data[0] == 0xC6 && frame.dlc > 1), "old configuration replayed");
            }
            std::cout << (python ? R"({"pending":"failed+closed","claimed":"dispatched","replay":false})"
                                 : R"({"pending":"failed+closed","claimed":"failed+closed","replay":false})");
        } else if (scenario == "polling_fairness") {
            struct State {
                std::mutex mutex;
                std::condition_variable changed;
                bool polled = false;
            };
            const auto state = std::make_shared<State>();
            transport->on_request(0x28, [state](const Message &request) {
                if (request.data[0] == 1 && request.dlc == 1) {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    state->polled = true;
                    state->changed.notify_all();
                }
                return l6_reply(request);
            });
            const auto claimed = device.block();
            auto cached = std::async(std::launch::async, [&] {
                (void)hand.get_snapshot();
                (void)hand.get_health();
            });
            const bool available = cached.wait_for(500ms) == std::future_status::ready;
            hand.start_polling({{TelemetrySemantic::kAngles, 20ms}});
            for (int i = 0; i < targets; ++i) {
                (void)hand.set_angles(std::vector<int32_t>(6, i));
            }
            transport->release_tx_gate();
            cached.get();
            complete(claimed);
            std::unique_lock<std::mutex> lock(state->mutex);
            require(state->changed.wait_for(lock, 2s, [&] { return state->polled; }), "polling starved");
            lock.unlock();
            hand.stop_polling();
            require(available, "cache reads waited for transport");
            std::cout << R"({"cached_reads_blocked":false,"poll_progress":true})";
        } else {
            throw std::runtime_error("unknown vector scenario: " + scenario);
        }
        std::cout << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
