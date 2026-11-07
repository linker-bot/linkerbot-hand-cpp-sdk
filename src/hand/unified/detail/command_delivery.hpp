#pragma once
#include <functional>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/transport/session.hpp>
#include <mutex>
#include <optional>

namespace linkerbot::hand::unified_detail {
    struct CommandDelivery {
        std::mutex mutex;
        std::optional<transport::SendReceipt> receipt;
        bool final_send = false;
        bool closing_claimed = false;
        bool closing = false;
        bool claimed_batch = false;
        transport::SendReceipt admit(const std::function<transport::SendReceipt()> &submit, bool final) {
            const std::lock_guard<std::mutex> lock(mutex);
            if (closing) {
                throw ClosedError("command admission ended");
            }
            auto next = submit();
            receipt = next;
            final_send = final;
            return next;
        }
        void accepted(const transport::SendReceipt &next, bool final) {
            const std::lock_guard<std::mutex> lock(mutex);
            receipt = next;
            final_send = final;
        }
        bool preserve_claimed(bool allow_dispatched = true) {
            const std::lock_guard<std::mutex> lock(mutex);
            closing = true;
            claimed_batch = receipt && receipt->result().claimed_at.has_value();
            if (allow_dispatched && final_send && claimed_batch) {
                closing_claimed = true;
            }
            return closing_claimed;
        }
        bool draining_claimed() {
            const std::lock_guard<std::mutex> lock(mutex);
            return claimed_batch;
        }
        bool preserved() {
            const std::lock_guard<std::mutex> lock(mutex);
            return closing_claimed;
        }
        std::optional<transport::SendReceipt> retained() {
            const std::lock_guard<std::mutex> lock(mutex);
            return closing_claimed ? receipt : std::nullopt;
        }
    };
}
