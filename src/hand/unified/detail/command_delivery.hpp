#pragma once
#include <linkerbot/transport/session.hpp>
#include <mutex>
#include <optional>

namespace linkerbot::hand::unified_detail {
    struct CommandDelivery {
        std::mutex mutex;
        std::optional<transport::SendReceipt> receipt;
        bool final_send = false;
        bool closing_claimed = false;
        void accepted(const transport::SendReceipt &next, bool final) {
            const std::lock_guard<std::mutex> lock(mutex);
            receipt = next;
            final_send = final;
        }
        bool preserve_claimed() {
            const std::lock_guard<std::mutex> lock(mutex);
            if (receipt && final_send && receipt->result().claimed_at) {
                closing_claimed = true;
            }
            return closing_claimed;
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
