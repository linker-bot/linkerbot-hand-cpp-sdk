#pragma once

#include <linkerbot/transport/session.hpp>

namespace linkerbot::hand::unified_detail {
    inline bool python_response_after_claim(const transport::RoutedMessage &input, const transport::SendResult &send) {
        // Session event order is authoritative when both sides provide it.
        // Timestamp fallback matches the Python runner's legacy transport path.
        if (input.received_order != 0 && send.claimed_order) {
            return input.received_order > *send.claimed_order;
        }
        return send.claimed_at && input.received_at >= *send.claimed_at;
    }
}
