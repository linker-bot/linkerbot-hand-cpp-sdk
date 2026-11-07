#pragma once
#include <linkerbot/hand/protocol/transaction.hpp>

namespace linkerbot::hand::unified_detail {
    struct ClassicVectorResponse {
        uint32_t id;
        uint8_t opcode;
        size_t count;
    };
    inline protocol::TransactionPlan strict_classic_vector(protocol::TransactionPlan plan,
                                                           ClassicVectorResponse response) {
        if (!plan.consume || response.count > 7) {
            throw ValidationError("invalid classic response plan");
        }
        auto decode = std::move(plan.consume);
        plan.consume = [response, decode = std::move(decode)](const can::Message &message) mutable {
            if (message.id == response.id && !message.is_fd && !message.is_extended && !message.is_brs &&
                message.dlc > 0 && message.data[0] == response.opcode && message.dlc != response.count + 1) {
                throw ProtocolError("classic configuration response has an invalid length");
            }
            return decode(message);
        };
        plan.late_guard = std::chrono::milliseconds{5};
        plan.poison_on_timeout = false;
        return plan;
    }
}
