#pragma once
#include <linkerbot/hand/l30/protocol_backend.hpp>

namespace linkerbot::hand::unified_detail {
    inline bool python_l30_tactile(const protocol::TransactionPlan &plan) {
        return !plan.frames.empty() &&
               l30::protocol::parse_can_id(plan.frames.front().id).parent == l30::protocol::kParentTactile;
    }
    inline protocol::TransactionPlan python_l30_response(protocol::TransactionPlan plan) {
        if (!plan.consume || plan.frames.empty()) {
            return plan;
        }
        const auto request = l30::protocol::parse_can_id(plan.frames.front().id);
        const auto response_id = l30::protocol::expected_response_id(plan.frames.front().id);
        auto decode = std::move(plan.consume);
        plan.consume = [request, response_id, decode = std::move(decode)](const can::Message &message) mutable {
            if (!l30::protocol::valid_frame(message, response_id)) {
                return protocol::Progress::unmatched();
            }
            if (message.dlc < 3) {
                throw ProtocolError("L30 response is too short for its status header");
            }
            size_t expected = message.data[0];
            if (request.parent == l30::protocol::kParentTactile) {
                const auto transaction = message.data[1];
                if (transaction != l30::protocol::kTactileFirstTransaction &&
                    transaction != l30::protocol::kTactileSecondTransaction) {
                    throw ProtocolError("L30 tactile transaction control is invalid");
                }
                expected = transaction == l30::protocol::kTactileFirstTransaction ? l30::protocol::kTactileFirstLength
                                                                                  : l30::protocol::kTactileSecondLength;
                if (message.data[0] != expected) {
                    throw ProtocolError("L30 tactile response declares an invalid length");
                }
            } else {
                if (message.data[1] != l30::protocol::kSingleFrameTransaction) {
                    throw ProtocolError("L30 single-frame response transaction control is invalid");
                }
                if (request.access == l30::protocol::kAccessWrite) {
                    expected = 0;
                } else if (request.parent == l30::protocol::kParentQuery) {
                    expected =
                        request.subcmd == l30::protocol::kSubTemperature || request.subcmd == l30::protocol::kSubFault
                            ? l30::protocol::kU8VectorPayloadBytes
                            : l30::protocol::kVectorPayloadBytes;
                } else if (request.parent == l30::protocol::kParentConfig &&
                           request.subcmd == l30::protocol::kSubDeviceInfo) {
                    expected = l30::protocol::kDeviceInfoBytes;
                }
            }
            if (message.data[2] != l30::protocol::kStatusOk) {
                throw DeviceFaultError("L30 response contains a device fault status");
            }
            if (message.data[0] != expected || expected + 3 > message.dlc) {
                throw ProtocolError("L30 response length does not match its object");
            }
            return decode(message);
        };
        return plan;
    }
}
