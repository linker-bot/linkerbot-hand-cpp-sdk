#pragma once
#include <linkerbot/hand/api/config.hpp>
#include <linkerbot/hand/protocol/transaction.hpp>

namespace linkerbot::hand::unified_detail {
    inline protocol::TransactionPlan python_tactile(protocol::TransactionPlan plan, Model model) {
        if (plan.frames.empty() || plan.frames.front().is_fd || plan.frames.front().is_extended ||
            model == Model::kL30 || model == Model::kO30) {
            return plan;
        }
        const auto id = plan.frames.front().id;
        const auto first = plan.frames.front().data[0];
        if (first < 0xB1 || first > 0xB5) {
            return plan;
        }
        auto decode = std::move(plan.consume);
        plan.consume = [decode = std::move(decode), id, first, model,
                        all = plan.frames.size() > 1](const can::Message &message) mutable {
            if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc == 0 ||
                message.data[0] < 0xB1 || message.data[0] > 0xB5 || (!all && message.data[0] != first)) {
                return protocol::Progress::unmatched();
            }
            const auto length = model == Model::kO6 ? 6 : 8;
            if (message.dlc != length) {
                throw ProtocolError("matching tactile row has an invalid length");
            }
            const auto rows = model == Model::kO6 ? 10 : 12;
            const auto marker = message.data[1];
            // Python accepts compact row indices and nibble row markers.
            const auto row = marker < rows ? marker : (marker & 15) == 0 ? marker >> 4 : 255;
            if (row >= rows) {
                throw ProtocolError("matching tactile row has an invalid index");
            }
            auto normalized = message;
            if (model != Model::kL6) {
                normalized.data[1] = static_cast<uint8_t>(row << 4);
            }
            auto progress = decode(normalized);
            if (progress.kind == protocol::ProgressKind::failed) {
                // A malformed matching sample is terminal for this transaction,
                // without poisoning unrelated channels or successful cache data.
                progress.terminal_error = false;
            }
            return progress;
        };
        return plan;
    }
}
