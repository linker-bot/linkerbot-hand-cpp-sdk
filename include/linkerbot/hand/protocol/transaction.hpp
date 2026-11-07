#pragma once

// Pure protocol contract: values and transaction-local parsing only. No
// transport/session headers, send/subscribe, mutexes, threads or waits.
#include <linkerbot/can/message.hpp>
#include <linkerbot/hand/errors.hpp>

#include <any>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace linkerbot::hand::protocol {
    enum class ProgressKind { unmatched, pending, complete, failed };
    enum class Origin { response, command_feedback, report, unknown };

    struct Progress {
        ProgressKind kind = ProgressKind::unmatched;
        std::any value;
        Origin origin = Origin::response;
        uint64_t duplicates = 0;
        uint64_t conflicting_duplicates = 0;
        std::exception_ptr error;
        bool terminal_error = false;
        uint64_t protocol_errors = 0;
        // Completed sub-results of a multipart response; never incomplete fragments.
        std::vector<std::pair<std::string, std::any>> updates;
        static Progress unmatched() { return {}; }
        static Progress pending() { return {ProgressKind::pending, {}, Origin::response}; }
        template <class T> static Progress complete(T value, Origin origin = Origin::response) {
            return {ProgressKind::complete, std::any(std::move(value)), origin};
        }
    };

    struct TransactionPlan {
        // Keys name conflicting on-wire response domains, not local request IDs.
        std::vector<std::string> keys;
        // Semantic exclusion without late-response poisoning of unrelated replies.
        std::vector<std::string> exclusion_groups;
        std::string domain;
        // Non-empty only for logical commands. Runtime uses this stable semantic
        // family to account queue admission and terminal outcomes; reads leave it
        // empty even when they share the same response domain.
        std::string command_family;
        std::vector<can::Message> frames;
        std::chrono::microseconds gap{0};
        std::chrono::microseconds initial_delay{0};
        // Mutable captures must be transaction-local assemblers. A protocol error
        // is thrown as its original typed exception and preserved by the runtime.
        std::function<Progress(const can::Message &)> consume;
        // Finite guard is only an explicit firmware timing assumption. When no
        // response bound is known, poison_on_timeout blocks reuse until recover.
        std::chrono::milliseconds late_guard{0};
        bool poison_on_timeout = true;
        // Complete batch snapshots survive a later incomplete batch.
        bool preserve_last_success = false;
        // Some batches must not publish before every outgoing frame is accepted.
        bool publish_after_send = false;
        // Optional command feedback: success still requires backend acceptance.
        // If no reply arrives within this window after the receipt, runtime ends
        // the request without a value and applies the configured late-response guard.
        std::optional<std::chrono::microseconds> optional_response_window;
        // Send-only controls may supersede optional feedback from an earlier control.
        // Ambiguous feedback keys remain poisoned for reads; subsequent writes still
        // transmit but cannot publish feedback until explicit recovery.
        bool supersede_optional_feedback = false;
        // Required response budget starts at the physical batch completion.
        // The submit deadline remains an independent total execution bound.
        std::optional<std::chrono::microseconds> response_window;
        // Keep a matching early error until the physical receipt is known.
        bool defer_errors_until_send = false;
    };

    class ProtocolBackend {
    public:
        virtual ~ProtocolBackend() = default;
        virtual TransactionPlan make_read(const std::string &operation, const std::any &argument = {}) const = 0;
        virtual TransactionPlan make_command(const std::string &operation, const std::any &argument = {}) const = 0;
        virtual Progress decode_report(const can::Message &) { return Progress::unmatched(); }
    };
} // namespace linkerbot::hand::protocol
