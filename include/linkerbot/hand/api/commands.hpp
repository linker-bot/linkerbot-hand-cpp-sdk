#pragma once

#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <vector>

#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/transport/session.hpp>

namespace linkerbot::hand {
    enum class CommandStage : uint8_t { kAccepted, kDispatched, kAcknowledged, kFailed, kSuperseded, kCancelled };

    enum class CommandFamily : uint8_t {
        kAngleTarget,
        kSpeedSetting,
        kAccelerationSetting,
        kCurrentLimit,
        kControlGain = 4,
        kTorqueLimit,
        kTorqueTarget,
        kMotionTime,
        kStallConfiguration = 8,
        kLimitCompensation,
        kDiscreteOperation = 10
    };

    // ACK support describes a protocol capability. ACK expectation describes
    // how this particular command is completed.
    enum class AckExpectation : uint8_t { kNone, kRequired };

    struct CommandTransition {
        CommandStage stage = CommandStage::kAccepted;
        std::chrono::steady_clock::time_point timestamp;
        uint64_t sent_frames = 0;
    };

    namespace command_detail {
        class CommandHandleAccess;
    }

    // Thread-safe observer for one accepted command. It represents protocol
    // delivery only; acknowledgement never implies mechanical arrival.
    // Numeric configuration ACKs require complete responses, not equality
    // with requested values or proof that the settings have taken effect.
    class CommandHandle {
    public:
        CommandHandle(uint64_t command_id, CommandFamily family, AckSupport ack_support,
                      std::chrono::steady_clock::time_point accepted_at = std::chrono::steady_clock::now());
        CommandHandle(uint64_t command_id, CommandFamily family, AckSupport ack_support, AckExpectation ack_expectation,
                      std::chrono::steady_clock::time_point accepted_at = std::chrono::steady_clock::now());

        [[nodiscard]] uint64_t command_id() const noexcept;
        [[nodiscard]] CommandFamily family() const noexcept;
        [[nodiscard]] CommandStage stage() const noexcept;
        [[nodiscard]] AckSupport ack_support() const noexcept;
        [[nodiscard]] AckExpectation ack_expectation() const noexcept;
        // Known successful physical writes, including a partial prefix while
        // still Accepted. Dispatched requires the entire command batch.
        [[nodiscard]] uint64_t sent_frames() const noexcept;
        [[nodiscard]] std::exception_ptr error() const noexcept;
        // Terminal physical batch receipts for unified asynchronous setters,
        // in observation order. Legacy synchronous operations do not collect
        // receipts. A composite
        // command can produce multiple receipts. Logical failure/cancellation
        // does not discard a later receipt from already-claimed backend I/O.
        // Empty means no terminal receipt has been observed, not zero delivery.
        [[nodiscard]] std::vector<transport::SendResult> send_results() const;
        [[nodiscard]] std::vector<CommandTransition> history() const;
        [[nodiscard]] bool done() const noexcept;

        // Returns false on this caller's wait timeout, without cancelling or
        // resubmitting the command. True means terminal, not necessarily success;
        // inspect stage()/error(). A non-positive timeout is rejected.
        [[nodiscard]] bool wait_for(std::chrono::milliseconds timeout) const;
        CommandStage wait() const;

    private:
        struct State;
        explicit CommandHandle(std::shared_ptr<State> state);

        std::shared_ptr<State> state_;

        friend class command_detail::CommandHandleAccess;
    };
} // namespace linkerbot::hand
