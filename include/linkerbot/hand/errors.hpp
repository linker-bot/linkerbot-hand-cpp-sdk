#pragma once

#include <chrono>
#include <cstdint>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace linkerbot::hand {
    // Increment when an existing error code or its machine-readable meaning
    // changes incompatibly. Adding a new append-only code keeps this version.
    inline constexpr uint16_t kErrorSchemaVersion = 1;

    // Stable, machine-readable categories for errors crossing the hand SDK API.
    enum class ErrorCode : uint16_t {
        kValidation = 1,
        kUnsupportedOperation = 2,
        kUnsupportedFirmware = 3,
        kIdentityMismatch = 4,
        kDeviceNotReady = 5,
        kTransport = 6,
        kProtocol = 7,
        kTimeout = 8,
        kDeviceFault = 9,
        kQueueFull = 10,
        kPollingPlan = 11,
        kClosed = 12,
        kLifecycle = 13,
        kInternal = 14,
        kDiscovery = 15,
        kNoHandFound = 16,
        kAmbiguousHand = 17,
        kIncompleteIdentity = 18,
    };

    [[nodiscard]] constexpr std::string_view error_code_name(ErrorCode code) noexcept {
        switch (code) {
        case ErrorCode::kValidation:
            return "validation";
        case ErrorCode::kUnsupportedOperation:
            return "unsupported_operation";
        case ErrorCode::kUnsupportedFirmware:
            return "unsupported_firmware";
        case ErrorCode::kIdentityMismatch:
            return "identity_mismatch";
        case ErrorCode::kDeviceNotReady:
            return "device_not_ready";
        case ErrorCode::kTransport:
            return "transport";
        case ErrorCode::kProtocol:
            return "protocol";
        case ErrorCode::kTimeout:
            return "timeout";
        case ErrorCode::kDeviceFault:
            return "device_fault";
        case ErrorCode::kQueueFull:
            return "queue_full";
        case ErrorCode::kPollingPlan:
            return "polling_plan";
        case ErrorCode::kClosed:
            return "closed";
        case ErrorCode::kLifecycle:
            return "lifecycle";
        case ErrorCode::kInternal:
            return "internal";
        case ErrorCode::kDiscovery:
            return "discovery";
        case ErrorCode::kNoHandFound:
            return "no_hand_found";
        case ErrorCode::kAmbiguousHand:
            return "ambiguous_hand";
        case ErrorCode::kIncompleteIdentity:
            return "incomplete_identity";
        }
        return "unknown";
    }

    struct ErrorContext {
        std::string operation;
        std::string model;
        std::string channel;
        uint64_t generation = 0;
        std::optional<uint32_t> arbitration_id;
        std::optional<int> native_status;
    };

    class HandError : public std::runtime_error {
    public:
        HandError(ErrorCode code, std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : std::runtime_error(std::move(message)), code_(code), context_(std::move(context)),
              cause_(std::move(cause)) {}

        [[nodiscard]] ErrorCode code() const noexcept { return code_; }

        [[nodiscard]] const ErrorContext &context() const noexcept { return context_; }

        [[nodiscard]] std::exception_ptr cause() const noexcept { return cause_; }

    private:
        void enrich_context(const ErrorContext &context) {
            if (context_.operation.empty()) {
                context_.operation = context.operation;
            }
            if (context_.model.empty()) {
                context_.model = context.model;
            }
            if (context_.channel.empty()) {
                context_.channel = context.channel;
            }
            if (context_.generation == 0) {
                context_.generation = context.generation;
            }
            if (!context_.arbitration_id) {
                context_.arbitration_id = context.arbitration_id;
            }
            if (!context_.native_status) {
                context_.native_status = context.native_status;
            }
        }

        friend std::exception_ptr classify_exception(std::exception_ptr cause, ErrorContext context,
                                                     ErrorCode fallback);

        ErrorCode code_;
        ErrorContext context_;
        std::exception_ptr cause_;
    };

    class ValidationError : public HandError {
    public:
        explicit ValidationError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kValidation, std::move(message), std::move(context), std::move(cause)) {}
    };

    class UnsupportedOperationError : public HandError {
    public:
        explicit UnsupportedOperationError(std::string message, ErrorContext context = {},
                                           std::exception_ptr cause = {})
            : HandError(ErrorCode::kUnsupportedOperation, std::move(message), std::move(context), std::move(cause)) {}
    };

    class UnsupportedFirmwareError : public HandError {
    public:
        explicit UnsupportedFirmwareError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kUnsupportedFirmware, std::move(message), std::move(context), std::move(cause)) {}
    };

    class IdentityMismatchError : public HandError {
    public:
        explicit IdentityMismatchError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kIdentityMismatch, std::move(message), std::move(context), std::move(cause)) {}
    };

    class DeviceNotReadyError : public HandError {
    public:
        explicit DeviceNotReadyError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kDeviceNotReady, std::move(message), std::move(context), std::move(cause)) {}
    };

    class TransportError : public HandError {
    public:
        explicit TransportError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kTransport, std::move(message), std::move(context), std::move(cause)) {}
    };

    class ProtocolError : public HandError {
    public:
        explicit ProtocolError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kProtocol, std::move(message), std::move(context), std::move(cause)) {}
    };

    class TimeoutError : public HandError {
    public:
        explicit TimeoutError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kTimeout, std::move(message), std::move(context), std::move(cause)) {}
    };

    // A sent transaction exhausted its response window. Recovery may retry
    // this category without retrying protocol errors or transport failures.
    class ResponseTimeoutError : public TimeoutError {
    public:
        using TimeoutError::TimeoutError;
        ResponseTimeoutError(std::string message, ErrorContext context, std::chrono::microseconds window,
                             std::chrono::milliseconds guard)
            : TimeoutError(std::move(message), std::move(context)), response_window_(window), late_guard_(guard) {}
        [[nodiscard]] std::chrono::microseconds response_window() const noexcept { return response_window_; }
        [[nodiscard]] std::chrono::milliseconds late_guard() const noexcept { return late_guard_; }

    private:
        std::chrono::microseconds response_window_{0};
        std::chrono::milliseconds late_guard_{0};
    };

    class DeviceFaultError : public HandError {
    public:
        explicit DeviceFaultError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kDeviceFault, std::move(message), std::move(context), std::move(cause)) {}
    };

    class QueueFullError : public HandError {
    public:
        explicit QueueFullError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kQueueFull, std::move(message), std::move(context), std::move(cause)) {}
    };

    class PollingPlanError : public HandError {
    public:
        explicit PollingPlanError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kPollingPlan, std::move(message), std::move(context), std::move(cause)) {}
    };

    class ClosedError : public HandError {
    public:
        explicit ClosedError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kClosed, std::move(message), std::move(context), std::move(cause)) {}
    };

    class LifecycleError : public HandError {
    public:
        explicit LifecycleError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kLifecycle, std::move(message), std::move(context), std::move(cause)) {}
    };

    class InternalError : public HandError {
    public:
        explicit InternalError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : HandError(ErrorCode::kInternal, std::move(message), std::move(context), std::move(cause)) {}
    };

    class DiscoveryError : public HandError {
    public:
        explicit DiscoveryError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : DiscoveryError(ErrorCode::kDiscovery, std::move(message), std::move(context), std::move(cause)) {}

    protected:
        DiscoveryError(ErrorCode code, std::string message, ErrorContext context, std::exception_ptr cause)
            : HandError(code, std::move(message), std::move(context), std::move(cause)) {}
    };

    class NoHandFoundError : public DiscoveryError {
    public:
        explicit NoHandFoundError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : DiscoveryError(ErrorCode::kNoHandFound, std::move(message), std::move(context), std::move(cause)) {}
    };

    class AmbiguousHandError : public DiscoveryError {
    public:
        explicit AmbiguousHandError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : DiscoveryError(ErrorCode::kAmbiguousHand, std::move(message), std::move(context), std::move(cause)) {}
    };

    class IncompleteIdentityError : public DiscoveryError {
    public:
        explicit IncompleteIdentityError(std::string message, ErrorContext context = {}, std::exception_ptr cause = {})
            : DiscoveryError(ErrorCode::kIncompleteIdentity, std::move(message), std::move(context), std::move(cause)) {
        }
    };

    // Compatibility names used by the existing identity parser.
    class IdentityProtocolError : public ProtocolError {
    public:
        using ProtocolError::ProtocolError;
    };

    // Normalize exceptions crossing from transport, protocol workers, or
    // legacy model implementations into the public hand error hierarchy.
    [[nodiscard]] std::exception_ptr classify_exception(std::exception_ptr cause, ErrorContext context = {},
                                                        ErrorCode fallback = ErrorCode::kInternal);

    [[noreturn]] void rethrow_as_hand_error(std::exception_ptr cause, ErrorContext context = {},
                                            ErrorCode fallback = ErrorCode::kInternal);
}
