#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include <linkerbot/hand/capabilities.hpp>

namespace linkerbot::hand {
    enum class O30Layout : uint8_t { kMountedV36, kPythonV25 };

    // Opt-in runtime semantics for migration from the Python SDK. Native
    // constructors and model Managers retain their existing behavior.
    enum class RuntimeBehavior : uint8_t { kNative, kPython };

    enum class L6Protocol : uint8_t { kClassic, kHopV004 };
    // Runtime-only policies that may be supplied independently of a physical
    // endpoint. Discovery owns model, side, interface and address selection;
    // callers own these execution policies.
    struct HandRuntimeConfig {
        // L30 can optionally enable device-side reports while opening. The
        // unified facade leaves them disabled by default and uses host polling.
        bool auto_start_periodic_reports = false;

        std::chrono::milliseconds polling_interval{100};
        std::optional<std::filesystem::path> angle_mapping_path;

        std::chrono::milliseconds command_timeout{100};
        std::chrono::milliseconds identity_timeout{1000};
        std::chrono::milliseconds transaction_timeout{100};
        std::chrono::milliseconds shutdown_timeout{2000};

        // Maximum setter batch start rate; FIFO capacity counts pending items
        // only. Four latest slots and one executing setter are additional.
        double command_rate_hz = 250.0;
        size_t discrete_queue_capacity = 16;

        uint32_t recovery_attempts = 1;
        std::chrono::milliseconds recovery_backoff{100};
        O30Layout o30_layout = O30Layout::kMountedV36;
        bool empty_polling_plan_disables = false;
        RuntimeBehavior runtime_behavior = RuntimeBehavior::kNative;
        size_t read_waiter_capacity = 256;
        // Python profile only: nullopt uses the protocol's response window.
        std::optional<std::chrono::milliseconds> response_timeout;
        std::chrono::milliseconds recovery_timeout{5000};
        std::chrono::milliseconds recovery_quiet{200};
        // Python policy only: nullopt selects 256 routed receive frames per device.
        // Native constructors retain their existing 512-frame default.
        std::optional<size_t> route_queue_capacity;
    };

    // Fully resolved input consumed by HandDevice. Direct construction fills
    // both endpoint and runtime fields; discovery combines its confirmed
    // endpoint with a caller-provided HandRuntimeConfig.
    struct HandDeviceConfig {
        Model model = Model::kUnknown;
        HandSide side = HandSide::kUnknown;
        std::string interface_name = "can0";

        // Classic CAN models derive their fixed arbitration ID from side. If
        // supplied there, address must match that derived ID. For L30 this is
        // the node ID; for O30 it is the HOP request ID.
        std::optional<uint32_t> address;

        // L30 host ID. It is rejected for every other model.
        std::optional<uint32_t> host_address;

        // L30 can optionally enable device-side reports while opening. The
        // unified facade leaves them disabled by default and uses host polling.
        bool auto_start_periodic_reports = false;

        std::chrono::milliseconds polling_interval{100};
        std::optional<std::filesystem::path> angle_mapping_path;

        // Native policy: total setter budget from admission, including queueing,
        // sending and any required ACK. The Python policy uses response_timeout
        // or protocol windows instead. Handle wait timeouts remain independent.
        std::chrono::milliseconds command_timeout{100};

        // Bounds identity verification performed as part of open() and each
        // recover() attempt.
        std::chrono::milliseconds identity_timeout{1000};

        // Bounds each active baseline telemetry read performed before the
        // device becomes ready after open() or recover().
        std::chrono::milliseconds transaction_timeout{100};

        // Total caller wait budget for close()/recover(), including serialized
        // lifecycle ownership and recovery initialization. On expiry the facade
        // stays non-Ready; safe cleanup retains resources until blocked drivers
        // or callbacks return. Process teardown still requires bounded drivers.
        std::chrono::milliseconds shutdown_timeout{2000};

        // recover() makes at most this many complete backend recovery,
        // identity and baseline attempts. Identity mismatches are terminal.
        // Negative backoff values are rejected; retries are never unbounded.
        // Maximum setter batch start rate; FIFO capacity counts pending items
        // only. Four latest slots and one executing setter are additional.
        double command_rate_hz = 250.0;
        size_t discrete_queue_capacity = 16;

        uint32_t recovery_attempts = 1;
        std::chrono::milliseconds recovery_backoff{100};
        // Explicit opt-in for the latest L6 firmware; legacy defaults remain Classic.
        L6Protocol l6_protocol = L6Protocol::kClassic;
        // Explicit protocol layout, not firmware inference. V25 matches the
        // Python whole-hand path; native Managers retain mounted V36 behavior.
        O30Layout o30_layout = O30Layout::kMountedV36;
        bool empty_polling_plan_disables = false;
        RuntimeBehavior runtime_behavior = RuntimeBehavior::kNative;
        size_t read_waiter_capacity = 256;
        // Python profile only: nullopt uses the protocol's response window.
        std::optional<std::chrono::milliseconds> response_timeout;
        std::chrono::milliseconds recovery_timeout{5000};
        std::chrono::milliseconds recovery_quiet{200};
        // Python policy only: nullopt selects 256 routed receive frames per device.
        // Native constructors retain their existing 512-frame default.
        std::optional<size_t> route_queue_capacity;
    };
} // namespace linkerbot::hand
