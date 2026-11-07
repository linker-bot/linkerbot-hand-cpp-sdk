#pragma once

#include <chrono>

#include <linkerbot/hand/l30/client.hpp>

namespace linkerbot::hand::l30 {

    // Global motor enable/disable. Both are acknowledged writes; neither touches
    // any sensor snapshot.
    class ControlManager {
        friend class L30;

    public:
        explicit ControlManager(Client &client);

        ControlManager(const ControlManager &) = delete;
        ControlManager &operator=(const ControlManager &) = delete;
        ControlManager(ControlManager &&) = delete;
        ControlManager &operator=(ControlManager &&) = delete;

        // Throws ProtocolError if the device ACK reports a non-zero status.
        void enable(std::chrono::milliseconds timeout_ms = std::chrono::milliseconds{100});

        void disable(std::chrono::milliseconds timeout_ms = std::chrono::milliseconds{100});

        // Disable -> unlock -> calibrate. Failure never advances the chain.
        // One total timeout bounds all three ACK-gated steps.
        void calibrate(std::chrono::milliseconds timeout_ms = std::chrono::milliseconds{100});

    private:
        Client *client_;
    };

} // namespace linkerbot::hand::l30
