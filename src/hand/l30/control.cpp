#include "linkerbot/hand/l30/control_manager.hpp"
#include <linkerbot/hand/errors.hpp>
#include <stdexcept>

namespace linkerbot::hand::l30 {

    namespace {
        void send_control_write(Client &client, uint8_t subcmd, std::chrono::milliseconds timeout_ms) {
            if (timeout_ms.count() <= 0) {
                throw ValidationError("timeout_ms must be positive");
            }
            const auto response =
                client.request_ack(protocol::kParentControl, subcmd, protocol::empty_request_payload(),
                                   protocol::kEmptyRequestBytes, timeout_ms);
            protocol::check_response_status(response.data(), response.size());
        }
    } // namespace

    ControlManager::ControlManager(Client &client) : client_(&client) {}

    void ControlManager::enable(std::chrono::milliseconds timeout_ms) {
        const auto call = client_->enter();
        send_control_write(*client_, protocol::kSubEnable, timeout_ms);
    }

    void ControlManager::disable(std::chrono::milliseconds timeout_ms) {
        const auto call = client_->enter();
        send_control_write(*client_, protocol::kSubDisable, timeout_ms);
    }

    void ControlManager::calibrate(std::chrono::milliseconds timeout_ms) {
        auto operation = client_->operation(timeout_ms);
        send_control_write(*client_, protocol::kSubDisable, operation.remaining());
        const auto unlock = client_->request_ack(protocol::kParentConfig, protocol::kSubConfigUnlock,
                                                 {6, 0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}, 12, operation.remaining());
        protocol::check_response_status(unlock.data(), unlock.size());
        const auto response =
            client_->request_ack(protocol::kParentConfig, protocol::kSubCalibrate, protocol::empty_request_payload(),
                                 protocol::kEmptyRequestBytes, operation.remaining());
        protocol::check_response_status(response.data(), response.size());
    }
} // namespace linkerbot::hand::l30
