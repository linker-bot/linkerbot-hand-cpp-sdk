#include "fake_classic_hand.hpp"
#include <iostream>
#include <linkerbot/can/frame.hpp>
#include <linkerbot/hand/l30/l30.hpp>
#include <linkerbot/hand/o30/v25.hpp>
#include <map>

namespace {
    using namespace linkerbot::hand;
    using linkerbot::can::FakeTransport;
    using linkerbot::can::Message;
    std::vector<Message> capture(const std::string &name) {
        if (name == "l30_calibrate") {
            auto transport = std::make_unique<FakeTransport>();
            auto *raw = transport.get();
            for (const auto fields : std::vector<std::pair<uint8_t, uint8_t>>{{1, 8}, {3, 1}, {3, 5}}) {
                const auto id = l30::protocol::FrameId{0, 1, fields.first, fields.second, 1, 0}.to_int();
                raw->on_request(id, [](const Message &request) {
                    Message reply{};
                    reply.id = l30::protocol::expected_response_id(request.id);
                    reply.is_fd = true;
                    reply.is_extended = true;
                    reply.dlc = 3;
                    return std::vector<Message>{reply};
                });
            }
            l30::L30 hand(std::move(transport), 1, 0, false);
            raw->clear_sent();
            hand.control.calibrate(std::chrono::milliseconds{300});
            return raw->sent();
        }
        if (name.rfind("o30_", 0) == 0) {
            const std::map<std::string, TelemetrySemantic> reads{
                {"o30_read_angles", TelemetrySemantic::kAngles},
                {"o30_read_measured_speeds", TelemetrySemantic::kMeasuredSpeeds},
                {"o30_read_configured_speeds", TelemetrySemantic::kConfiguredSpeeds},
                {"o30_read_configured_accelerations", TelemetrySemantic::kConfiguredAccelerations},
                {"o30_read_currents", TelemetrySemantic::kCurrents},
                {"o30_read_current_limits", TelemetrySemantic::kCurrentLimits},
                {"o30_read_torques", TelemetrySemantic::kTorques},
                {"o30_read_torque_limits", TelemetrySemantic::kTorqueLimits},
                {"o30_read_motion_times", TelemetrySemantic::kMotionTimes},
                {"o30_read_temperatures", TelemetrySemantic::kTemperatures},
                {"o30_read_voltages", TelemetrySemantic::kVoltages},
                {"o30_read_faults", TelemetrySemantic::kFaults},
                {"o30_read_stall_detection_times", TelemetrySemantic::kStallDetectionTimes},
                {"o30_read_stall_thresholds", TelemetrySemantic::kStallThresholds},
                {"o30_read_stall_hold_currents", TelemetrySemantic::kStallHoldCurrents}};
            if (const auto found = reads.find(name); found != reads.end()) {
                return o30::v25::transaction(1, o30::v25::object(found->second)).frames;
            }
            const std::map<std::string, CommandSemantic> writes{
                {"o30_angles", CommandSemantic::kAngles},
                {"o30_speeds", CommandSemantic::kSpeeds},
                {"o30_accelerations", CommandSemantic::kAccelerations},
                {"o30_current_limits", CommandSemantic::kCurrentLimits},
                {"o30_torque_limits", CommandSemantic::kTorqueLimits},
                {"o30_motion_times", CommandSemantic::kMotionTimes},
                {"o30_stall_detection_times", CommandSemantic::kStallDetectionTimes},
                {"o30_stall_thresholds", CommandSemantic::kStallThresholds},
                {"o30_stall_hold_currents", CommandSemantic::kStallHoldCurrents}};
            const auto found = writes.find(name);
            if (found == writes.end()) {
                throw std::runtime_error("unknown O30 scenario");
            }
            std::vector<double> values;
            for (int i = 1; i <= 20; ++i) {
                values.push_back(i);
            }
            return o30::v25::transaction(1, o30::v25::object(found->second), o30::v25::encode(values)).frames;
        }
        validation::FakeHand device;
        device.transport->on_request(0x28, [](const Message &request) {
            if (request.data[0] == 0xCF) {
                auto reply = request;
                reply.dlc = 2;
                reply.data[1] = 1;
                return std::vector<Message>{reply};
            }
            return validation::l6_reply(request);
        });
        device.transport->clear_sent();
        const std::vector<double> values{1, 2, 3, 4, 5, 6};
        auto handle = [&] {
            if (name == "l6_position_kp") {
                return device.hand.set_position_kp(values);
            }
            if (name == "l6_position_kd") {
                return device.hand.set_position_kd(values);
            }
            if (name == "l6_torque_kd") {
                return device.hand.set_torque_kd(values);
            }
            if (name == "l6_torques") {
                return device.hand.set_torques(values);
            }
            if (name == "l6_limit_compensations") {
                return device.hand.set_limit_compensations(values);
            }
            if (name == "l6_save_confirmed") {
                return device.hand.save_parameters_confirmed();
            }
            throw std::runtime_error("unknown migration scenario");
        }();
        validation::require(handle.wait_for(std::chrono::seconds{2}), "command wait expired");
        if (handle.error()) {
            std::rethrow_exception(handle.error());
        }
        validation::require(handle.stage() == CommandStage::kAcknowledged, "command was not acknowledged");
        return device.transport->sent();
    }
}
int main(int argc, char **argv) {
    try {
        if (argc != 2) {
            throw std::runtime_error("expected scenario");
        }
        const auto frames = capture(argv[1]);
        std::cout << "[";
        for (size_t i = 0; i < frames.size(); ++i) {
            if (i) {
                std::cout << ",";
            }
            const auto &f = frames[i];
            const auto flags = linkerbot::can::resolve_tx_flags(linkerbot::can::LinkKind::kCanFd, f);
            std::cout << "{\"id\":" << f.id << ",\"fd\":" << (flags.is_fd ? "true" : "false")
                      << ",\"extended\":" << (f.is_extended ? "true" : "false")
                      << ",\"brs\":" << (flags.is_brs ? "true" : "false") << ",\"data\":[";
            for (size_t j = 0; j < f.dlc; ++j) {
                if (j) {
                    std::cout << ",";
                }
                std::cout << unsigned(f.data[j]);
            }
            std::cout << "]}";
        }
        std::cout << "]\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
