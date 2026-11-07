#include "../detail/checked_optional.hpp"
#include "detail/classic_response.hpp"
#include "detail/model_adapters.hpp"

#include <algorithm>
#include <any>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/l20lite/protocol_backend.hpp>
#include <linkerbot/hand/l6/protocol_backend.hpp>
#include <linkerbot/hand/o6/protocol_backend.hpp>

namespace linkerbot::hand::unified_detail {
    namespace {
        uint32_t required_request_id(const DeviceCapabilities &capabilities) {
            const auto &id = capabilities.transport.request_id;
            if (!id.has_value()) {
                throw ValidationError("command transport requires a request ID");
            }
            return *id;
        }

        template <typename Value, size_t Size>
        [[nodiscard]] std::array<Value, Size> fixed_values(const CommandRequest &request) {
            if (request.raw_values.size() != Size) {
                throw ValidationError("internal command vector size mismatch");
            }
            std::array<Value, Size> result{};
            for (size_t index = 0; index < Size; ++index) {
                const auto value = request.raw_values[index];
                if (!std::isfinite(value) || std::trunc(value) != value) {
                    throw ValidationError("internal command value must be a finite integer");
                }
                result[index] = static_cast<Value>(value);
            }
            return result;
        }

        template <size_t Size, typename Convert>
        [[nodiscard]] std::array<float, Size> converted_values(const CommandRequest &request, Convert convert) {
            if (request.raw_values.size() != Size) {
                throw ValidationError("internal command vector size mismatch");
            }
            std::array<float, Size> result{};
            for (size_t index = 0; index < Size; ++index) {
                result[index] = static_cast<float>(convert(request.raw_values[index]));
            }
            return result;
        }

        template <size_t Size> [[nodiscard]] std::array<float, Size> percent_from_u8(const CommandRequest &request) {
            return converted_values<Size>(request, [](double raw) { return raw * 100.0 / 255.0; });
        }

        template <size_t Size>
        [[nodiscard]] std::array<float, Size> milliseconds_from_u8(const CommandRequest &request) {
            return converted_values<Size>(request, [](double raw) { return raw * 10.0; });
        }

        // Correlate the write response without issuing an extra read. A
        // complete configuration response need not equal the requested values.
        protocol::TransactionPlan with_response(protocol::TransactionPlan write, protocol::TransactionPlan read) {
            write.domain = read.domain;
            if (write.keys.empty()) {
                write.keys = std::move(read.keys);
            }
            write.consume = std::move(read.consume);
            return write;
        }

        template <typename Device>
        CommandOutcome acknowledged_write(Device &device, protocol::TransactionPlan plan,
                                          const CommandRequest &request) {
            if (!plan.consume) {
                throw LifecycleError("acknowledged write requires a response decoder");
            }
            const auto sent_frames = plan.frames.size();
            if (plan.command_family.empty()) {
                plan.command_family = plan.domain;
            }
            plan.optional_response_window.reset();
            plan.publish_after_send = true;
            plan.preserve_last_success = true;
            auto decode = std::move(plan.consume);
            plan.consume = [decode = std::move(decode)](const can::Message &message) mutable {
                auto progress = decode(message);
                progress.origin = protocol::Origin::command_feedback;
                return progress;
            };
            (void)device.runtime()->execute(std::move(plan), runtime::deadline_after(request.timeout));
            return {sent_frames, true};
        }

        CommandOutcome l6_acknowledged_write(l6::L6 &device, uint8_t opcode, bool stall,
                                             const CommandRequest &request) {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, l6::kJointCount>(request);
            auto read = stall ? l6::protocol::read_stall(id, opcode) : l6::protocol::read_telemetry(id, opcode);
            return acknowledged_write(
                device, with_response(l6::protocol::command(id, opcode, {raw.begin(), raw.end()}), std::move(read)),
                request);
        }

        CommandOutcome l6_raw_write(l6::L6 &device, const CommandRequest &request, uint8_t opcode, const char *domain) {
            const auto id = required_request_id(device.capabilities());
            auto plan = l6::protocol::raw_control_transaction(id, opcode, domain,
                                                              fixed_values<uint8_t, l6::kJointCount>(request));
            return acknowledged_write(device, strict_classic_vector(std::move(plan), {id, opcode, l6::kJointCount}),
                                      request);
        }

        template <typename Device> CommandOutcome confirmed_save(Device &device, std::chrono::milliseconds timeout) {
            const auto id = required_request_id(device.capabilities());
            protocol::TransactionPlan plan;
            can::Message frame{};
            frame.format = can::FrameFormat::kClassic;
            frame.id = id;
            frame.dlc = 8;
            std::fill_n(frame.data.begin(), 8, uint8_t{0xCF});
            plan.frames = {frame};
            plan.domain = "config.save_confirmed";
            plan.keys = {"config.save"};
            plan.command_family = "discrete_operations";
            plan.late_guard = std::chrono::milliseconds{20};
            plan.poison_on_timeout = false;
            plan.consume = [id](const can::Message &message) {
                if (message.id != id || message.is_fd || message.is_extended || message.dlc == 0 ||
                    message.data[0] != 0xCF) {
                    return protocol::Progress::unmatched();
                }
                if (message.dlc != 2 || message.data[1] != 1) {
                    throw ProtocolError("save-parameters response was not CF 01");
                }
                return protocol::Progress::complete(true);
            };
            (void)device.runtime()->execute(std::move(plan), runtime::deadline_after(timeout));
            return {1, true};
        }

        void require_timeout(const CommandRequest &request) {
            if (request.timeout.count() <= 0) {
                throw ValidationError("command timeout must be positive");
            }
        }

        // Mounted O30 writes ACK three independent ranges. Publish a typed
        // cache only after all ranges succeeded, using their returned bytes.
        // A legal short write ACK carries insufficient telemetry: retain the
        // preceding cache instead of filling absent values from the request.
        template <typename MakeData>
        void publish_mounted_ack(o30::O30 &device, const CommandRequest &request, uint8_t mi, const char *domain,
                                 MakeData make_data) {
            if (!request.completion) {
                return;
            }
            const auto accepted = request.completion->history().front().timestamp;
            auto core = device.runtime();
            std::array<uint8_t, o30::kSlotCount> slots{};
            std::optional<runtime::Sample> provenance;
            for (const auto &segment : o30::kSafeWriteSegments) {
                const auto key = "hop/" + std::to_string(mi) + "/" + std::to_string(segment.si);
                auto sample = core->snapshot(key + "/measured");
                const auto setpoint = core->snapshot(key + "/set");
                if (setpoint && (!sample || setpoint->received_at > sample->received_at)) {
                    sample = setpoint;
                }
                if (!sample || !sample->valid || sample->started_at < accepted) {
                    return;
                }
                const auto &bytes = std::any_cast<const std::vector<uint8_t> &>(sample->value);
                if (bytes.size() != segment.edl) {
                    return;
                }
                std::copy(bytes.begin(), bytes.end(), slots.begin() + segment.si);
                if (!provenance || sample->received_at > provenance->received_at) {
                    provenance = sample;
                }
            }
            if (request.generation_valid && !request.generation_valid()) {
                throw ClosedError("command generation ended before cache publication");
            }
            if (request.deadline && runtime::Clock::now() >= *request.deadline) {
                throw TimeoutError("command feedback publication deadline expired");
            }
            ::linkerbot::hand::detail::checked_value(provenance, "missing runtime state: provenance").origin =
                protocol::Origin::command_feedback;
            (void)core->publish(
                domain,
                make_data(o30::project_u8(slots),
                          ::linkerbot::hand::detail::checked_value(provenance, "missing runtime state: provenance")
                              .received_at),
                ::linkerbot::hand::detail::checked_value(provenance, "missing runtime state: provenance"));
        }
    } // namespace

    CommandOutcome submit_command(l6::L6 &device, const CommandRequest &request) {
        require_timeout(request);
        switch (request.semantic) {
        case CommandSemantic::kPositionKp:
            return l6_raw_write(device, request, l6::kPositionKpOpcode, "position_kp");
        case CommandSemantic::kPositionKd:
            return l6_raw_write(device, request, l6::kPositionKdOpcode, "position_kd");
        case CommandSemantic::kTorqueKd:
            return l6_raw_write(device, request, l6::kTorqueKdOpcode, "torque_kd");
        case CommandSemantic::kLimitCompensations:
            return l6_raw_write(device, request, l6::kLimitCompensationOpcode, "limit_compensations");
        case CommandSemantic::kTorques: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, l6::kJointCount>(request);
            auto plan = with_response(l6::protocol::command(id, l6::kTorqueOpcode, {raw.begin(), raw.end()}),
                                      l6::protocol::read_telemetry(id, l6::kTorqueOpcode));
            return acknowledged_write(
                device, strict_classic_vector(std::move(plan), {id, l6::kTorqueOpcode, l6::kJointCount}), request);
        }
        case CommandSemantic::kAngles:
            device.angle.set_angles(fixed_values<uint8_t, l6::kJointCount>(request));
            break;
        case CommandSemantic::kSpeeds:
            device.speed.set_raw_speeds(fixed_values<uint8_t, l6::kJointCount>(request), request.timeout);
            return {1, true};
        case CommandSemantic::kCurrentLimits:
            return l6_acknowledged_write(device, l6::kCurrentLimitOpcode, false, request);
        case CommandSemantic::kStallDetectionTimes:
            return l6_acknowledged_write(device, l6::kStallTimeOpcode, true, request);
        case CommandSemantic::kStallThresholds:
            return l6_acknowledged_write(device, l6::kStallThresholdOpcode, true, request);
        case CommandSemantic::kStallHoldCurrents:
            return l6_acknowledged_write(device, l6::kStallTorqueOpcode, true, request);
        default:
            throw UnsupportedOperationError("command semantic is not implemented for L6");
        }
        return {1, false};
    }

    CommandOutcome submit_command(o6::O6 &device, const CommandRequest &request) {
        require_timeout(request);
        switch (request.semantic) {
        case CommandSemantic::kAngles:
            device.angle.set_angles(fixed_values<uint8_t, o6::kJointCount>(request));
            break;
        case CommandSemantic::kSpeeds: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, o6::kJointCount>(request);
            return acknowledged_write(
                device, with_response(o6::protocol::write_speed(id, raw), o6::protocol::read_speed(id)), request);
        }
        case CommandSemantic::kAccelerations: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, o6::kJointCount>(request);
            return acknowledged_write(
                device, with_response(o6::protocol::write_acceleration(id, raw), o6::protocol::read_acceleration(id)),
                request);
        }
        case CommandSemantic::kCurrentLimits: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, o6::kJointCount>(request);
            return acknowledged_write(device, o6::protocol::write_current_limit(id, raw), request);
        }
        case CommandSemantic::kStallDetectionTimes: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, o6::kJointCount>(request);
            return acknowledged_write(
                device, with_response(o6::protocol::write_stall_time(id, raw), o6::protocol::read_stall_time(id)),
                request);
        }
        case CommandSemantic::kStallThresholds: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, o6::kJointCount>(request);
            return acknowledged_write(
                device,
                with_response(o6::protocol::write_stall_threshold(id, raw), o6::protocol::read_stall_threshold(id)),
                request);
        }
        case CommandSemantic::kStallHoldCurrents: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, o6::kJointCount>(request);
            return acknowledged_write(
                device, with_response(o6::protocol::write_stall_torque(id, raw), o6::protocol::read_stall_torque(id)),
                request);
        }
        default:
            throw UnsupportedOperationError("command semantic is not implemented for O6");
        }
        return {1, false};
    }

    CommandOutcome submit_command(l20lite::L20lite &device, const CommandRequest &request) {
        require_timeout(request);
        switch (request.semantic) {
        case CommandSemantic::kAngles:
            device.angle.set_angles(fixed_values<uint8_t, l20lite::kJointCount>(request));
            break;
        case CommandSemantic::kSpeeds: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, l20lite::kJointCount>(request);
            auto plan = with_response(l20lite::protocol::write_speed(id, raw), l20lite::protocol::read_speed(id));
            plan.domain = "configured_speed";
            return acknowledged_write(device, std::move(plan), request);
        }
        case CommandSemantic::kAccelerations: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, l20lite::kJointCount>(request);
            return acknowledged_write(
                device,
                with_response(l20lite::protocol::write_acceleration(id, raw), l20lite::protocol::read_acceleration(id)),
                request);
        }
        case CommandSemantic::kCurrentLimits: {
            const auto id = required_request_id(device.capabilities());
            const auto raw = fixed_values<uint8_t, l20lite::kJointCount>(request);
            return acknowledged_write(device, l20lite::protocol::write_current_limit(id, raw), request);
        }
        case CommandSemantic::kStallDetectionTimes:
            device.stall.set_times(milliseconds_from_u8<l20lite::kJointCount>(request), request.timeout);
            return {2, true};
        case CommandSemantic::kStallThresholds:
            device.stall.set_thresholds(percent_from_u8<l20lite::kJointCount>(request), request.timeout);
            return {2, true};
        case CommandSemantic::kStallHoldCurrents:
            device.stall.set_torques(percent_from_u8<l20lite::kJointCount>(request), request.timeout);
            return {2, true};
        default:
            throw UnsupportedOperationError("command semantic is not implemented for L20Lite");
        }
        return {2, false};
    }

    CommandOutcome submit_command(l20::L20 &device, const CommandRequest &request) {
        require_timeout(request);
        switch (request.semantic) {
        case CommandSemantic::kAngles:
            device.angle.set_angles(fixed_values<uint8_t, l20::kJointCount>(request));
            break;
        case CommandSemantic::kSpeeds:
            device.speed.set_speeds(percent_from_u8<l20::kJointCount>(request));
            break;
        case CommandSemantic::kCurrentLimits:
            device.current_limit.set_current_limits(fixed_values<uint8_t, l20::kJointCount>(request));
            break;
        default:
            throw UnsupportedOperationError("command semantic is not implemented for L20");
        }
        return {5, false};
    }

    CommandOutcome submit_command(l30::L30 &device, const CommandRequest &request) {
        require_timeout(request);
        switch (request.semantic) {
        case CommandSemantic::kAngles:
            device.angle.set_angles(fixed_values<int16_t, l30::kJointCount>(request));
            break;
        case CommandSemantic::kSpeeds:
            device.speed.set_speeds(fixed_values<uint16_t, l30::kJointCount>(request));
            break;
        case CommandSemantic::kCurrentLimits:
            device.current_limit.set_current_limits(fixed_values<uint16_t, l30::kJointCount>(request));
            break;
        default:
            throw UnsupportedOperationError("command semantic is not implemented for L30");
        }
        return {1, false};
    }

    CommandOutcome submit_command(o30::O30 &device, const CommandRequest &request) {
        require_timeout(request);
        const auto raw = fixed_values<uint8_t, o30::kJointCount>(request);
        switch (request.semantic) {
        case CommandSemantic::kAngles:
            device.angle.set_angles(raw, request.timeout);
            publish_mounted_ack(
                device, request, o30::protocol::kMiPosition, "angle/relay_", [](const auto &values, auto at) {
                    return o30::AngleData{o30::Angle::from_sensor_raw(values), o30::ValueKind::kSetpoint, at};
                });
            break;
        case CommandSemantic::kSpeeds:
            device.speed.set_speeds(raw, request.timeout);
            publish_mounted_ack(
                device, request, o30::protocol::kMiSpeed, "speed/relay_",
                [](const auto &values, auto at) { return o30::SpeedData{values, o30::ValueKind::kSetpoint, at}; });
            break;
        case CommandSemantic::kAccelerations:
            device.acceleration.set_accelerations(raw, request.timeout);
            publish_mounted_ack(device, request, o30::protocol::kMiAcceleration, "acceleration/relay_",
                                [](const auto &values, auto at) {
                                    return o30::AccelerationData{values, o30::ValueKind::kSetpoint, at};
                                });
            break;
        case CommandSemantic::kCurrentLimits:
            device.current.set_current_limits(raw, request.timeout);
            publish_mounted_ack(device, request, o30::protocol::kMiCurrent, "current_limit",
                                [](const auto &values, auto at) { return o30::CurrentLimitData{values, at}; });
            break;
        case CommandSemantic::kStallDetectionTimes:
            device.stall.set_times(raw, request.timeout);
            publish_mounted_ack(
                device, request, o30::protocol::kMiStallTime, "stall/times_relay_",
                [](const auto &values, auto at) { return o30::StallTimeData{values, o30::ValueKind::kSetpoint, at}; });
            break;
        case CommandSemantic::kStallThresholds:
            device.stall.set_thresholds(raw, request.timeout);
            publish_mounted_ack(device, request, o30::protocol::kMiStallThreshold, "stall/thresholds_relay_",
                                [](const auto &values, auto at) {
                                    return o30::StallThresholdData{values, o30::ValueKind::kSetpoint, at};
                                });
            break;
        case CommandSemantic::kStallHoldCurrents:
            device.stall.set_hold_currents(raw, request.timeout);
            publish_mounted_ack(device, request, o30::protocol::kMiStallHoldCurrent, "stall/hold_relay_",
                                [](const auto &values, auto at) {
                                    return o30::StallHoldCurrentData{values, o30::ValueKind::kSetpoint, at};
                                });
            break;
        default:
            throw UnsupportedOperationError("command semantic is not implemented for O30");
        }
        return {3, true};
    }

    CommandOutcome operate_device(l6::L6 &device, DeviceOperation operation, std::chrono::milliseconds timeout) {
        switch (operation) {
        case DeviceOperation::kClearFaults:
            device.fault.clear_faults();
            return {1, false};
        case DeviceOperation::kSaveParametersConfirmed:
            return confirmed_save(device, timeout);
        case DeviceOperation::kSaveParameters:
            device.config.save();
            return {1, false};
        default:
            throw UnsupportedOperationError("device operation is not implemented for L6");
        }
    }

    CommandOutcome operate_device(o6::O6 &device, DeviceOperation operation, std::chrono::milliseconds timeout) {
        if (operation == DeviceOperation::kSaveParametersConfirmed) {
            return confirmed_save(device, timeout);
        }
        if (operation == DeviceOperation::kSaveParameters) {
            device.config.save();
            return {1, false};
        }
        throw UnsupportedOperationError("device operation is not implemented for O6");
    }

    CommandOutcome operate_device(l20lite::L20lite &, DeviceOperation, std::chrono::milliseconds) {
        throw UnsupportedOperationError("device operation is not implemented for L20Lite");
    }

    CommandOutcome operate_device(l20::L20 &device, DeviceOperation operation, std::chrono::milliseconds) {
        if (operation == DeviceOperation::kClearFaults) {
            device.fault.clear_faults();
            return {1, false};
        }
        throw UnsupportedOperationError("device operation is not implemented for L20");
    }

    CommandOutcome operate_device(l30::L30 &device, DeviceOperation operation, std::chrono::milliseconds timeout) {
        switch (operation) {
        case DeviceOperation::kCalibrate:
            device.control.calibrate(timeout);
            return {3, true};
        case DeviceOperation::kEnable:
            device.control.enable(timeout);
            return {1, true};
        case DeviceOperation::kDisable:
            device.control.disable(timeout);
            return {1, true};
        default:
            throw UnsupportedOperationError("device operation is not implemented for L30");
        }
    }

    CommandOutcome operate_device(o30::O30 &, DeviceOperation, std::chrono::milliseconds) {
        throw UnsupportedOperationError("device operation is not implemented for O30");
    }
} // namespace linkerbot::hand::unified_detail
