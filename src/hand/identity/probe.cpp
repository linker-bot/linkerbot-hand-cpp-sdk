#include "probe.hpp"
#include "../unified/detail/python_identity.hpp"
#include "../unified/detail/runtime_access.hpp"

#include <algorithm>
#include <any>
#include <array>
#include <optional>
#include <string>
#include <utility>

#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/l20/protocol_backend.hpp>
#include <linkerbot/hand/l20lite/protocol_backend.hpp>
#include <linkerbot/hand/l30/protocol_backend.hpp>
#include <linkerbot/hand/l30/version.hpp>
#include <linkerbot/hand/l6/protocol_backend.hpp>
#include <linkerbot/hand/o30/protocol.hpp>
#include <linkerbot/hand/o30/transaction.hpp>
#include <linkerbot/hand/o30/version.hpp>
#include <linkerbot/hand/o6/protocol_backend.hpp>

namespace linkerbot::hand::identity_probe {
    namespace {
        [[nodiscard]] std::optional<IdentitySide> identity_side(HandSide side) {
            switch (side) {
            case HandSide::kLeft:
                return IdentitySide::kLeft;
            case HandSide::kRight:
                return IdentitySide::kRight;
            case HandSide::kUnknown:
                return std::nullopt;
            }
            return std::nullopt;
        }

        [[nodiscard]] HandSide public_side(IdentitySide side) noexcept {
            return side == IdentitySide::kLeft ? HandSide::kLeft : HandSide::kRight;
        }

        void assign_if_present(std::optional<std::string> &target, const std::string &value) {
            if (!value.empty()) {
                target = value;
            }
        }

        template <class Version> struct ClassicVersions {
            const Version &pcb;
            const Version &firmware;
            const Version &mechanical;
        };
        template <class Version>
        ClassicVersions(const Version &, const Version &, const Version &) -> ClassicVersions<Version>;

        template <typename Version>
        [[nodiscard]] DeviceIdentity
        classic_identity(const Target &target, const ModelRule &rule, const std::string &product_code,
                         const ClassicVersions<Version> &versions, std::chrono::steady_clock::time_point timestamp) {
            const auto side = identity_side(target.expected_side);
            if (!side) {
                throw IdentityProtocolError("classic CAN identity requires a configured hand side");
            }

            const auto factory = parse_factory_identity(product_code, rule.factory_prefixes, *side);
            DeviceIdentity identity;
            identity.model = target.model;
            identity.side = public_side(factory.side);
            identity.address = target.request_id;
            identity.product_code = factory.product_code;
            identity.serial_number = factory.serial_number;
            identity.firmware_version = versions.firmware.to_string();
            identity.hardware_version = versions.pcb.to_string();
            identity.mechanical_version = versions.mechanical.to_string();
            identity.timestamp = timestamp;
            return identity;
        }

        template <typename Version, typename SerialPlan, typename PcbPlan, typename FirmwarePlan,
                  typename MechanicalPlan>
        [[nodiscard]] DeviceIdentity probe_classic(const std::shared_ptr<runtime::DeviceRuntime> &runtime,
                                                   const Target &target, const ModelRule &rule,
                                                   runtime::Deadline deadline, SerialPlan serial_plan, PcbPlan pcb_plan,
                                                   FirmwarePlan firmware_plan, MechanicalPlan mechanical_plan) {
            if (target.request_id != 0x27U && target.request_id != 0x28U) {
                throw IdentityProtocolError("unknown classic CAN identity endpoint");
            }

            auto operation = runtime->operation("identity.classic", deadline);
            if (unified_detail::RuntimeAccess::uses_response_windows(*runtime)) {
                std::array<protocol::TransactionPlan, 4> parts{
                    serial_plan(target.request_id), pcb_plan(target.request_id), firmware_plan(target.request_id),
                    mechanical_plan(target.request_id)};
                parts[0].consume = unified_detail::python_serial(target.request_id, target.model);
                protocol::TransactionPlan batch;
                batch.domain = "version.identity";
                batch.gap = std::chrono::milliseconds{5};
                for (const auto &part : parts) {
                    batch.frames.insert(batch.frames.end(), part.frames.begin(), part.frames.end());
                    batch.keys.insert(batch.keys.end(), part.keys.begin(), part.keys.end());
                }
                batch.consume = [parts = std::move(parts),
                                 values = std::array<std::any, 4>{}](const can::Message &message) mutable {
                    bool matched = false;
                    for (size_t i = 0; i < parts.size(); ++i) {
                        auto progress = parts[i].consume(message);
                        if (progress.kind == protocol::ProgressKind::failed) {
                            return progress;
                        }
                        matched = matched || progress.kind != protocol::ProgressKind::unmatched;
                        if (i == 0 && progress.kind == protocol::ProgressKind::pending) {
                            values[0].reset();
                        }
                        if (progress.kind == protocol::ProgressKind::complete) {
                            if (values[i].has_value()) {
                                const bool conflict = i == 0 ? std::any_cast<std::string>(values[i]) !=
                                                                   std::any_cast<std::string>(progress.value)
                                                             : std::any_cast<Version>(values[i]).to_string() !=
                                                                   std::any_cast<Version>(progress.value).to_string();
                                if (conflict) {
                                    throw ProtocolError("identity response conflicts with a duplicate");
                                }
                            }
                            values[i] = std::move(progress.value);
                        }
                    }
                    if (std::all_of(values.begin(), values.end(),
                                    [](const auto &value) { return value.has_value(); })) {
                        return protocol::Progress::complete(values);
                    }
                    return matched ? protocol::Progress::pending() : protocol::Progress::unmatched();
                };
                const auto values = runtime->query<std::array<std::any, 4>>(std::move(batch), deadline);
                const auto pcb = std::any_cast<Version>(values[1]);
                const auto firmware = std::any_cast<Version>(values[2]);
                const auto mechanical = std::any_cast<Version>(values[3]);
                return classic_identity(target, rule, std::any_cast<std::string>(values[0]),
                                        ClassicVersions{pcb, firmware, mechanical},
                                        runtime->response_provenance().received_at);
            }
            auto product_code = runtime->query<std::string>(serial_plan(target.request_id), deadline);
            const auto pcb = runtime->query<Version>(pcb_plan(target.request_id), deadline);
            const auto firmware = runtime->query<Version>(firmware_plan(target.request_id), deadline);
            const auto mechanical = runtime->query<Version>(mechanical_plan(target.request_id), deadline);
            return classic_identity(target, rule, product_code, ClassicVersions{pcb, firmware, mechanical},
                                    runtime->response_provenance().received_at);
        }

        [[nodiscard]] protocol::TransactionPlan l30_plan(uint8_t node_id, uint8_t host_id, uint8_t subcommand) {
            return l30::protocol::request_plan(l30::protocol::request_frame(
                node_id, host_id, l30::protocol::kParentConfig, subcommand, l30::protocol::kAccessRead,
                l30::protocol::empty_request_payload(), l30::protocol::kEmptyRequestBytes));
        }

        [[nodiscard]] DeviceIdentity probe_l30(const std::shared_ptr<runtime::DeviceRuntime> &runtime,
                                               const Target &target, runtime::Deadline deadline) {
            if (target.request_id < l30::protocol::kNodeIdMin || target.request_id > l30::protocol::kNodeIdMax ||
                target.host_id > l30::protocol::kHostIdMax) {
                throw ValidationError("L30 identity target requires node ID 1..31 and host ID 0..31");
            }

            const auto node_id = static_cast<uint8_t>(target.request_id);
            auto operation = runtime->operation("identity.l30", deadline);
            const auto raw_info = runtime->query<std::vector<uint8_t>>(
                l30_plan(node_id, target.host_id, l30::protocol::kSubDeviceInfo), deadline);
            auto info = l30::decode_device_info_response(raw_info);
            const auto raw_code = runtime->query<std::vector<uint8_t>>(
                l30_plan(node_id, target.host_id, l30::protocol::kSubProductCode), deadline);
            const auto product_code = l30::protocol::decode_ascii_response(raw_code.data(), raw_code.size());
            l30::verify_device_info(info, node_id, identity_side(target.expected_side), product_code);

            DeviceIdentity identity;
            identity.model = Model::kL30;
            identity.side = info.hand_side == l30::HandSide::kLeft ? HandSide::kLeft : HandSide::kRight;
            identity.address = info.node_id;
            identity.product_code = product_code;
            identity.serial_number = std::to_string(info.serial_number);
            identity.firmware_version = info.software_version.to_string();
            identity.hardware_version = info.hardware_version.to_string();
            identity.mechanical_version = info.structure_version.to_string();
            identity.timestamp = runtime->response_provenance().received_at;
            return identity;
        }

        [[nodiscard]] std::vector<uint8_t> read_o30_range(const std::shared_ptr<runtime::DeviceRuntime> &runtime,
                                                          uint32_t request_id, uint8_t offset, uint8_t length,
                                                          runtime::Deadline deadline) {
            return runtime->query<std::vector<uint8_t>>(
                o30::protocol::transaction(request_id, o30::protocol::kMiProductInfo, offset, length, {}, false),
                deadline);
        }

        [[nodiscard]] DeviceIdentity probe_o30(const std::shared_ptr<runtime::DeviceRuntime> &runtime,
                                               const Target &target, runtime::Deadline deadline) {
            if (target.request_id > 0x3FFU) {
                throw ValidationError("O30 identity request ID must leave an 11-bit response ID");
            }

            auto operation = runtime->operation("identity.o30", deadline);
            auto blob = read_o30_range(runtime, target.request_id, 0, o30::protocol::kProductInfoBytes, deadline);
            if (o30::product_info_uses_documented_layout(blob)) {
                auto tail = read_o30_range(runtime, target.request_id, 0xE1, 0x18, deadline);
                blob.insert(blob.end(), tail.begin(), tail.end());
            }
            auto info = o30::decode_product_info_blob(blob);
            // Discovery may intentionally leave O30 side unresolved. Validate
            // every device-reported identity field first, then allow an
            // explicit HandDevice side to resolve only an absent side.
            o30::verify_product_info(info, std::nullopt);

            auto reported_side = identity_detail::parse_side(info.hand_side);
            if (!reported_side) {
                const auto parts = identity_detail::split_product(info.model);
                if (parts.size() >= 4) {
                    reported_side = identity_detail::parse_side(parts[3]);
                }
            }
            const auto expected_side = identity_side(target.expected_side);
            if (expected_side && reported_side && *expected_side != *reported_side) {
                throw IdentityMismatchError("O30 side does not match expected side");
            }

            DeviceIdentity identity;
            identity.model = Model::kO30;
            if (reported_side) {
                identity.side = public_side(*reported_side);
            } else if (expected_side) {
                identity.side = public_side(*expected_side);
            }
            assign_if_present(identity.product_code, info.model);
            assign_if_present(identity.device_uid, info.device_uid);
            assign_if_present(identity.firmware_version, info.app_version);
            assign_if_present(identity.protocol_name, info.protocol_name);
            assign_if_present(identity.protocol_version, info.protocol_version);
            assign_if_present(identity.hardware_version, info.control_hw_version);
            assign_if_present(identity.mechanical_version, info.mechanical_version);
            identity.timestamp = runtime->response_provenance().received_at;
            return identity;
        }
    } // namespace

    const std::vector<ModelRule> &model_rules() {
        static const std::vector<ModelRule> rules{
            {Model::kL6, TransportKind::kCan, {"LHL6"}},
            {Model::kO6, TransportKind::kCan, {"LHO6"}},
            {Model::kL20Lite, TransportKind::kCan, {"LHT10"}},
            {Model::kL20, TransportKind::kCan, {"L20", "LHT20", "L25", "LHL25", "LHG20"}},
            {Model::kL30, TransportKind::kCanFd, {}},
            {Model::kO30, TransportKind::kCanFd, {}},
        };
        return rules;
    }

    const ModelRule &rule_for(Model model) {
        const auto &rules = model_rules();
        const auto found =
            std::find_if(rules.begin(), rules.end(), [model](const ModelRule &rule) { return rule.model == model; });
        if (found == rules.end()) {
            throw UnsupportedOperationError("no identity probe is registered for the requested model");
        }
        return *found;
    }

    Model classic_model_from_product_code(std::string_view code) {
        const auto parts = identity_detail::split_product(code);
        if (parts.empty()) {
            return Model::kUnknown;
        }
        const auto prefix = identity_detail::upper(parts.front());
        for (const auto &rule : model_rules()) {
            for (const auto expected : rule.factory_prefixes) {
                if (prefix == expected) {
                    return rule.model;
                }
            }
        }
        return Model::kUnknown;
    }

    protocol::TransactionPlan classic_product_plan(uint32_t arbitration_id) {
        return l6::protocol::read_serial(arbitration_id);
    }

    DeviceIdentity probe(const std::shared_ptr<runtime::DeviceRuntime> &runtime, const Target &target,
                         std::chrono::milliseconds timeout) {
        if (!runtime) {
            throw ValidationError("identity probe requires a runtime");
        }
        if (timeout.count() <= 0) {
            throw ValidationError("identity probe timeout must be positive");
        }

        const auto &rule = rule_for(target.model);
        const auto deadline = runtime::deadline_after(timeout);
        switch (target.model) {
        case Model::kL6:
            return probe_classic<l6::Version>(
                runtime, target, rule, deadline, l6::protocol::read_serial,
                [](uint32_t id) { return l6::protocol::read_version(id, l6::kPcbVersionOpcode); },
                [](uint32_t id) { return l6::protocol::read_version(id, l6::kFirmwareVersionOpcode); },
                [](uint32_t id) { return l6::protocol::read_version(id, l6::kMechanicalVersionOpcode); });
        case Model::kO6:
            return probe_classic<o6::Version>(runtime, target, rule, deadline, o6::protocol::read_version_sn,
                                              o6::protocol::read_version_pcb, o6::protocol::read_version_firmware,
                                              o6::protocol::read_version_mechanical);
        case Model::kL20Lite:
            return probe_classic<l20lite::Version>(runtime, target, rule, deadline, l20lite::protocol::read_version_sn,
                                                   l20lite::protocol::read_version_pcb,
                                                   l20lite::protocol::read_version_firmware,
                                                   l20lite::protocol::read_version_mechanical);
        case Model::kL20:
            return probe_classic<l20::Version>(runtime, target, rule, deadline, l20::protocol::read_version_sn,
                                               l20::protocol::read_version_pcb, l20::protocol::read_version_firmware,
                                               l20::protocol::read_version_mechanical);
        case Model::kL30:
            return probe_l30(runtime, target, deadline);
        case Model::kO30:
            return probe_o30(runtime, target, deadline);
        case Model::kUnknown:
            break;
        }
        throw UnsupportedOperationError("no identity probe is registered for the requested model");
    }
} // namespace linkerbot::hand::identity_probe
