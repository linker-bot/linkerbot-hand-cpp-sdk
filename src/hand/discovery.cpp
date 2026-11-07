#include <linkerbot/hand/discovery.hpp>

#include "unified/detail/common.hpp"

#include <algorithm>
#include <exception>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace linkerbot::hand {
    namespace {
        [[nodiscard]] bool known_transport_kind(TransportKind kind) noexcept {
            return kind == TransportKind::kCan || kind == TransportKind::kCanFd;
        }

        void require_non_empty(const std::string &value, const char *name) {
            if (value.empty()) {
                throw ValidationError(std::string(name) + " must not be empty");
            }
        }

        template <typename Value> void require_unique(const std::vector<Value> &values, const char *name) {
            const std::unordered_set<Value> unique(values.begin(), values.end());
            if (unique.size() != values.size()) {
                throw ValidationError(std::string(name) + " must not contain duplicates");
            }
        }

        void validate_config(const DiscoveryConfig &config) {
            if (config.response_timeout.count() <= 0) {
                throw ValidationError("discovery response_timeout must be positive");
            }
            if (config.send_timeout.count() <= 0) {
                throw ValidationError("discovery send_timeout must be positive");
            }
            if (config.inter_request_delay.count() < 0) {
                throw ValidationError("discovery inter_request_delay must not be negative");
            }
            if (config.total_timeout.count() <= 0) {
                throw ValidationError("discovery total_timeout must be positive");
            }
            if (config.route_queue_capacity == 0) {
                throw ValidationError("discovery route_queue_capacity must be positive");
            }
            for (const auto &backend : config.backends) {
                require_non_empty(backend, "discovery backend name");
            }
            require_unique(config.backends, "discovery backends");
            for (const auto &channel : config.channels) {
                require_non_empty(channel, "discovery channel name");
            }
            require_unique(config.channels, "discovery channels");

            const auto validate_adapter_config = [](const auto &adapter, const char *name) {
                for (const auto &channel : adapter.channels) {
                    require_non_empty(channel, name);
                }
                require_unique(adapter.channels, name);
                if (adapter.probe_protocols.empty()) {
                    throw ValidationError(std::string(name) + " probe protocols must not be empty");
                }
                for (const auto protocol : adapter.probe_protocols) {
                    if (!known_transport_kind(protocol)) {
                        throw ValidationError(std::string(name) + " has an unknown probe protocol");
                    }
                }
                require_unique(adapter.probe_protocols, name);
                if (adapter.nominal_bitrate == 0 || adapter.data_bitrate == 0) {
                    throw ValidationError(std::string(name) + " bitrates must be positive");
                }
            };
            validate_adapter_config(config.pcan, "PCAN discovery config");
            if (config.pcan.library_path && config.pcan.library_path->empty()) {
                throw ValidationError("PCAN discovery library path must not be empty");
            }
            if (std::find(config.pcan.probe_protocols.begin(), config.pcan.probe_protocols.end(),
                          TransportKind::kCanFd) != config.pcan.probe_protocols.end()) {
                (void)config.pcan.fd_timing.bitrate_string(config.pcan.nominal_bitrate, config.pcan.data_bitrate);
            }
            validate_adapter_config(config.slcan, "SLCAN discovery config");

            if (config.minicanfd.library_path && config.minicanfd.library_path->empty()) {
                throw ValidationError("MiniCANFD discovery library path must not be empty");
            }
            if (config.minicanfd.nominal_bitrate == 0 || config.minicanfd.data_bitrate == 0) {
                throw ValidationError("MiniCANFD discovery bitrates must be positive");
            }
            std::unordered_set<uint64_t> minicanfd_channels;
            for (const auto &channel : config.minicanfd.channels) {
                const auto key = (static_cast<uint64_t>(channel.device_index) << 32U) | channel.channel_index;
                if (!minicanfd_channels.insert(key).second) {
                    throw ValidationError("MiniCANFD discovery channels must not contain duplicates");
                }
            }

            if (config.l30_node_ids.empty()) {
                throw ValidationError("discovery l30_node_ids must not be empty");
            }
            for (const auto node_id : config.l30_node_ids) {
                if (node_id < 1 || node_id > 31) {
                    throw ValidationError("discovery L30 node ID must be in range 1..31");
                }
            }
            require_unique(config.l30_node_ids, "discovery l30_node_ids");

            if (config.o30_request_ids.empty()) {
                throw ValidationError("discovery o30_request_ids must not be empty");
            }
            for (const auto request_id : config.o30_request_ids) {
                if (request_id > 0x3FEU) {
                    throw ValidationError("discovery O30 request ID must be in range 0x000..0x3FE");
                }
            }
            require_unique(config.o30_request_ids, "discovery o30_request_ids");
        }

        [[nodiscard]] bool same_channel(const DiscoveryChannel &left, const DiscoveryChannel &right) noexcept {
            return left.backend() == right.backend() && left.native_name() == right.native_name();
        }

        [[nodiscard]] bool only_missing_side(const std::vector<DiscoveryProblem> &problems) noexcept {
            return !problems.empty() &&
                   std::all_of(problems.begin(), problems.end(),
                               [](const DiscoveryProblem &problem) { return problem.code() == "missing_side"; });
        }

        void apply_runtime_config(HandDeviceConfig &config, const HandRuntimeConfig &runtime) {
            config.auto_start_periodic_reports = runtime.auto_start_periodic_reports;
            config.polling_interval = runtime.polling_interval;
            config.angle_mapping_path = runtime.angle_mapping_path;
            config.command_timeout = runtime.command_timeout;
            config.command_rate_hz = runtime.command_rate_hz;
            config.discrete_queue_capacity = runtime.discrete_queue_capacity;
            config.identity_timeout = runtime.identity_timeout;
            config.transaction_timeout = runtime.transaction_timeout;
            config.shutdown_timeout = runtime.shutdown_timeout;
            config.recovery_attempts = runtime.recovery_attempts;
            config.recovery_backoff = runtime.recovery_backoff;
            config.o30_layout = runtime.o30_layout;
            config.empty_polling_plan_disables = runtime.empty_polling_plan_disables;
            config.runtime_behavior = runtime.runtime_behavior;
            config.read_waiter_capacity = runtime.read_waiter_capacity;
            config.response_timeout = runtime.response_timeout;
            config.recovery_timeout = runtime.recovery_timeout;
            config.recovery_quiet = runtime.recovery_quiet;
        }
    } // namespace

    DiscoveryConfig::DiscoveryConfig() {
        l30_node_ids.reserve(31);
        for (uint8_t node_id = 1; node_id <= 31; ++node_id) {
            l30_node_ids.push_back(node_id);
        }
    }

    DiscoveryChannel::DiscoveryChannel(std::string provider, std::string name, TransportKind kind,
                                       std::optional<uint32_t> nominal_bitrate, std::optional<uint32_t> data_bitrate,
                                       bool supports_classic, std::optional<bool> supports_fd,
                                       std::string configuration_id)
        : DiscoveryChannel(ConstructorTag{}, ConstructorArguments{std::move(provider), name, std::move(name), kind,
                                                                  nominal_bitrate, data_bitrate, supports_classic,
                                                                  supports_fd, std::move(configuration_id)}) {}

    DiscoveryChannel::DiscoveryChannel(std::string backend, std::string name, std::string native_name,
                                       TransportKind kind, std::optional<uint32_t> nominal_bitrate,
                                       std::optional<uint32_t> data_bitrate, bool supports_classic,
                                       std::optional<bool> supports_fd, std::string configuration_id)
        : DiscoveryChannel(ConstructorTag{},
                           ConstructorArguments{std::move(backend), std::move(name), std::move(native_name), kind,
                                                nominal_bitrate, data_bitrate, supports_classic, supports_fd,
                                                std::move(configuration_id)}) {}

    DiscoveryChannel::DiscoveryChannel(ConstructorTag, ConstructorArguments arguments)
        : provider_(std::move(arguments.backend)), name_(std::move(arguments.name)),
          native_name_(std::move(arguments.native_name)), kind_(arguments.kind),
          nominal_bitrate_(arguments.nominal_bitrate), data_bitrate_(arguments.data_bitrate),
          supports_classic_(arguments.supports_classic), supports_fd_(arguments.supports_fd),
          configuration_id_(std::move(arguments.configuration_id)) {
        require_non_empty(provider_, "discovery channel backend");
        require_non_empty(name_, "discovery channel name");
        require_non_empty(native_name_, "discovery channel native name");
        if (!known_transport_kind(kind_)) {
            throw ValidationError("discovery channel has an unknown transport kind");
        }
        if (nominal_bitrate_ && *nominal_bitrate_ == 0) {
            throw ValidationError("discovery nominal bitrate must be positive");
        }
        if (data_bitrate_ && *data_bitrate_ == 0) {
            throw ValidationError("discovery data bitrate must be positive");
        }
        if (kind_ == TransportKind::kCan && data_bitrate_) {
            throw ValidationError("classic CAN discovery channel cannot have a data bitrate");
        }
        if (kind_ == TransportKind::kCanFd && supports_fd_ == false) {
            throw ValidationError("CAN FD discovery channel cannot explicitly reject CAN FD");
        }
    }

    const std::string &DiscoveryChannel::provider() const noexcept { return provider_; }

    const std::string &DiscoveryChannel::backend() const noexcept { return provider_; }

    const std::string &DiscoveryChannel::name() const noexcept { return name_; }

    const std::string &DiscoveryChannel::native_name() const noexcept { return native_name_; }

    TransportKind DiscoveryChannel::kind() const noexcept { return kind_; }

    std::optional<uint32_t> DiscoveryChannel::nominal_bitrate() const noexcept { return nominal_bitrate_; }

    std::optional<uint32_t> DiscoveryChannel::data_bitrate() const noexcept { return data_bitrate_; }

    bool DiscoveryChannel::supports_classic() const noexcept { return supports_classic_; }

    std::optional<bool> DiscoveryChannel::supports_fd() const noexcept { return supports_fd_; }

    DiscoveryAddress::DiscoveryAddress(std::string scheme, uint32_t request_id, uint32_t response_id, bool extended_id,
                                       std::optional<uint8_t> node_id, std::optional<uint8_t> host_id)
        : DiscoveryAddress(ConstructorTag{}, ConstructorArguments{std::move(scheme), request_id, response_id,
                                                                  extended_id, node_id, host_id}) {}

    DiscoveryAddress::DiscoveryAddress(ConstructorTag, ConstructorArguments arguments)
        : scheme_(std::move(arguments.scheme)), request_id_(arguments.request_id), response_id_(arguments.response_id),
          extended_id_(arguments.extended_id), node_id_(arguments.node_id), host_id_(arguments.host_id) {
        require_non_empty(scheme_, "discovery address scheme");
        const auto maximum = extended_id_ ? 0x1FFFFFFFU : 0x7FFU;
        if (request_id_ > maximum || response_id_ > maximum) {
            throw ValidationError("discovery address does not fit the selected CAN ID format");
        }
    }

    const std::string &DiscoveryAddress::scheme() const noexcept { return scheme_; }

    uint32_t DiscoveryAddress::request_id() const noexcept { return request_id_; }

    uint32_t DiscoveryAddress::response_id() const noexcept { return response_id_; }

    bool DiscoveryAddress::extended_id() const noexcept { return extended_id_; }

    std::optional<uint8_t> DiscoveryAddress::node_id() const noexcept { return node_id_; }

    std::optional<uint8_t> DiscoveryAddress::host_id() const noexcept { return host_id_; }

    DiscoveryProblem::DiscoveryProblem(std::string code, std::string message)
        : code_(std::move(code)), message_(std::move(message)) {
        require_non_empty(code_, "discovery problem code");
        require_non_empty(message_, "discovery problem message");
    }

    const std::string &DiscoveryProblem::code() const noexcept { return code_; }

    const std::string &DiscoveryProblem::message() const noexcept { return message_; }

    DiscoveryFailure::DiscoveryFailure(std::string operation, std::string code, std::string message,
                                       std::optional<std::string> channel, std::optional<std::string> backend,
                                       std::optional<std::string> stage, std::optional<std::string> native_channel,
                                       std::optional<std::string> error_type, std::optional<int> native_status)
        : operation_(std::move(operation)), code_(std::move(code)), message_(std::move(message)),
          channel_(std::move(channel)), backend_(std::move(backend)), stage_(std::move(stage)),
          native_channel_(std::move(native_channel)), error_type_(std::move(error_type)),
          native_status_(native_status) {
        require_non_empty(operation_, "discovery failure operation");
        require_non_empty(code_, "discovery failure code");
        require_non_empty(message_, "discovery failure message");
        if (channel_) {
            require_non_empty(*channel_, "discovery failure channel");
        }
        if (backend_) {
            require_non_empty(*backend_, "discovery failure backend");
        }
        if (stage_) {
            require_non_empty(*stage_, "discovery failure stage");
        }
        if (native_channel_) {
            require_non_empty(*native_channel_, "discovery failure native channel");
        }
        if (error_type_) {
            require_non_empty(*error_type_, "discovery failure error type");
        }
    }

    const std::string &DiscoveryFailure::operation() const noexcept { return operation_; }

    const std::string &DiscoveryFailure::code() const noexcept { return code_; }

    const std::string &DiscoveryFailure::message() const noexcept { return message_; }

    const std::optional<std::string> &DiscoveryFailure::channel() const noexcept { return channel_; }

    const std::optional<std::string> &DiscoveryFailure::backend() const noexcept { return backend_; }

    const std::optional<std::string> &DiscoveryFailure::stage() const noexcept { return stage_; }

    const std::optional<std::string> &DiscoveryFailure::native_channel() const noexcept { return native_channel_; }

    const std::optional<std::string> &DiscoveryFailure::error_type() const noexcept { return error_type_; }

    std::optional<int> DiscoveryFailure::native_status() const noexcept { return native_status_; }

    DiscoveryEnumeration::DiscoveryEnumeration(std::vector<DiscoveryChannel> channels,
                                               std::vector<DiscoveryFailure> failures)
        : channels_(std::move(channels)), failures_(std::move(failures)) {
        for (size_t index = 0; index < channels_.size(); ++index) {
            for (size_t other = index + 1; other < channels_.size(); ++other) {
                if (same_channel(channels_[index], channels_[other])) {
                    throw ValidationError("discovery enumeration must not contain duplicate channels");
                }
            }
        }
    }

    const std::vector<DiscoveryChannel> &DiscoveryEnumeration::channels() const noexcept { return channels_; }

    const std::vector<DiscoveryFailure> &DiscoveryEnumeration::failures() const noexcept { return failures_; }

    DiscoveryTransportSpec::DiscoveryTransportSpec(std::string provider, std::string channel, TransportKind kind,
                                                   HandDevice::TransportFactory factory,
                                                   std::optional<uint32_t> nominal_bitrate,
                                                   std::optional<uint32_t> data_bitrate,
                                                   std::optional<std::string> native_channel,
                                                   std::string configuration_id)
        : DiscoveryTransportSpec(ConstructorTag{},
                                 ConstructorArguments{std::move(provider), std::move(channel), kind, std::move(factory),
                                                      nominal_bitrate, data_bitrate, std::move(native_channel),
                                                      std::move(configuration_id)}) {}

    DiscoveryTransportSpec::DiscoveryTransportSpec(ConstructorTag, ConstructorArguments arguments)
        : provider_(std::move(arguments.provider)), channel_(std::move(arguments.channel)),
          native_channel_(arguments.native_channel ? std::move(*arguments.native_channel) : channel_),
          kind_(arguments.kind), factory_(std::move(arguments.factory)), nominal_bitrate_(arguments.nominal_bitrate),
          data_bitrate_(arguments.data_bitrate), configuration_id_(std::move(arguments.configuration_id)) {
        require_non_empty(provider_, "discovery transport provider");
        require_non_empty(channel_, "discovery transport channel");
        require_non_empty(native_channel_, "discovery transport native channel");
        if (!known_transport_kind(kind_)) {
            throw ValidationError("discovery transport has an unknown kind");
        }
        if (!factory_) {
            throw ValidationError("discovery transport factory must not be empty");
        }
        if (nominal_bitrate_ && *nominal_bitrate_ == 0) {
            throw ValidationError("discovery nominal bitrate must be positive");
        }
        if (data_bitrate_ && *data_bitrate_ == 0) {
            throw ValidationError("discovery data bitrate must be positive");
        }
        if (kind_ == TransportKind::kCan && data_bitrate_) {
            throw ValidationError("classic CAN discovery transport cannot have a data bitrate");
        }
    }

    const std::string &DiscoveryTransportSpec::provider() const noexcept { return provider_; }

    const std::string &DiscoveryTransportSpec::channel() const noexcept { return channel_; }

    const std::string &DiscoveryTransportSpec::native_channel() const noexcept { return native_channel_; }

    TransportKind DiscoveryTransportSpec::kind() const noexcept { return kind_; }

    std::optional<uint32_t> DiscoveryTransportSpec::nominal_bitrate() const noexcept { return nominal_bitrate_; }

    std::optional<uint32_t> DiscoveryTransportSpec::data_bitrate() const noexcept { return data_bitrate_; }

    const std::string &DiscoveryChannel::configuration_id() const noexcept { return configuration_id_; }
    const std::string &DiscoveryTransportSpec::configuration_id() const noexcept { return configuration_id_; }

    HandDevice::TransportFactory DiscoveryTransportSpec::factory() const { return factory_; }

    DiscoveryConnectionKey DiscoveryTransportSpec::connection_key() const {
        return {provider_, native_channel_, kind_, nominal_bitrate_, data_bitrate_, configuration_id_};
    }

    DiscoveryCandidate::DiscoveryCandidate(DiscoveryChannel channel, TransportKind wire_protocol,
                                           DiscoveryAddress address, DiscoveryStatus status, DeviceIdentity identity,
                                           DiscoveryTransportSpec transport,
                                           std::optional<DeviceCapabilities> capabilities,
                                           std::vector<DiscoveryProblem> problems)
        : channel_(std::move(channel)), wire_protocol_(wire_protocol), address_(std::move(address)), status_(status),
          identity_(std::move(identity)), transport_(std::move(transport)), capabilities_(std::move(capabilities)),
          problems_(std::move(problems)) {
        if (!known_transport_kind(wire_protocol_)) {
            throw ValidationError("discovery candidate has an unknown wire protocol");
        }
        if (status_ != DiscoveryStatus::kVerified && status_ != DiscoveryStatus::kPartial) {
            throw ValidationError("discovery candidate has an unknown status");
        }
        if (wire_protocol_ == TransportKind::kCanFd && channel_.kind() != TransportKind::kCanFd) {
            throw ValidationError("classic CAN channel cannot carry a CAN FD candidate");
        }
        if (transport_.configuration_id() != channel_.configuration_id() ||
            transport_.provider() != channel_.provider() || transport_.channel() != channel_.name() ||
            transport_.native_channel() != channel_.native_name() || transport_.kind() != wire_protocol_) {
            throw ValidationError("discovery candidate transport does not match its channel");
        }
        const auto expected_data_bitrate =
            wire_protocol_ == TransportKind::kCanFd ? channel_.data_bitrate() : std::optional<uint32_t>{};
        if (transport_.nominal_bitrate() != channel_.nominal_bitrate() ||
            transport_.data_bitrate() != expected_data_bitrate) {
            throw ValidationError("discovery candidate transport bitrates do not match its channel");
        }

        if (status_ == DiscoveryStatus::kVerified) {
            if (identity_.model == Model::kUnknown || identity_.side == HandSide::kUnknown) {
                throw ValidationError("verified discovery candidate requires model and side");
            }
            if (!capabilities_) {
                throw ValidationError("verified discovery candidate requires capabilities");
            }
            if (capabilities_->model != identity_.model || capabilities_->side != identity_.side) {
                throw ValidationError("discovery capabilities do not match candidate identity");
            }
            if (!problems_.empty()) {
                throw ValidationError("verified discovery candidate cannot contain problems");
            }
        } else if (problems_.empty()) {
            throw ValidationError("partial discovery candidate requires at least one problem");
        }
    }

    const DiscoveryChannel &DiscoveryCandidate::channel() const noexcept { return channel_; }

    TransportKind DiscoveryCandidate::wire_protocol() const noexcept { return wire_protocol_; }

    const DiscoveryAddress &DiscoveryCandidate::address() const noexcept { return address_; }

    DiscoveryStatus DiscoveryCandidate::status() const noexcept { return status_; }

    const DeviceIdentity &DiscoveryCandidate::identity() const noexcept { return identity_; }

    Model DiscoveryCandidate::model() const noexcept { return identity_.model; }

    HandSide DiscoveryCandidate::side() const noexcept { return identity_.side; }

    const DiscoveryTransportSpec &DiscoveryCandidate::transport() const noexcept { return transport_; }

    const DeviceCapabilities *DiscoveryCandidate::capabilities() const noexcept {
        return capabilities_ ? &*capabilities_ : nullptr;
    }

    const std::vector<DiscoveryProblem> &DiscoveryCandidate::problems() const noexcept { return problems_; }

    HandDeviceConfig DiscoveryCandidate::make_config() const { return make_config(HandRuntimeConfig{}); }

    HandDeviceConfig DiscoveryCandidate::make_config(const HandRuntimeConfig &runtime) const {
        return make_config(runtime, std::nullopt);
    }

    HandDeviceConfig DiscoveryCandidate::make_config(const HandRuntimeConfig &runtime,
                                                     std::optional<HandSide> requested_side) const {
        if (identity_.model == Model::kUnknown) {
            throw IncompleteIdentityError("discovery candidate model is unknown",
                                          {"discovery_make_config", "Unknown", channel_.name()});
        }

        auto resolved_side = identity_.side;
        if (requested_side) {
            if (*requested_side == HandSide::kUnknown) {
                throw ValidationError(
                    "explicit discovery side must be Left or Right",
                    {"discovery_make_config", unified_detail::model_name(identity_.model), channel_.name()});
            }
            if (resolved_side != HandSide::kUnknown && resolved_side != *requested_side) {
                throw ValidationError(
                    "requested side disagrees with discovered identity",
                    {"discovery_make_config", unified_detail::model_name(identity_.model), channel_.name()});
            }
            resolved_side = *requested_side;
        }

        if (status_ == DiscoveryStatus::kPartial) {
            if (!only_missing_side(problems_)) {
                throw IncompleteIdentityError(
                    "discovery candidate has unrecoverable identity problems",
                    {"discovery_make_config", unified_detail::model_name(identity_.model), channel_.name()});
            }
            if (!requested_side) {
                throw IncompleteIdentityError(
                    "candidate side is unknown; pass side explicitly",
                    {"discovery_make_config", unified_detail::model_name(identity_.model), channel_.name()});
            }
        }
        if (resolved_side == HandSide::kUnknown) {
            throw IncompleteIdentityError(
                "discovery candidate side is unknown",
                {"discovery_make_config", unified_detail::model_name(identity_.model), channel_.name()});
        }

        HandDeviceConfig config;
        apply_runtime_config(config, runtime);
        config.model = identity_.model;
        config.side = resolved_side;
        config.interface_name = channel_.name();
        if (identity_.model == Model::kL30) {
            if (!address_.node_id()) {
                throw IncompleteIdentityError("L30 discovery candidate has no node ID",
                                              {"discovery_make_config", "L30", channel_.name()});
            }
            config.address = *address_.node_id();
            config.host_address = address_.host_id();
        } else {
            config.address = address_.request_id();
        }
        unified_detail::validate_config(config);
        return config;
    }

    std::unique_ptr<HandDevice> DiscoveryCandidate::create_hand_device() const {
        return create_hand_device(HandRuntimeConfig{});
    }

    std::unique_ptr<HandDevice> DiscoveryCandidate::create_hand_device(const HandRuntimeConfig &runtime) const {
        return std::unique_ptr<HandDevice>(new HandDevice(make_config(runtime), transport_.factory(), identity_));
    }

    std::unique_ptr<HandDevice> DiscoveryCandidate::create_hand_device(const HandRuntimeConfig &runtime,
                                                                       HandSide side) const {
        auto expected_identity = identity_;
        expected_identity.side = side;
        return std::unique_ptr<HandDevice>(
            new HandDevice(make_config(runtime, side), transport_.factory(), std::move(expected_identity)));
    }

    DiscoveryReport::DiscoveryReport(std::vector<DiscoveryChannel> channels, std::vector<DiscoveryCandidate> candidates,
                                     std::vector<DiscoveryFailure> failures,
                                     std::chrono::steady_clock::duration elapsed)
        : channels_(std::move(channels)), candidates_(std::move(candidates)), failures_(std::move(failures)),
          elapsed_(elapsed) {
        if (elapsed_ < std::chrono::steady_clock::duration::zero()) {
            throw ValidationError("discovery elapsed duration must not be negative");
        }

        for (size_t index = 0; index < channels_.size(); ++index) {
            for (size_t other = index + 1; other < channels_.size(); ++other) {
                if (same_channel(channels_[index], channels_[other])) {
                    throw ValidationError("discovery report must not contain duplicate channels");
                }
            }
        }
        for (const auto &candidate : candidates_) {
            const auto found = std::any_of(channels_.begin(), channels_.end(), [&candidate](const auto &channel) {
                return same_channel(channel, candidate.channel());
            });
            if (!found) {
                throw ValidationError("discovery candidate channel is absent from its report");
            }
        }
    }

    const std::vector<DiscoveryChannel> &DiscoveryReport::channels() const noexcept { return channels_; }

    const std::vector<DiscoveryCandidate> &DiscoveryReport::candidates() const noexcept { return candidates_; }

    const std::vector<DiscoveryFailure> &DiscoveryReport::failures() const noexcept { return failures_; }

    std::chrono::steady_clock::duration DiscoveryReport::elapsed() const noexcept { return elapsed_; }

    std::vector<std::reference_wrapper<const DiscoveryCandidate>> DiscoveryReport::verified_candidates() const {
        std::vector<std::reference_wrapper<const DiscoveryCandidate>> result;
        for (const auto &candidate : candidates_) {
            if (candidate.status() == DiscoveryStatus::kVerified) {
                result.emplace_back(std::cref(candidate));
            }
        }
        return result;
    }

    std::vector<std::reference_wrapper<const DiscoveryCandidate>> DiscoveryReport::partial_candidates() const {
        std::vector<std::reference_wrapper<const DiscoveryCandidate>> result;
        for (const auto &candidate : candidates_) {
            if (candidate.status() == DiscoveryStatus::kPartial) {
                result.emplace_back(std::cref(candidate));
            }
        }
        return result;
    }

    const DiscoveryCandidate &DiscoveryReport::require_one(bool allow_partial) const {
        if (candidates_.empty()) {
            throw NoHandFoundError("discovery returned no hand candidates", {"discovery_require_one"});
        }
        if (candidates_.size() != 1) {
            std::ostringstream message;
            message << "discovery returned " << candidates_.size() << " hand candidates";
            throw AmbiguousHandError(message.str(), {"discovery_require_one"});
        }
        const auto &candidate = candidates_.front();
        if (!allow_partial && candidate.status() == DiscoveryStatus::kPartial) {
            throw IncompleteIdentityError("the only discovered hand has incomplete identity",
                                          {"discovery_require_one"});
        }
        return candidate;
    }

    DiscoveryReport discover_hands(const DiscoveryConfig &config, DiscoverySource &source) {
        validate_config(config);
        try {
            return source.scan(config);
        } catch (const DiscoveryError &) {
            throw;
        } catch (const ValidationError &) {
            throw;
        } catch (...) {
            auto cause = std::current_exception();
            try {
                std::rethrow_exception(cause);
            } catch (const std::exception &error) {
                throw DiscoveryError(std::string("discovery source failed: ") + error.what(), {"discover_hands"},
                                     cause);
            } catch (...) {
                throw DiscoveryError("discovery source failed with a non-standard exception", {"discover_hands"},
                                     cause);
            }
        }
    }
} // namespace linkerbot::hand
