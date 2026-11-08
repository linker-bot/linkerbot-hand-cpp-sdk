#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <linkerbot/can/fake_transport.hpp>
#include <linkerbot/can/message.hpp>
#include <linkerbot/hand/discovery.hpp>

using namespace linkerbot::hand;

namespace {
    using linkerbot::can::FakeTransport;
    using linkerbot::can::Message;

    [[nodiscard]] HandDevice::TransportFactory fake_factory(int *created = nullptr) {
        return [created]() {
            if (created) {
                ++*created;
            }
            return std::unique_ptr<linkerbot::can::CANTransport>(std::make_unique<linkerbot::can::FakeTransport>());
        };
    }

    [[nodiscard]] DiscoveryChannel channel(std::string name = "can0") {
        return DiscoveryChannel{"fake", std::move(name), TransportKind::kCan, 1'000'000U};
    }

    [[nodiscard]] DiscoveryTransportSpec transport(std::string name = "can0", int *created = nullptr) {
        return DiscoveryTransportSpec{"fake", std::move(name), TransportKind::kCan, fake_factory(created), 1'000'000U};
    }

    [[nodiscard]] DiscoveryCandidate verified_candidate(std::string name = "can0", int *created = nullptr) {
        DeviceIdentity identity;
        identity.model = Model::kL6;
        identity.side = HandSide::kLeft;
        identity.address = 0x28U;
        identity.product_code = "LHL6-L";
        identity.timestamp = std::chrono::steady_clock::now();
        auto selected_channel = channel(name);
        auto selected_transport = transport(std::move(name), created);
        return DiscoveryCandidate{std::move(selected_channel),
                                  TransportKind::kCan,
                                  DiscoveryAddress{"classic_can_id", 0x28U, 0x28U, false},
                                  DiscoveryStatus::kVerified,
                                  std::move(identity),
                                  std::move(selected_transport),
                                  declared_capabilities(Model::kL6, HandSide::kLeft)};
    }

    [[nodiscard]] DiscoveryCandidate partial_candidate(std::string name = "can0", int *created = nullptr,
                                                       std::string problem_code = "missing_side") {
        DeviceIdentity identity;
        identity.model = Model::kL6;
        identity.side = HandSide::kUnknown;
        identity.address = 0x28U;
        identity.timestamp = std::chrono::steady_clock::now();
        auto selected_channel = channel(name);
        auto selected_transport = transport(std::move(name), created);
        return DiscoveryCandidate{std::move(selected_channel),
                                  TransportKind::kCan,
                                  DiscoveryAddress{"classic_can_id", 0x28U, 0x28U, false},
                                  DiscoveryStatus::kPartial,
                                  std::move(identity),
                                  std::move(selected_transport),
                                  {},
                                  {DiscoveryProblem{std::move(problem_code), "device identity is incomplete"}}};
    }

    [[nodiscard]] DiscoveryCandidate candidate_for(const DiscoveryChannel &selected_channel,
                                                   DiscoveryStatus status = DiscoveryStatus::kVerified,
                                                   uint32_t address = 0x28U) {
        DeviceIdentity identity;
        identity.model = Model::kL6;
        identity.side = status == DiscoveryStatus::kVerified ? HandSide::kLeft : HandSide::kUnknown;
        identity.address = address;
        identity.timestamp = std::chrono::steady_clock::now();
        return DiscoveryCandidate{
            selected_channel,
            TransportKind::kCan,
            DiscoveryAddress{"classic_can_id", address, address, false},
            status,
            std::move(identity),
            DiscoveryTransportSpec{selected_channel.backend(),
                                   selected_channel.name(),
                                   TransportKind::kCan,
                                   fake_factory(),
                                   selected_channel.nominal_bitrate(),
                                   {},
                                   selected_channel.native_name()},
            status == DiscoveryStatus::kVerified
                ? std::optional<DeviceCapabilities>{declared_capabilities(Model::kL6, HandSide::kLeft)}
                : std::nullopt,
            status == DiscoveryStatus::kPartial
                ? std::vector<DiscoveryProblem>{DiscoveryProblem{"missing_side", "device side is unavailable"}}
                : std::vector<DiscoveryProblem>{}};
    }

    [[nodiscard]] DiscoveryCandidate l6_candidate_with_attached_identity(std::string attached_product_code,
                                                                         int *created = nullptr,
                                                                         bool empty_discovery_commands = false) {
        DeviceIdentity identity;
        identity.model = Model::kL6;
        identity.side = HandSide::kLeft;
        identity.address = 0x28U;
        identity.product_code = "LHL6-1-123-L";
        identity.serial_number = "123";
        identity.timestamp = std::chrono::steady_clock::now();

        auto factory = [code = std::move(attached_product_code), created]() {
            if (created) {
                ++*created;
            }
            auto next = std::make_unique<FakeTransport>();
            next->on_request(0x28U, [code](const Message &request) {
                std::vector<Message> replies;
                if (request.data[0] == 0xC0) {
                    for (uint8_t chunk = 0; chunk < 4; ++chunk) {
                        Message reply{};
                        reply.id = request.id;
                        reply.dlc = 8;
                        reply.data[0] = 0xC0;
                        reply.data[1] = chunk;
                        for (size_t index = 0; index < 6; ++index) {
                            const auto offset = static_cast<size_t>(chunk) * 6 + index;
                            reply.data[index + 2] =
                                offset < code.size() ? static_cast<uint8_t>(code[offset]) : uint8_t{0};
                        }
                        replies.push_back(reply);
                    }
                    return replies;
                }

                Message reply{};
                reply.id = request.id;
                reply.data[0] = request.data[0];
                if (request.data[0] == 0xC1 || request.data[0] == 0xC2 || request.data[0] == 0xC4) {
                    reply.dlc = request.data[0] == 0xC1 ? 5 : 4;
                    reply.data[1] = 1;
                    reply.data[2] = 2;
                    reply.data[3] = 3;
                } else {
                    reply.dlc = 7;
                    for (size_t index = 1; index < 7; ++index) {
                        reply.data[index] = 64;
                    }
                }
                replies.push_back(reply);
                return replies;
            });
            return std::unique_ptr<linkerbot::can::CANTransport>(std::move(next));
        };

        auto capabilities = declared_capabilities(Model::kL6, HandSide::kLeft);
        if (empty_discovery_commands) {
            capabilities.commands.clear();
        }
        return DiscoveryCandidate{
            channel(),
            TransportKind::kCan,
            DiscoveryAddress{"classic_can_id", 0x28U, 0x28U, false},
            DiscoveryStatus::kVerified,
            std::move(identity),
            DiscoveryTransportSpec{"fake", "can0", TransportKind::kCan, std::move(factory), 1'000'000U},
            std::move(capabilities)};
    }

    class FakeDiscoverySource final : public DiscoverySource {
    public:
        explicit FakeDiscoverySource(DiscoveryReport report) : report_(std::move(report)) {}

        DiscoveryReport scan(const DiscoveryConfig &config) override {
            ++calls;
            last_backends = config.backends;
            last_channels = config.channels;
            return report_;
        }

        int calls = 0;
        std::vector<std::string> last_backends;
        std::vector<std::string> last_channels;

    private:
        DiscoveryReport report_;
    };

    class ThrowingDiscoverySource final : public DiscoverySource {
    public:
        DiscoveryReport scan(const DiscoveryConfig &) override { throw std::runtime_error("enumeration failed"); }
    };

    class DelayedDiscoverySource final : public DiscoverySource {
    public:
        DelayedDiscoverySource(DiscoveryReport report, std::chrono::milliseconds delay)
            : report_(std::move(report)), delay_(delay) {}

        DiscoveryReport scan(const DiscoveryConfig &config) override {
            ++calls;
            received_timeout = config.total_timeout;
            std::this_thread::sleep_for(delay_);
            return report_;
        }

        int calls = 0;
        std::chrono::milliseconds received_timeout{0};

    private:
        DiscoveryReport report_;
        std::chrono::milliseconds delay_;
    };
} // namespace

TEST_CASE("discovery contract: helper ownership preserves both channel names", "[discovery][contract]") {
    const std::string name = "stable-channel-with-a-name-longer-than-small-string-storage";
    int created = 0;
    const auto verified = verified_candidate(name, &created);
    const auto partial = partial_candidate(name, &created);
    for (const auto *candidate : {&verified, &partial}) {
        REQUIRE(candidate->channel().name() == name);
        REQUIRE(candidate->channel().native_name() == name);
        REQUIRE(candidate->transport().channel() == name);
        REQUIRE(candidate->transport().native_channel() == name);
        REQUIRE(candidate->transport().factory()() != nullptr);
    }
    REQUIRE(created == 2);
}

TEST_CASE("discovery contract: default scan bounds are explicit", "[discovery][contract]") {
    const DiscoveryConfig config;
    REQUIRE(config.backends.empty());
    REQUIRE(config.channels.empty());
    REQUIRE(config.response_timeout == std::chrono::milliseconds{120});
    REQUIRE(config.send_timeout == std::chrono::milliseconds{100});
    REQUIRE(config.total_timeout == std::chrono::seconds{30});
    REQUIRE(config.route_queue_capacity == 256);
    REQUIRE(config.l30_node_ids.size() == 31);
    REQUIRE(config.l30_node_ids.front() == 1);
    REQUIRE(config.l30_node_ids.back() == 31);
    REQUIRE(config.o30_request_ids == std::vector<uint32_t>{0x001U, 0x002U});
    REQUIRE(config.pcan.probe_protocols == std::vector<TransportKind>{TransportKind::kCan});
    REQUIRE(config.slcan.probe_protocols == std::vector<TransportKind>{TransportKind::kCan});
    REQUIRE(config.minicanfd.channels.empty());
}

TEST_CASE("discovery contract: channel and failure expose backend-native details", "[discovery][contract]") {
    const DiscoveryChannel selected{"pcan", "pcan:usb-1", "PCAN_USBBUS1", TransportKind::kCan, 1'000'000U};
    REQUIRE(selected.provider() == "pcan");
    REQUIRE(selected.backend() == "pcan");
    REQUIRE(selected.name() == "pcan:usb-1");
    REQUIRE(selected.native_name() == "PCAN_USBBUS1");

    const DiscoveryChannel compatible{"socketcan", "can0", TransportKind::kCan};
    REQUIRE(compatible.native_name() == "can0");

    const DiscoveryFailure failure{"enumerate",    "driver_error",   "driver rejected the request",
                                   "pcan:usb-1",   "pcan",           "enumerate_channel",
                                   "PCAN_USBBUS1", "PCANBasicError", 7};
    REQUIRE(failure.backend() == "pcan");
    REQUIRE(failure.stage() == "enumerate_channel");
    REQUIRE(failure.native_channel() == "PCAN_USBBUS1");
    REQUIRE(failure.error_type() == "PCANBasicError");
    REQUIRE(failure.native_status() == 7);
}

TEST_CASE("discovery contract: injected source returns a report", "[discovery][contract]") {
    std::vector<DiscoveryChannel> channels;
    channels.push_back(channel());
    std::vector<DiscoveryCandidate> candidates;
    candidates.push_back(verified_candidate());
    FakeDiscoverySource source{DiscoveryReport{std::move(channels),
                                               std::move(candidates),
                                               {DiscoveryFailure{"enumerate", "busy", "can1 is busy", "can1"}},
                                               std::chrono::milliseconds{7}}};

    DiscoveryConfig config;
    config.channels = {"can0"};
    const auto report = discover_hands(config, source);

    REQUIRE(source.calls == 1);
    REQUIRE(source.last_channels == std::vector<std::string>{"can0"});
    REQUIRE(report.channels().size() == 1);
    REQUIRE(report.candidates().size() == 1);
    REQUIRE(report.failures().size() == 1);
    REQUIRE(report.elapsed() == std::chrono::milliseconds{7});
    REQUIRE(report.verified_candidates().size() == 1);
    REQUIRE(report.partial_candidates().empty());
    REQUIRE(report.require_one().model() == Model::kL6);
}

TEST_CASE("discovery contract: require_one reports cardinality and completeness", "[discovery][contract]") {
    REQUIRE_THROWS_AS(DiscoveryReport{}.require_one(), NoHandFoundError);

    std::vector<DiscoveryChannel> many_channels;
    many_channels.push_back(channel("can0"));
    many_channels.push_back(channel("can1"));
    std::vector<DiscoveryCandidate> many_candidates;
    many_candidates.push_back(verified_candidate("can0"));
    many_candidates.push_back(verified_candidate("can1"));
    DiscoveryReport many{std::move(many_channels), std::move(many_candidates)};
    REQUIRE_THROWS_AS(many.require_one(), AmbiguousHandError);

    std::vector<DiscoveryChannel> partial_channels;
    partial_channels.push_back(channel());
    std::vector<DiscoveryCandidate> partial_candidates;
    partial_candidates.push_back(partial_candidate());
    DiscoveryReport partial{std::move(partial_channels), std::move(partial_candidates)};
    REQUIRE(partial.partial_candidates().size() == 1);
    REQUIRE_THROWS_AS(partial.require_one(), IncompleteIdentityError);
    REQUIRE(partial.require_one(true).status() == DiscoveryStatus::kPartial);
}

TEST_CASE("discovery contract: candidate creates a closed HandDevice without IO", "[discovery][contract]") {
    int transports_created = 0;
    const auto candidate = verified_candidate("can7", &transports_created);

    const auto config = candidate.make_config();
    REQUIRE(config.model == Model::kL6);
    REQUIRE(config.side == HandSide::kLeft);
    REQUIRE(config.interface_name == "can7");
    REQUIRE(config.address == 0x28U);

    auto hand = candidate.create_hand_device();
    REQUIRE(hand != nullptr);
    REQUIRE_FALSE(hand->is_open());
    REQUIRE(transports_created == 0);
}

TEST_CASE("discovery contract: runtime policy is preserved without endpoint override",
          "[discovery][contract][config]") {
    int transports_created = 0;
    const auto candidate = verified_candidate("can7", &transports_created);
    HandRuntimeConfig runtime;
    runtime.polling_interval = std::chrono::milliseconds{17};
    runtime.angle_mapping_path = "mapping.json";
    runtime.command_timeout = std::chrono::milliseconds{211};
    runtime.command_rate_hz = 123.5;
    runtime.discrete_queue_capacity = 7;
    runtime.identity_timeout = std::chrono::milliseconds{433};
    runtime.transaction_timeout = std::chrono::milliseconds{89};
    runtime.shutdown_timeout = std::chrono::milliseconds{987};
    runtime.recovery_attempts = 4;
    runtime.recovery_backoff = std::chrono::milliseconds{23};

    const auto config = candidate.make_config(runtime);
    REQUIRE(config.model == Model::kL6);
    REQUIRE(config.side == HandSide::kLeft);
    REQUIRE(config.interface_name == "can7");
    REQUIRE(config.address == 0x28U);
    REQUIRE(config.polling_interval == runtime.polling_interval);
    REQUIRE(config.angle_mapping_path == runtime.angle_mapping_path);
    REQUIRE(config.command_timeout == runtime.command_timeout);
    REQUIRE(config.command_rate_hz == runtime.command_rate_hz);
    REQUIRE(config.discrete_queue_capacity == runtime.discrete_queue_capacity);
    REQUIRE(config.identity_timeout == runtime.identity_timeout);
    REQUIRE(config.transaction_timeout == runtime.transaction_timeout);
    REQUIRE(config.shutdown_timeout == runtime.shutdown_timeout);
    REQUIRE(config.recovery_attempts == runtime.recovery_attempts);
    REQUIRE(config.recovery_backoff == runtime.recovery_backoff);

    auto hand = candidate.create_hand_device(runtime);
    REQUIRE(hand != nullptr);
    REQUIRE_FALSE(hand->is_open());
    REQUIRE(transports_created == 0);

    DeviceIdentity l30_identity;
    l30_identity.model = Model::kL30;
    l30_identity.side = HandSide::kLeft;
    l30_identity.address = 7U;
    l30_identity.product_code = "LHT30-06-123-L-B-3-A";
    l30_identity.serial_number = "123";
    l30_identity.timestamp = std::chrono::steady_clock::now();
    const DiscoveryCandidate l30_candidate{
        DiscoveryChannel{"fake", "canfd7", TransportKind::kCanFd, 1'000'000U, 5'000'000U, false, true},
        TransportKind::kCanFd,
        DiscoveryAddress{"l30_node_id", 7U, 7U, true, uint8_t{7}, uint8_t{3}},
        DiscoveryStatus::kVerified,
        std::move(l30_identity),
        DiscoveryTransportSpec{"fake", "canfd7", TransportKind::kCanFd, fake_factory(&transports_created), 1'000'000U,
                               5'000'000U},
        declared_capabilities(Model::kL30, HandSide::kLeft)};
    HandRuntimeConfig l30_runtime;
    l30_runtime.auto_start_periodic_reports = true;
    const auto l30_config = l30_candidate.make_config(l30_runtime);
    REQUIRE(l30_config.auto_start_periodic_reports);
    REQUIRE(l30_config.address == 7U);
    REQUIRE(l30_config.host_address == 3U);
    REQUIRE(transports_created == 0);
}

TEST_CASE("discovery contract: candidate pins the discovered physical identity", "[discovery][contract][identity]") {
    SECTION("same physical device opens") {
        int transports_created = 0;
        const auto candidate = l6_candidate_with_attached_identity("LHL6-1-123-L", &transports_created, true);
        REQUIRE(candidate.capabilities()->commands.empty());
        auto hand = candidate.create_hand_device();
        REQUIRE(transports_created == 0);
        REQUIRE_NOTHROW(hand->open());
        REQUIRE(transports_created == 1);
        REQUIRE(hand->is_open());
        REQUIRE(hand->get_verified_capabilities().has_value());
        REQUIRE_FALSE(hand->get_capabilities().commands.empty());
    }

    SECTION("same model and side with another serial is rejected") {
        auto hand = l6_candidate_with_attached_identity("LHL6-1-999-L").create_hand_device();
        REQUIRE_THROWS_AS(hand->open(), IdentityMismatchError);
        REQUIRE_FALSE(hand->is_open());
    }
}

TEST_CASE("discovery contract: only missing-side partial candidates can be completed",
          "[discovery][contract][partial]") {
    int transports_created = 0;
    const auto candidate = partial_candidate("can0", &transports_created);
    REQUIRE_THROWS_AS(candidate.make_config(), IncompleteIdentityError);
    REQUIRE_THROWS_AS(candidate.create_hand_device(), IncompleteIdentityError);

    HandRuntimeConfig runtime;
    runtime.identity_timeout = std::chrono::milliseconds{321};
    auto hand = candidate.create_hand_device(runtime, HandSide::kLeft);
    REQUIRE(hand != nullptr);
    REQUIRE_FALSE(hand->is_open());
    REQUIRE(transports_created == 0);

    // The classic endpoint 0x28 is left-hand evidence, so an explicit right
    // side is rejected by endpoint validation before transport creation.
    REQUIRE_THROWS_AS(candidate.create_hand_device(runtime, HandSide::kRight), ValidationError);
    REQUIRE_THROWS_AS(candidate.create_hand_device(runtime, HandSide::kUnknown), ValidationError);
    REQUIRE(transports_created == 0);

    const auto collision = partial_candidate("can0", &transports_created, "address_collision");
    REQUIRE_THROWS_AS(collision.create_hand_device(runtime, HandSide::kLeft), IncompleteIdentityError);
    REQUIRE(transports_created == 0);

    const auto verified = verified_candidate("can0", &transports_created);
    auto verified_hand = verified.create_hand_device(runtime, HandSide::kLeft);
    REQUIRE(verified_hand != nullptr);
    REQUIRE_THROWS_AS(verified.create_hand_device(runtime, HandSide::kRight), ValidationError);
    REQUIRE(transports_created == 0);

    DeviceIdentity unknown_identity;
    unknown_identity.side = HandSide::kUnknown;
    const DiscoveryCandidate unknown_model{channel(),
                                           TransportKind::kCan,
                                           DiscoveryAddress{"classic_can_id", 0x28U, 0x28U, false},
                                           DiscoveryStatus::kPartial,
                                           std::move(unknown_identity),
                                           transport("can0", &transports_created),
                                           {},
                                           {DiscoveryProblem{"missing_side", "model is unavailable"}}};
    REQUIRE_THROWS_AS(unknown_model.create_hand_device(runtime, HandSide::kLeft), IncompleteIdentityError);

    DeviceIdentity l30_identity;
    l30_identity.model = Model::kL30;
    l30_identity.side = HandSide::kUnknown;
    const DiscoveryCandidate missing_node{
        DiscoveryChannel{"fake", "canfd0", TransportKind::kCanFd, 1'000'000U, 5'000'000U, false, true},
        TransportKind::kCanFd,
        DiscoveryAddress{"l30_node_id", 0U, 0U, true},
        DiscoveryStatus::kPartial,
        std::move(l30_identity),
        DiscoveryTransportSpec{"fake", "canfd0", TransportKind::kCanFd, fake_factory(&transports_created), 1'000'000U,
                               5'000'000U},
        {},
        {DiscoveryProblem{"missing_side", "side is unavailable"}}};
    REQUIRE_THROWS_AS(missing_node.create_hand_device(runtime, HandSide::kLeft), IncompleteIdentityError);
    REQUIRE(transports_created == 0);
}

TEST_CASE("discovery contract: values reject invalid invariants", "[discovery][contract]") {
    REQUIRE_THROWS_AS((DiscoveryChannel{"", "can0", TransportKind::kCan}), ValidationError);
    REQUIRE_THROWS_AS((DiscoveryChannel{"fake", "can0", TransportKind::kCan, {}, 5'000'000U}), ValidationError);
    REQUIRE_THROWS_AS((DiscoveryAddress{"classic", 0x800U, 0x800U, false}), ValidationError);
    REQUIRE_THROWS_AS((DiscoveryProblem{"", "missing"}), ValidationError);
    REQUIRE_THROWS_AS((DiscoveryTransportSpec{"fake", "can0", TransportKind::kCan, {}}), ValidationError);

    DeviceIdentity incomplete;
    incomplete.model = Model::kL6;
    incomplete.side = HandSide::kUnknown;
    REQUIRE_THROWS_AS((DiscoveryCandidate{channel(), TransportKind::kCan,
                                          DiscoveryAddress{"classic", 0x28U, 0x28U, false}, DiscoveryStatus::kVerified,
                                          incomplete, transport(), declared_capabilities(Model::kL6, HandSide::kLeft)}),
                      ValidationError);

    REQUIRE_THROWS_AS((DiscoveryReport{{channel()}, {verified_candidate("can1")}}), ValidationError);
}

TEST_CASE("discovery contract: invalid config is rejected before source invocation", "[discovery][contract]") {
    FakeDiscoverySource source{DiscoveryReport{}};
    DiscoveryConfig config;

    SECTION("empty channel") { config.channels = {""}; }
    SECTION("duplicate channel") { config.channels = {"can0", "can0"}; }
    SECTION("duplicate backend") { config.backends = {"socketcan", "socketcan"}; }
    SECTION("non-positive timeout") { config.response_timeout = std::chrono::milliseconds{0}; }
    SECTION("invalid L30 node") { config.l30_node_ids = {0}; }
    SECTION("duplicate O30 endpoint") { config.o30_request_ids = {1, 1}; }
    SECTION("invalid backend-specific bitrate") { config.pcan.nominal_bitrate = 0; }
    SECTION("duplicate MiniCANFD channel") { config.minicanfd.channels = {{0, 1}, {0, 1}}; }

    REQUIRE_THROWS_AS(discover_hands(config, source), ValidationError);
    REQUIRE(source.calls == 0);
}

TEST_CASE("composite discovery retains results and isolates backend failures", "[discovery][contract][composite]") {
    const DiscoveryChannel first_channel{"socketcan", "socketcan:can0", "can0", TransportKind::kCan, 1'000'000U};
    const DiscoveryChannel second_channel{"pcan", "pcan:usb1", "PCAN_USBBUS1", TransportKind::kCan, 1'000'000U};
    auto first =
        std::make_shared<FakeDiscoverySource>(DiscoveryReport{{first_channel}, {candidate_for(first_channel)}});
    auto second = std::make_shared<FakeDiscoverySource>(DiscoveryReport{
        {second_channel},
        {candidate_for(second_channel, DiscoveryStatus::kPartial, 0x27U)},
        {DiscoveryFailure{"probe_identity", "timeout", "one endpoint timed out", second_channel.name()}}});
    auto throwing = std::make_shared<ThrowingDiscoverySource>();
    CompositeDiscoverySource source{{{"socketcan", first}, {"pcan", second}, {"broken", throwing}}};

    const auto report = source.scan(DiscoveryConfig{});
    REQUIRE(first->calls == 1);
    REQUIRE(second->calls == 1);
    REQUIRE(report.channels().size() == 2);
    REQUIRE(report.candidates().size() == 2);
    REQUIRE(report.verified_candidates().size() == 1);
    REQUIRE(report.partial_candidates().size() == 1);
    REQUIRE(report.failures().size() == 2);
    REQUIRE(report.failures()[0].backend() == "broken");
    REQUIRE(report.failures()[0].code() == "backend_failed");
    REQUIRE(report.failures()[1].backend() == "pcan");
}

TEST_CASE("composite discovery applies backend and channel filters globally", "[discovery][contract][composite]") {
    const DiscoveryChannel socket_channel{"socketcan", "socketcan:can0", "can0", TransportKind::kCan, 1'000'000U};
    const DiscoveryChannel pcan_channel{"pcan", "pcan:usb1", "PCAN_USBBUS1", TransportKind::kCan, 1'000'000U};
    auto socket =
        std::make_shared<FakeDiscoverySource>(DiscoveryReport{{socket_channel}, {candidate_for(socket_channel)}});
    auto pcan = std::make_shared<FakeDiscoverySource>(DiscoveryReport{{pcan_channel}, {candidate_for(pcan_channel)}});
    CompositeDiscoverySource source{{{"socketcan", socket}, {"pcan", pcan}}};

    DiscoveryConfig config;
    config.backends = {"pcan"};
    config.channels = {"PCAN_USBBUS1"};
    const auto report = source.scan(config);
    REQUIRE(socket->calls == 0);
    REQUIRE(pcan->calls == 1);
    REQUIRE(pcan->last_backends.empty());
    REQUIRE(pcan->last_channels == std::vector<std::string>{"PCAN_USBBUS1"});
    REQUIRE(report.channels().size() == 1);
    REQUIRE(report.channels().front().name() == "pcan:usb1");
    REQUIRE(report.failures().empty());

    config.backends = {"missing"};
    config.channels.clear();
    const auto missing_backend = source.scan(config);
    REQUIRE(missing_backend.channels().empty());
    REQUIRE(missing_backend.failures().size() == 1);
    REQUIRE(missing_backend.failures().front().code() == "backend_not_found");

    config.backends.clear();
    config.channels = {"does-not-exist"};
    const auto missing_channel = source.scan(config);
    REQUIRE(missing_channel.channels().empty());
    REQUIRE(missing_channel.candidates().empty());
    REQUIRE(missing_channel.failures().size() == 1);
    REQUIRE(missing_channel.failures().front().code() == "channel_not_found");
}

TEST_CASE("composite discovery deduplicates and rejects channel conflicts", "[discovery][contract][composite]") {
    const DiscoveryChannel original{"adapter", "adapter:0", "0", TransportKind::kCan, 1'000'000U};
    auto first = std::make_shared<FakeDiscoverySource>(DiscoveryReport{{original}, {candidate_for(original)}});
    auto duplicate = std::make_shared<FakeDiscoverySource>(DiscoveryReport{{original}, {candidate_for(original)}});
    CompositeDiscoverySource deduplicating{{{"adapter", first}, {"adapter", duplicate}}};

    const auto deduplicated = deduplicating.scan(DiscoveryConfig{});
    REQUIRE(deduplicated.channels().size() == 1);
    REQUIRE(deduplicated.candidates().size() == 1);
    REQUIRE(deduplicated.failures().empty());

    const DiscoveryChannel conflicting{"adapter", "adapter:0", "0", TransportKind::kCan, 500'000U};
    auto conflict = std::make_shared<FakeDiscoverySource>(DiscoveryReport{{conflicting}});
    CompositeDiscoverySource rejecting{{{"adapter", first}, {"adapter", conflict}}};
    const auto rejected = rejecting.scan(DiscoveryConfig{});
    REQUIRE(rejected.channels().empty());
    REQUIRE(rejected.candidates().empty());
    REQUIRE(rejected.failures().size() == 1);
    REQUIRE(rejected.failures().front().code() == "channel_descriptor_conflict");
}

TEST_CASE("composite discovery shares one absolute timeout budget", "[discovery][contract][composite][timeout]") {
    const DiscoveryChannel first_channel{"first", "first:0", "0", TransportKind::kCan, 1'000'000U};
    auto first = std::make_shared<DelayedDiscoverySource>(
        DiscoveryReport{{first_channel}, {candidate_for(first_channel)}}, std::chrono::milliseconds{25});
    auto second = std::make_shared<FakeDiscoverySource>(DiscoveryReport{});
    CompositeDiscoverySource source{{{"first", first}, {"second", second}}};

    DiscoveryConfig config;
    config.total_timeout = std::chrono::milliseconds{10};
    const auto report = source.scan(config);

    REQUIRE(first->calls == 1);
    REQUIRE(first->received_timeout <= config.total_timeout);
    REQUIRE(second->calls == 0);
    REQUIRE(report.channels().size() == 1);
    REQUIRE(report.candidates().size() == 1);
    const auto timeout = std::find_if(report.failures().begin(), report.failures().end(),
                                      [](const DiscoveryFailure &failure) { return failure.code() == "timeout"; });
    REQUIRE(timeout != report.failures().end());
    REQUIRE(timeout->backend() == std::optional<std::string>{"second"});
}

TEST_CASE("composite discovery returns channels candidates and failures in stable order",
          "[discovery][contract][composite]") {
    const DiscoveryChannel last{"adapter", "adapter:z", "z", TransportKind::kCan, 1'000'000U};
    const DiscoveryChannel first{"adapter", "adapter:a", "a", TransportKind::kCan, 1'000'000U};
    auto unordered = std::make_shared<FakeDiscoverySource>(DiscoveryReport{
        {last, first},
        {candidate_for(last), candidate_for(first)},
        {DiscoveryFailure{"probe", "z_error", "last failure", last.name(), last.backend(), "probe", last.native_name()},
         DiscoveryFailure{"probe", "a_error", "first failure", first.name(), first.backend(), "probe",
                          first.native_name()}}});
    CompositeDiscoverySource source{{{"adapter", unordered}}};

    const auto report = source.scan(DiscoveryConfig{});
    REQUIRE(report.channels().size() == 2);
    REQUIRE(report.channels()[0].native_name() == "a");
    REQUIRE(report.channels()[1].native_name() == "z");
    REQUIRE(report.candidates().size() == 2);
    REQUIRE(report.candidates()[0].channel().native_name() == "a");
    REQUIRE(report.candidates()[1].channel().native_name() == "z");
    REQUIRE(report.failures().size() == 2);
    REQUIRE(report.failures()[0].native_channel() == std::optional<std::string>{"a"});
    REQUIRE(report.failures()[1].native_channel() == std::optional<std::string>{"z"});
}

TEST_CASE("discovery source construction performs no scan or device IO", "[discovery][contract][composite]") {
    auto child = std::make_shared<FakeDiscoverySource>(DiscoveryReport{});
    REQUIRE_NOTHROW(CompositeDiscoverySource{{{"fake", child}}});
    REQUIRE(child->calls == 0);
    REQUIRE_NOTHROW(PlatformDiscoverySource{});
}

TEST_CASE("platform discovery reports unsupported hosts without construction failure",
          "[discovery][contract][platform]") {
#if !defined(__linux__) && !defined(_WIN32)
    PlatformDiscoverySource source;
    const auto report = source.scan(DiscoveryConfig{});
    REQUIRE(report.channels().empty());
    REQUIRE(report.candidates().empty());
    REQUIRE(report.failures().size() == 1);
    REQUIRE(report.failures().front().code() == "platform_backend_unavailable");
#else
    SUCCEED("default platform discovery backends are covered by platform builds");
#endif
}

TEST_CASE("discovery contract: source exceptions cross a stable boundary", "[discovery][contract]") {
    ThrowingDiscoverySource source;
    try {
        static_cast<void>(discover_hands(DiscoveryConfig{}, source));
        FAIL("expected discovery source failure");
    } catch (const DiscoveryError &error) {
        REQUIRE(error.code() == ErrorCode::kDiscovery);
        REQUIRE(error.context().operation == "discover_hands");
        REQUIRE(error.cause() != nullptr);
    }
}

TEST_CASE("discovery connection keys and descriptors include backend timing identity", "[discovery][adapter]") {
    DiscoveryChannel first{
        "pcan", "pcan:PCAN_USBBUS2",    "PCAN_USBBUS2", TransportKind::kCanFd, 1000000U, 5000000U, false,
        true,   "resolved.dll|timing=A"};
    DiscoveryChannel second{
        "pcan", "pcan:PCAN_USBBUS2",    "PCAN_USBBUS2", TransportKind::kCanFd, 1000000U, 5000000U, false,
        true,   "resolved.dll|timing=B"};
    const auto factory = []() {
        return std::unique_ptr<linkerbot::can::CANTransport>(std::make_unique<linkerbot::can::FakeTransport>());
    };
    DiscoveryTransportSpec a{"pcan",   first.name(), TransportKind::kCanFd, factory,
                             1000000U, 5000000U,     first.native_name(),   first.configuration_id()};
    DiscoveryTransportSpec b{"pcan",   second.name(), TransportKind::kCanFd, factory,
                             1000000U, 5000000U,      second.native_name(),  second.configuration_id()};
    CHECK_FALSE(a.connection_key() == b.connection_key());
    auto left = std::make_shared<FakeDiscoverySource>(DiscoveryReport{{first}});
    auto right = std::make_shared<FakeDiscoverySource>(DiscoveryReport{{second}});
    CompositeDiscoverySource source{{{"pcan", left}, {"pcan", right}}};
    const auto report = discover_hands(DiscoveryConfig{}, source);
    CHECK(report.channels().empty());
    REQUIRE(report.failures().size() == 1);
    CHECK(report.failures()[0].code() == "channel_descriptor_conflict");
}

TEST_CASE("discovery preserves Python routed receive capacity without opening a device", "[discovery][python-phase4]") {
    int created = 0;
    const auto candidate = verified_candidate("python-inbox", &created);
    HandRuntimeConfig runtime;
    runtime.runtime_behavior = RuntimeBehavior::kPython;
    runtime.route_queue_capacity = 17;
    const auto config = candidate.make_config(runtime);
    REQUIRE((config.route_queue_capacity == 17));
    auto hand = candidate.create_hand_device(runtime);
    REQUIRE_FALSE(hand->is_open());
    REQUIRE((created == 0));
    runtime.route_queue_capacity = 0;
    REQUIRE_THROWS_AS(candidate.make_config(runtime), ValidationError);
    REQUIRE((created == 0));
}
