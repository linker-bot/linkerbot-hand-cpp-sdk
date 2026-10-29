#include "hop_fixture.hpp"
#include <catch2/catch_test_macros.hpp>
#include <future>
#include <linkerbot/hand/api/device.hpp>
#include <linkerbot/hand/protocol/tactile.hpp>
#include <thread>
using namespace linkerbot::hand;
using namespace std::chrono_literals;
using l6_hop_test::config;
using l6_hop_test::Firmware;

TEST_CASE("L6 HOP explicit capabilities preserve classic default", "[comm][l6-hop]") {
    auto cfg = config(0x123);
    const auto profile = declared_capabilities(cfg);
    REQUIRE(profile.validate());
    REQUIRE(profile.transport.kind == TransportKind::kCanFd);
    REQUIRE(profile.transport.request_id == 0x123);
    REQUIRE(profile.transport.response_id == 0x523);
    REQUIRE(profile.joints[0].raw_angle_range.maximum == 65535);
    REQUIRE(profile.command(CommandSemantic::kAngles)->byte_order == ByteOrder::kLittleEndian);
    REQUIRE(profile.force_field("fn").supported == std::nullopt);
    REQUIRE(declared_capabilities(Model::kL6).joints[0].raw_angle_range.maximum == 255);
    REQUIRE_FALSE(profile.supports(DeviceOperation::kClearFaults));
    REQUIRE_FALSE(profile.supports(CommandSemantic::kStallThresholds));
    cfg.model = Model::kO6;
    REQUIRE_THROWS_AS(HandDevice(cfg), ValidationError);
    cfg = config();
    cfg.address = 0x3ff;
    REQUIRE_THROWS_AS(HandDevice(cfg), ValidationError);
    cfg = config();
    cfg.angle_mapping_path = "old.json";
    REQUIRE_THROWS_AS(HandDevice(cfg), ValidationError);
    cfg = config();
    cfg.side = HandSide::kUnknown;
    REQUIRE_THROWS_AS(HandDevice(cfg), ValidationError);
}
TEST_CASE("L6 HOP unified open reads u16 values and preserves identity version", "[comm][l6-hop]") {
    auto firmware = std::make_shared<Firmware>();
    auto cfg = config(0x123);
    HandDevice device(cfg, [firmware] { return firmware->transport(0x123); });
    REQUIRE(device.get_capabilities().transport.kind == TransportKind::kCanFd);
    device.open();
    REQUIRE(device.get_identity()->model == Model::kL6);
    REQUIRE(device.get_identity()->protocol_version == "1.0.0");
    REQUIRE(device.get_identity()->serial_number == "632");
    REQUIRE(device.get_capabilities().joints[0].raw_angle_range.maximum == 65535);
    REQUIRE(device.get_angles().raw == std::vector<double>{256, 32768, 65535, 0, 1, 513});
    REQUIRE(device.get_angles().percent[2] == 100.0);
    REQUIRE(device.get_faults().statuses[0].known_bits == 0x80);
    REQUIRE(device.get_faults().statuses[0].reserved_bits == 0x30);
    REQUIRE(device.get_currents().raw[0] == 1);
    REQUIRE(device.get_current_limits().raw[5] == 6);
    REQUIRE(device.get_configured_speeds().meta.origin == TelemetryValueOrigin::kDeviceConfigured);
    REQUIRE(device.get_temperatures().raw[0] == 1);
    REQUIRE_THROWS_AS(device.set_stall_thresholds({1, 1, 1, 1, 1, 1}), UnsupportedOperationError);
    REQUIRE_THROWS_AS(device.clear_faults(), UnsupportedOperationError);
    const auto snapshot = device.get_snapshot();
    REQUIRE(snapshot.joints.size() == 6);
    REQUIRE(snapshot.joints[0].angle_raw == 256);
    const auto before_sequence = device.get_angles().meta.sequence;
    auto set = device.set_angles({65535, 256, 513, 32768, 1, 0});
    REQUIRE(set.wait_for(1s));
    REQUIRE(set.stage() == CommandStage::kAcknowledged);
    REQUIRE(set.sent_frames() == 2);
    REQUIRE(device.get_snapshot().angles->meta.sequence > before_sequence);
    REQUIRE(device.get_angles().raw == std::vector<double>{65535, 256, 513, 32768, 1, 0});
    auto percent = device.set_percent_angles({0, 100, 50, 25, 75, 0});
    REQUIRE(percent.wait_for(1s));
    REQUIRE(percent.stage() == CommandStage::kAcknowledged);
    REQUIRE(device.get_angles().raw == std::vector<double>{0, 65535, 32768, 16384, 49151, 0});
    auto speed = device.set_speeds({1, 2, 3, 4, 5, 255});
    REQUIRE(speed.wait_for(1s));
    REQUIRE(speed.stage() == CommandStage::kAcknowledged);
    auto current = device.set_current_limits({255, 2, 3, 4, 5, 6});
    REQUIRE(current.wait_for(1s));
    REQUIRE(current.stage() == CommandStage::kAcknowledged);
    REQUIRE(device.get_current_limits().raw[0] == 255);
    auto saved = device.save_parameters();
    REQUIRE(saved.wait_for(1s));
    REQUIRE(saved.stage() == CommandStage::kAcknowledged);
    device.close();
    REQUIRE(device.get_capabilities().joints[0].raw_angle_range.maximum == 65535);
}
TEST_CASE("L6 HOP angle command waits for both mounted replies", "[comm][l6-hop]") {
    auto firmware = std::make_shared<Firmware>();
    auto transport = firmware->transport();
    auto *fake = transport.get();
    auto session = linkerbot::transport::Session::own(std::move(transport));
    HandDevice device(config(), session);
    device.open();
    const auto before = device.get_snapshot().angles;
    {
        std::lock_guard<std::mutex> lock(firmware->mutex);
        firmware->hold_tail = true;
    }
    auto handle = device.set_angles({5, 6, 7, 8, 9, 10});
    REQUIRE_FALSE(handle.wait_for(20ms));
    REQUIRE(handle.stage() == CommandStage::kDispatched);
    REQUIRE(device.get_snapshot().angles->raw == before->raw);
    std::optional<linkerbot::can::Message> held;
    {
        std::lock_guard<std::mutex> lock(firmware->mutex);
        held = firmware->held;
    }
    REQUIRE(held);
    fake->inject(*held);
    REQUIRE(handle.wait_for(1s));
    REQUIRE(handle.stage() == CommandStage::kAcknowledged);
    REQUIRE(device.get_snapshot().angles->raw == std::vector<double>{5, 6, 7, 8, 9, 10});
    device.close();
    session->close();
}
TEST_CASE("L6 HOP failed partial response preserves last complete sample", "[comm][l6-hop]") {
    auto firmware = std::make_shared<Firmware>();
    HandDevice device(config(), [firmware] { return firmware->transport(); });
    device.open();
    const auto before = device.get_snapshot().angles->raw;
    {
        std::lock_guard<std::mutex> lock(firmware->mutex);
        firmware->short_tail = true;
    }
    auto handle = device.set_angles({1, 2, 3, 4, 5, 6});
    REQUIRE(handle.wait_for(1s));
    REQUIRE(handle.stage() == CommandStage::kFailed);
    REQUIRE(handle.error());
    REQUIRE(device.get_snapshot().angles->raw == before);
    device.close();
}
TEST_CASE("L6 HOP identity rejects wrong model side and protocol", "[comm][l6-hop]") {
    for (int scenario = 0; scenario < 3; ++scenario) {
        auto firmware = std::make_shared<Firmware>();
        if (scenario == 0) {
            Firmware::put(firmware->info, 0, "O30", 16);
        }
        if (scenario == 1) {
            Firmware::put(firmware->info, 0xf1, "LEFT", 8);
        }
        if (scenario == 2) {
            Firmware::put(firmware->info, 0x59, "OTHER", 8);
        }
        HandDevice device(config(), [firmware] { return firmware->transport(); });
        REQUIRE_THROWS(device.open());
        REQUIRE_FALSE(device.is_open());
        device.close();
    }
}
TEST_CASE("L6 HOP descriptor touch supports aliases reserved data and all banks", "[comm][l6-hop]") {
    auto firmware = std::make_shared<Firmware>();
    auto transport = firmware->transport();
    auto *fake = transport.get();
    l6::HopL6 hand(l6::Side::kRight, std::move(transport));
    const auto sample = hand.force_sensor.get_blocking(2s);
    REQUIRE(sample.meta.valid);
    REQUIRE(sample.sensors.size() == 5);
    REQUIRE(sample.sensors[0].rows == 2);
    REQUIRE(sample.sensors[0].columns == 2);
    REQUIRE_FALSE(sample.sensors[2].available);
    REQUIRE(sample.field("fns")->raw[0]->at(0) == 42);
    REQUIRE(sample.field("fns")->raw[1]->at(0) == 9);
    REQUIRE(sample.field("fts")->raw[1]->at(0) == -2);
    REQUIRE(sample.field("fds")->raw[1]->at(0) == 45);
    REQUIRE(sample.field("fn")->raw[0] == std::vector<double>{1, 2, 3, 4});
    REQUIRE(sample.field("fn")->raw[4]->size() == 260);
    REQUIRE(sample.sensors[3].fields[0].values == std::vector<double>{1, 2, 3});
    const auto sent = fake->sent();
    REQUIRE(std::any_of(sent.begin(), sent.end(), [](const auto &req) { return req.data[0] == 0x34; }));
    for (const auto &req : sent) {
        REQUIRE(req.data[2] <= 61);
    }
    const auto old = hand.force_sensor.get_snapshot()->raw;
    {
        std::lock_guard<std::mutex> lock(firmware->mutex);
        firmware->bad_length = true;
    }
    REQUIRE_THROWS(hand.force_sensor.get_blocking(200ms));
    REQUIRE(hand.force_sensor.get_snapshot()->raw == old);
    hand.stop();
}
TEST_CASE("L6 HOP unified dynamic force capabilities and polling stream", "[comm][l6-hop]") {
    auto firmware = std::make_shared<Firmware>();
    HandDevice device(config(), [firmware] { return firmware->transport(); });
    device.open();
    const auto force = device.get_force({"fn", "fns", "fts", "fds"}, 2s);
    REQUIRE(force.field("fn"));
    REQUIRE(force.field("fns"));
    REQUIRE(force.field("fts"));
    REQUIRE(device.get_capabilities().force_field("fns").supported == true);
    auto stream = device.stream_handle();
    device.start_polling({{TelemetrySemantic::kAngles, 10ms}, {TelemetrySemantic::kConfiguredSpeeds, 20ms}});
    const auto previous = device.get_snapshot();
    const auto next = device.wait_for_snapshot(previous, 500ms);
    REQUIRE(next.angles->meta.valid);
    device.stop_polling();
    device.stop_stream();
    device.close();
}
TEST_CASE("L6 HOP close cancels commands and borrowed session survives", "[comm][l6-hop]") {
    auto firmware = std::make_shared<Firmware>();
    auto transport = firmware->transport();
    auto *fake = transport.get();
    auto session = linkerbot::transport::Session::own(std::move(transport));
    HandDevice device(config(), session);
    device.open();
    {
        std::lock_guard<std::mutex> lock(firmware->mutex);
        firmware->hold_tail = true;
    }
    auto handle = device.set_angles({1, 2, 3, 4, 5, 6});
    (void)handle.wait_for(10ms);
    device.close();
    REQUIRE(handle.done());
    REQUIRE(handle.stage() == CommandStage::kFailed);
    REQUIRE(fake->close_calls() == 0);
    session->close();
}
TEST_CASE("L6 HOP recover verifies identity and never replays targets", "[comm][l6-hop]") {
    auto firmware = std::make_shared<Firmware>();
    HandDevice device(config(), [firmware] { return firmware->transport(); });
    device.open();
    auto handle = device.set_angles({7, 8, 9, 10, 11, 12});
    REQUIRE(handle.wait_for(1s));
    device.start_polling({{TelemetrySemantic::kAngles, 20ms}});
    device.recover();
    REQUIRE(device.is_open());
    REQUIRE(device.get_identity()->device_uid.has_value());
    REQUIRE(device.get_capabilities().joints[0].raw_angle_range.maximum == 65535);
    REQUIRE(device.get_angles().raw == std::vector<double>{7, 8, 9, 10, 11, 12});
    {
        std::lock_guard<std::mutex> lock(firmware->mutex);
        REQUIRE(firmware->position_writes == 2);
    }
    device.stop_polling();
    device.close();
}
TEST_CASE("L6 HOP tactile malformed and nonfinite payloads fail", "[comm][l6-hop]") {
    for (int scenario = 0; scenario < 4; ++scenario) {
        auto firmware = std::make_shared<Firmware>();
        if (scenario == 0) {
            firmware->bad_selected = true;
        }
        if (scenario == 1) {
            firmware->invalid_mask = true;
        }
        if (scenario == 2) {
            firmware->sensors[0].descriptor = "Fn_BAD_4";
        }
        if (scenario == 3) {
            firmware->sensors[0] = {"Fn_F32_1", {0, 0, 128, 127}, 1, 1};
        }
        l6::HopL6 hand(l6::Side::kRight, firmware->transport());
        REQUIRE_THROWS_AS(hand.force_sensor.get_blocking(200ms), ProtocolError);
        REQUIRE_FALSE(hand.force_sensor.get_snapshot());
    }
}

TEST_CASE("L6 HOP timed out replies cannot complete a later request", "[comm][l6-hop]") {
    auto firmware = std::make_shared<Firmware>();
    auto transport = firmware->transport();
    auto *fake = transport.get();
    auto cfg = config();
    cfg.command_timeout = 40ms;
    auto session = linkerbot::transport::Session::own(std::move(transport));
    HandDevice device(cfg, session);
    device.open();
    const auto previous = device.get_snapshot().angles->raw;
    {
        std::lock_guard<std::mutex> lock(firmware->mutex);
        firmware->hold_tail = true;
    }
    const auto handle = device.set_angles({1, 2, 3, 4, 5, 6});
    REQUIRE(handle.wait_for(1s));
    REQUIRE(handle.stage() == CommandStage::kFailed);
    REQUIRE(handle.error());
    std::optional<linkerbot::can::Message> late;
    {
        std::lock_guard<std::mutex> lock(firmware->mutex);
        late = firmware->held;
    }
    REQUIRE(late);
    fake->inject(*late);
    REQUIRE_THROWS(device.get_angles(50ms));
    REQUIRE(device.get_snapshot().angles->raw == previous);
    device.close();
    session->close();
}

TEST_CASE("L6 HOP shares a bus without routing one hand to another", "[comm][l6-hop]") {
    auto right = std::make_shared<Firmware>();
    auto left = std::make_shared<Firmware>();
    Firmware::put(left->info, 0xf1, "LEFT", 8);
    Firmware::put(left->info, 0x39, "LHL6-03.3-633-L-B-1-D", 32);
    auto transport = right->transport(1);
    auto *fake = transport.get();
    fake->on_request(2, [left](const auto &request) { return left->handle(request); });
    auto session = linkerbot::transport::Session::own(std::move(transport));
    l6::HopL6 a(l6::Side::kRight, session, 1);
    l6::HopL6 b(l6::Side::kLeft, session, 2);
    a.angle.set_angles({1, 2, 3, 4, 5, 6});
    b.angle.set_angles({1000, 2000, 3000, 4000, 5000, 6000});
    REQUIRE(a.angle.get_blocking().raw == std::vector<double>{1, 2, 3, 4, 5, 6});
    REQUIRE(b.angle.get_blocking().raw == std::vector<double>{1000, 2000, 3000, 4000, 5000, 6000});
    a.stop();
    REQUIRE(fake->close_calls() == 0);
    REQUIRE(b.version.get_blocking().side == HandSide::kLeft);
    REQUIRE(b.angle.get_blocking().raw[0] == 1000);
    b.stop();
    session->close();
}

TEST_CASE("L6 HOP sensor composites serialize selection and suppress partial events", "[comm][l6-hop]") {
    auto firmware = std::make_shared<Firmware>();
    auto transport = firmware->transport();
    auto *fake = transport.get();
    l6::HopL6 hand(l6::Side::kRight, std::move(transport));
    auto stream = hand.stream_handle();
    auto first = std::async(std::launch::async, [&] { return hand.force_sensor.get_blocking(2s); });
    auto second = std::async(std::launch::async, [&] { return hand.force_sensor.get_blocking(2s); });
    REQUIRE(first.get().sensors.size() == 5);
    REQUIRE(second.get().sensors.size() == 5);
    const auto sent = fake->sent();
    std::vector<uint8_t> selections;
    for (const auto &req : sent) {
        if (req.data[0] == 0x31 && req.data[1] == 0x6e && req.dlc > 3) {
            selections.push_back(req.data[3]);
        }
    }
    REQUIRE(selections == std::vector<uint8_t>{1, 2, 3, 4, 5, 1, 2, 3, 4, 5});
    hand.stop_stream();
    size_t events = 0;
    while (const auto event = stream->pop()) {
        REQUIRE(event->semantic == TelemetrySemantic::kForce);
        REQUIRE(std::get<ForceSample>(event->sample).sensors.size() == 5);
        ++events;
    }
    REQUIRE(events == 2);
}

TEST_CASE("L6 HOP no sensors UTF8 and aggregate deadline behavior", "[comm][l6-hop]") {
    SECTION("No sensor data never reads the data banks") {
        auto firmware = std::make_shared<Firmware>();
        firmware->sensors = {};
        auto transport = firmware->transport();
        auto *fake = transport.get();
        l6::HopL6 hand(l6::Side::kRight, std::move(transport));
        REQUIRE_THROWS_AS(hand.force_sensor.get_blocking(1s), UnsupportedOperationError);
        const auto sent = fake->sent();
        REQUIRE(std::none_of(sent.begin(), sent.end(),
                             [](const auto &req) { return req.data[0] >= 0x32 && req.data[0] <= 0x34; }));
        REQUIRE_FALSE(hand.force_sensor.get_snapshot());
    }
    SECTION("Valid UTF8 product info is preserved") {
        auto firmware = std::make_shared<Firmware>();
        Firmware::put(firmware->info, 0x89, "控制板V1", 16);
        l6::HopL6 hand(l6::Side::kRight, firmware->transport());
        REQUIRE(hand.version.get_blocking().hardware_version == "控制板V1");
    }
    SECTION("Malformed UTF8 is rejected") {
        auto firmware = std::make_shared<Firmware>();
        Firmware::put(firmware->info, 0x89, std::string("\xc0\x80"), 16);
        l6::HopL6 hand(l6::Side::kRight, firmware->transport());
        REQUIRE_THROWS_AS(hand.version.get_blocking(), ProtocolError);
    }
    SECTION("All metadata requests share the same total deadline") {
        auto firmware = std::make_shared<Firmware>();
        auto transport = firmware->transport();
        auto *fake = transport.get();
        fake->on_request(1, [firmware](const auto &request) {
            std::this_thread::sleep_for(8ms);
            return firmware->handle(request);
        });
        l6::HopL6 hand(l6::Side::kRight, std::move(transport));
        REQUIRE_THROWS_AS(hand.version.get_blocking(20ms), TimeoutError);
        REQUIRE(fake->sent().size() < 5);
    }
}
