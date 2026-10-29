#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>

#include <linkerbot/hand/api/events.hpp>
#include <linkerbot/hand/l20lite/events.hpp>
#include <linkerbot/hand/o30/events.hpp>
#include <linkerbot/hand/o6/events.hpp>

#include "../../../src/hand/unified/detail/model_adapters.hpp"

using namespace linkerbot::hand;

TEST_CASE("L30 configured speed snapshots preserve command origin and freshness",
          "[comm][device][snapshot][configured-speed]") {
    const auto now = std::chrono::steady_clock::now();
    l30::Snapshot source{};
    source.timestamp = now;
    l30::SpeedData targets{};
    targets.speeds.fill(123);
    targets.timestamp = now - std::chrono::seconds{2};
    source.configured_speeds = targets;
    l30::SpeedData measured{};
    measured.speeds.fill(7);
    measured.timestamp = now;
    source.speed = measured;
    const auto snapshot =
        unified_detail::unified_snapshot(source, Model::kL30, HandSide::kUnknown, declared_capabilities(Model::kL30));
    REQUIRE(snapshot.configured_speeds->raw == std::vector<double>(17, 123));
    REQUIRE(snapshot.configured_speeds->meta.valid);
    REQUIRE(snapshot.configured_speeds->meta.stale);
    REQUIRE(snapshot.configured_speeds->meta.origin == TelemetryValueOrigin::kCommandCache);
    REQUIRE(snapshot.configured_speeds->meta.delivery == TelemetryDelivery::kCommandCache);
    REQUIRE(snapshot.measured_speeds->raw == std::vector<double>(17, 7));
    REQUIRE(snapshot.joints.front().configured_speed_raw == 123);
    source.configured_speeds->timestamp = now;
    REQUIRE_FALSE(
        unified_detail::unified_snapshot(source, Model::kL30, HandSide::kUnknown, declared_capabilities(Model::kL30))
            .configured_speeds->meta.stale);
}

TEST_CASE("L6 configured speed snapshots retain raw evidence sequence and cache freshness",
          "[comm][device][snapshot][l6-controls]") {
    const auto now = std::chrono::steady_clock::now();
    l6::Snapshot source{};
    source.timestamp = now;
    l6::RawControlData control{};
    control.values.fill(255);
    control.sequence = 17;
    control.command_response = true;
    control.timestamp = now - std::chrono::seconds{2};
    source.configured_speeds = control;
    const auto snapshot = unified_detail::unified_snapshot(source, Model::kL6, HandSide::kLeft,
                                                           declared_capabilities(Model::kL6, HandSide::kLeft));
    REQUIRE(snapshot.configured_speeds->raw == std::vector<double>(6, 255));
    REQUIRE(snapshot.configured_speeds->meta.sequence == 17);
    REQUIRE(snapshot.configured_speeds->meta.valid);
    REQUIRE(snapshot.configured_speeds->meta.stale);
    REQUIRE(snapshot.configured_speeds->meta.delivery == TelemetryDelivery::kCommandResponse);
    REQUIRE(snapshot.joints[0].configured_speed_raw == 255);
}

TEST_CASE("unified snapshot projects classic motion and stall caches", "[comm][device][snapshot]") {
    const auto now = std::chrono::steady_clock::now();
    o6::Snapshot source{};
    source.timestamp = now;

    std::array<float, o6::kJointCount> speed{};
    speed.fill(50.0F);
    source.speed = o6::SpeedData{o6::Speed::from_array(speed), now};

    std::array<float, o6::kJointCount> acceleration{};
    acceleration.fill(100.0F);
    source.acceleration = o6::AccelerationData{o6::Acceleration::from_array(acceleration), now};

    std::array<float, o6::kJointCount> stall_time{};
    stall_time.fill(100.0F);
    source.stall_time = o6::StallTimeData{o6::StallTime::from_array(stall_time), now};

    std::array<float, o6::kJointCount> stall_limit{};
    stall_limit.fill(1000.0F);
    source.stall_threshold = o6::StallLimitData{o6::StallLimit::from_array(stall_limit), now};

    const auto snapshot = unified_detail::unified_snapshot(source, Model::kO6, HandSide::kLeft,
                                                           declared_capabilities(Model::kO6, HandSide::kLeft));

    REQUIRE(snapshot.configured_speeds->raw.front() == 127.0);
    REQUIRE(snapshot.configured_accelerations->raw.front() == 0.0);
    REQUIRE(snapshot.stall_detection_times->raw.front() == 10.0);
    REQUIRE(snapshot.stall_thresholds->raw.front() == 255.0);
    REQUIRE(snapshot.joints.front().configured_speed_raw == 127.0);
    REQUIRE(snapshot.joints.front().configured_acceleration_raw == 0.0);
    REQUIRE(snapshot.joints.front().stall_detection_time_raw == 10.0);
}

TEST_CASE("unified snapshot exposes normal force resultants", "[comm][device][snapshot]") {
    const auto now = std::chrono::steady_clock::now();
    l20lite::Snapshot source{};
    source.timestamp = now;
    std::array<l20lite::TipFinger, 5> fingers{};
    for (size_t index = 0; index < fingers.size(); ++index) {
        fingers[index].normal = static_cast<uint8_t>(index + 1);
    }
    source.tip_force = l20lite::TipForceData{l20lite::TipForce::from_array(fingers), now};

    const auto snapshot = unified_detail::unified_snapshot(source, Model::kL20Lite, HandSide::kLeft,
                                                           declared_capabilities(Model::kL20Lite, HandSide::kLeft));

    REQUIRE(snapshot.normal_force_resultant.has_value());
    REQUIRE(snapshot.normal_force_resultant->raw == std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0});
    const HandEventPayload payload = *snapshot.normal_force_resultant;
    REQUIRE(std::holds_alternative<NormalForceResultantSample>(payload));
}

TEST_CASE("unified snapshot projects O30 voltage motion and stall channels", "[comm][device][snapshot]") {
    const auto now = std::chrono::steady_clock::now();
    o30::Snapshot source{};
    source.timestamp = now;

    o30::SpeedData speed{};
    speed.speeds.fill(11);
    speed.timestamp = now;
    source.speed = speed;

    o30::AccelerationData acceleration{};
    acceleration.accelerations.fill(12);
    acceleration.timestamp = now;
    source.acceleration = acceleration;

    o30::VoltageData voltage{};
    voltage.voltages.fill(13);
    voltage.timestamp = now;
    source.voltage = voltage;

    o30::StallTimeData stall_time{};
    stall_time.times.fill(15);
    stall_time.timestamp = now;
    source.stall_time = stall_time;

    o30::StallThresholdData threshold{};
    threshold.thresholds.fill(16);
    threshold.timestamp = now;
    source.stall_threshold = threshold;

    o30::StallHoldCurrentData hold{};
    hold.currents.fill(17);
    hold.timestamp = now;
    source.stall_hold_current = hold;

    const auto snapshot =
        unified_detail::unified_snapshot(source, Model::kO30, HandSide::kUnknown, declared_capabilities(Model::kO30));

    REQUIRE(snapshot.measured_speeds->raw.front() == 11.0);
    REQUIRE(snapshot.configured_accelerations->raw.front() == 12.0);
    REQUIRE(snapshot.voltages->raw.front() == 13.0);
    REQUIRE(snapshot.stall_detection_times->raw.front() == 15.0);
    REQUIRE(snapshot.stall_thresholds->raw.front() == 16.0);
    REQUIRE(snapshot.stall_hold_currents->raw.front() == 17.0);
    REQUIRE(snapshot.joints.front().voltage_raw == 13.0);
    REQUIRE(snapshot.joints.front().stall_hold_current_raw == 17.0);

    source.speed->kind = o30::ValueKind::kSetpoint;
    const auto configured =
        unified_detail::unified_snapshot(source, Model::kO30, HandSide::kUnknown, declared_capabilities(Model::kO30));
    REQUIRE_FALSE(configured.measured_speeds.has_value());
    REQUIRE(configured.configured_speeds->raw.front() == 11.0);
}

TEST_CASE("O30 tactile snapshot projects scalar normal force resultants", "[comm][device][snapshot][o30-force]") {
    const auto now = std::chrono::steady_clock::now();
    o30::Snapshot source{};
    source.timestamp = now;
    o30::SensorData sensors{};
    sensors.timestamp = now;
    for (uint8_t index = 1; index <= 5; ++index) {
        o30::SensorInfo info{};
        info.type = "TACTILE";
        info.unit = "N";
        info.selected = index;
        info.total_length = 1;
        info.descriptor = "FnS_U8_1";
        info.timestamp = now;
        sensors.sensors.push_back({info, {index}, now});
    }
    source.force_sensor = sensors;
    const auto snapshot = unified_detail::unified_snapshot(source, Model::kO30, HandSide::kRight,
                                                           declared_capabilities(Model::kO30, HandSide::kRight));
    REQUIRE(snapshot.force.has_value());
    REQUIRE(snapshot.normal_force_resultant.has_value());
    REQUIRE(snapshot.normal_force_resultant->raw == std::vector<double>{1, 2, 3, 4, 5});
}
