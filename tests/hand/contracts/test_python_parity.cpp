#include "hand/support/optional_value.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <limits>
#include <utility>
#include <vector>

#include "../../../src/hand/unified/detail/freshness.hpp"
#include <linkerbot/hand/capabilities.hpp>

using namespace linkerbot::hand;

// Golden contract values exported from Python Hand SDK b6c927a. These cover
// retained common model channels. Added controls are checked against the pinned
// Python migration vectors; the C++-only L20 current-limit cache has its own policy.
TEST_CASE("common models preserve Python telemetry freshness contracts", "[comm][contracts][parity][freshness]") {
    using Entry = std::pair<TelemetrySemantic, double>;
    const std::vector<std::pair<Model, std::vector<Entry>>> expected{
        {Model::kL6,
         {
             {TelemetrySemantic::kTorques, 3000},
             {TelemetrySemantic::kAngles, 1000.0 / 30.0},
             {TelemetrySemantic::kConfiguredSpeeds, 1000},
             {TelemetrySemantic::kCurrentLimits, 100},
             {TelemetrySemantic::kForce, 500},
             {TelemetrySemantic::kTemperatures, 3000},
             {TelemetrySemantic::kCurrents, 300},
             {TelemetrySemantic::kFaults, 500},
             {TelemetrySemantic::kStallDetectionTimes, 3000},
             {TelemetrySemantic::kStallThresholds, 3000},
             {TelemetrySemantic::kStallHoldCurrents, 3000},
         }},
        {Model::kO6,
         {
             {TelemetrySemantic::kAngles, 1000.0 / 30.0},
             {TelemetrySemantic::kConfiguredSpeeds, 100},
             {TelemetrySemantic::kConfiguredAccelerations, 300},
             {TelemetrySemantic::kCurrents, 100},
             {TelemetrySemantic::kCurrentLimits, 3000},
             {TelemetrySemantic::kForce, 100},
             {TelemetrySemantic::kTemperatures, 3000},
             {TelemetrySemantic::kFaults, 500},
             {TelemetrySemantic::kStallDetectionTimes, 3000},
             {TelemetrySemantic::kStallThresholds, 3000},
             {TelemetrySemantic::kStallHoldCurrents, 3000},
         }},
        {Model::kL20Lite,
         {
             {TelemetrySemantic::kStallThresholds, 3000},
             {TelemetrySemantic::kStallDetectionTimes, 3000},
             {TelemetrySemantic::kStallHoldCurrents, 3000},
             {TelemetrySemantic::kConfiguredSpeeds, 3000},
             {TelemetrySemantic::kCurrentLimits, 3000},
             {TelemetrySemantic::kConfiguredAccelerations, 3000},
             {TelemetrySemantic::kAngles, 1000.0 / 30.0},
             {TelemetrySemantic::kMeasuredSpeeds, 100},
             {TelemetrySemantic::kCurrents, 100},
             {TelemetrySemantic::kForce, 100},
             {TelemetrySemantic::kForceFns, 100},
             {TelemetrySemantic::kForceFts, 100},
             {TelemetrySemantic::kForceFds, 100},
             {TelemetrySemantic::kForcePx, 100},
             {TelemetrySemantic::kTemperatures, 3000},
             {TelemetrySemantic::kFaults, 500},
         }},
        {Model::kL20,
         {
             {TelemetrySemantic::kAngles, 1000.0 / 30.0},
             {TelemetrySemantic::kMeasuredSpeeds, 100},
             {TelemetrySemantic::kCurrents, 100},
             {TelemetrySemantic::kForce, 100},
             {TelemetrySemantic::kTemperatures, 3000},
             {TelemetrySemantic::kFaults, 500},
         }},
        {Model::kL30,
         {
             {TelemetrySemantic::kAngles, 1000.0 / 30.0},
             {TelemetrySemantic::kConfiguredSpeeds, 1000},
             {TelemetrySemantic::kMeasuredSpeeds, 100},
             {TelemetrySemantic::kCurrents, 300},
             {TelemetrySemantic::kCurrentLimits, 1000},
             {TelemetrySemantic::kForce, 100},
             {TelemetrySemantic::kTemperatures, 3000},
             {TelemetrySemantic::kFaults, 500},
         }},
        {Model::kO30,
         {
             {TelemetrySemantic::kAngles, 1000.0 / 15.0},
             {TelemetrySemantic::kMeasuredSpeeds, 100},
             {TelemetrySemantic::kConfiguredSpeeds, 100},
             {TelemetrySemantic::kConfiguredAccelerations, 300},
             {TelemetrySemantic::kTemperatures, 3000},
             {TelemetrySemantic::kForce, 300},
             {TelemetrySemantic::kCurrents, 300},
             {TelemetrySemantic::kCurrentLimits, 100},
             {TelemetrySemantic::kTorques, 300},
             {TelemetrySemantic::kFaults, 500},
             {TelemetrySemantic::kVoltages, 3000},
             {TelemetrySemantic::kStallDetectionTimes, 3000},
             {TelemetrySemantic::kStallThresholds, 3000},
             {TelemetrySemantic::kStallHoldCurrents, 3000},
         }},
    };
    for (const auto &[model, channels] : expected) {
        const auto &profile = declared_capabilities(model, HandSide::kLeft);
        for (const auto &[semantic, threshold] : channels) {
            CAPTURE(model, semantic);
            const auto *channel = profile.telemetry_channel(semantic);
            REQUIRE(channel != nullptr);
            REQUIRE(channel->freshness_ms.has_value());
            REQUIRE(test_support::require_value(channel->freshness_ms) == Catch::Approx(threshold));
        }
    }
}

TEST_CASE("common models preserve Python command ACK contracts", "[comm][contracts][parity][ack]") {
    using Entry = std::pair<CommandSemantic, AckSupport>;
    const std::vector<std::pair<Model, std::vector<Entry>>> expected{
        {Model::kL6,
         {
             {CommandSemantic::kAngles, AckSupport::kUnsupported},
             {CommandSemantic::kSpeeds, AckSupport::kRequired},
             {CommandSemantic::kCurrentLimits, AckSupport::kRequired},
             {CommandSemantic::kStallDetectionTimes, AckSupport::kRequired},
             {CommandSemantic::kStallThresholds, AckSupport::kRequired},
             {CommandSemantic::kStallHoldCurrents, AckSupport::kRequired},
             {CommandSemantic::kPositionKp, AckSupport::kRequired},
             {CommandSemantic::kPositionKd, AckSupport::kRequired},
             {CommandSemantic::kTorqueKd, AckSupport::kRequired},
             {CommandSemantic::kTorques, AckSupport::kRequired},
             {CommandSemantic::kLimitCompensations, AckSupport::kRequired},
         }},
        {Model::kO6,
         {
             {CommandSemantic::kAngles, AckSupport::kUnsupported},
             {CommandSemantic::kSpeeds, AckSupport::kRequired},
             {CommandSemantic::kAccelerations, AckSupport::kRequired},
             {CommandSemantic::kCurrentLimits, AckSupport::kRequired},
             {CommandSemantic::kStallDetectionTimes, AckSupport::kRequired},
             {CommandSemantic::kStallThresholds, AckSupport::kRequired},
             {CommandSemantic::kStallHoldCurrents, AckSupport::kRequired},
         }},
        {Model::kL20Lite,
         {
             {CommandSemantic::kStallThresholds, AckSupport::kRequired},
             {CommandSemantic::kStallDetectionTimes, AckSupport::kRequired},
             {CommandSemantic::kStallHoldCurrents, AckSupport::kRequired},
             {CommandSemantic::kAngles, AckSupport::kUnsupported},
             {CommandSemantic::kSpeeds, AckSupport::kRequired},
             {CommandSemantic::kAccelerations, AckSupport::kRequired},
             {CommandSemantic::kCurrentLimits, AckSupport::kRequired},
         }},
        {Model::kL20,
         {
             {CommandSemantic::kAngles, AckSupport::kUnsupported},
             {CommandSemantic::kSpeeds, AckSupport::kUnsupported},
             {CommandSemantic::kCurrentLimits, AckSupport::kUnsupported},
         }},
        {Model::kL30,
         {
             {CommandSemantic::kAngles, AckSupport::kUnsupported},
             {CommandSemantic::kSpeeds, AckSupport::kUnsupported},
             {CommandSemantic::kCurrentLimits, AckSupport::kUnsupported},
         }},
        {Model::kO30,
         {
             {CommandSemantic::kAngles, AckSupport::kRequired},
             {CommandSemantic::kSpeeds, AckSupport::kRequired},
             {CommandSemantic::kAccelerations, AckSupport::kRequired},
             {CommandSemantic::kCurrentLimits, AckSupport::kRequired},
             {CommandSemantic::kStallDetectionTimes, AckSupport::kRequired},
             {CommandSemantic::kStallThresholds, AckSupport::kRequired},
             {CommandSemantic::kStallHoldCurrents, AckSupport::kRequired},
         }},
    };
    for (const auto &[model, commands] : expected) {
        const auto &profile = declared_capabilities(model, HandSide::kLeft);
        REQUIRE(profile.commands.size() == commands.size());
        for (const auto &[semantic, ack] : commands) {
            CAPTURE(model, semantic);
            const auto *command = profile.command(semantic);
            REQUIRE(command != nullptr);
            REQUIRE(command->ack_support == ack);
        }
    }
}

TEST_CASE("fractional freshness thresholds retain sub-millisecond boundaries", "[comm][contracts][parity][freshness]") {
    using namespace std::chrono;
    const auto now = steady_clock::now();
    const auto *channel =
        declared_capabilities(Model::kL6, HandSide::kLeft).telemetry_channel(TelemetrySemantic::kAngles);
    SampleMeta meta{};
    meta.valid = true;
    meta.collected_at = now - microseconds{33333};
    unified_detail::complete_freshness(meta, channel, now);
    REQUIRE_FALSE(meta.stale);
    meta.collected_at = now - microseconds{33334};
    unified_detail::complete_freshness(meta, channel, now);
    REQUIRE(meta.stale);

    const auto *current = declared_capabilities(Model::kL30).telemetry_channel(TelemetrySemantic::kCurrents);
    meta.collected_at = now - milliseconds{300};
    unified_detail::complete_freshness(meta, current, now);
    REQUIRE_FALSE(meta.stale);
    unified_detail::complete_freshness(meta, current, now + microseconds{1});
    REQUIRE(meta.stale);
    meta.valid = false;
    meta.collected_at = now;
    unified_detail::complete_freshness(meta, current, now);
    REQUIRE(meta.stale);
}

TEST_CASE("freshness declarations reject non-finite or non-positive thresholds",
          "[comm][contracts][parity][freshness]") {
    for (const auto invalid :
         {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        auto profile = declared_capabilities(Model::kL6, HandSide::kLeft);
        profile.telemetry.front().freshness_ms = invalid;
        REQUIRE_FALSE(profile.validate());
    }
    const auto &profile = declared_capabilities(Model::kL6, HandSide::kLeft);
    REQUIRE(profile.validate());
    REQUIRE(profile.serialize().find("33.333333333333") != std::string::npos);
}
