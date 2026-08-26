#include "autolabor_canbus_driver/m2_status_polling.h"

#include <gtest/gtest.h>

#include <limits>

TEST(M2StatusPolling, SoleOwnerCoversEveryVisualSafetyField) {
    const auto types = autolabor_driver::m2_safety_status_types();
    ASSERT_EQ(4u, types.size());
    EXPECT_EQ(0x17, static_cast<int>(types[0]));
    EXPECT_EQ(0x18, static_cast<int>(types[1]));
    EXPECT_EQ(0x19, static_cast<int>(types[2]));
    EXPECT_EQ(0x80, static_cast<int>(types[3]));
}

TEST(M2StatusPolling, MatchingReplyAdvancesButOtherTrafficCannot) {
    autolabor_driver::M2SafetyQueryRetryGate gate(4);
    gate.begin(0x17);
    gate.record_attempt();

    EXPECT_FALSE(gate.observe(0x18));
    EXPECT_FALSE(gate.should_advance());
    EXPECT_TRUE(gate.observe(0x17));
    EXPECT_TRUE(gate.should_advance());
    EXPECT_FALSE(gate.exhausted());
    EXPECT_EQ(1u, gate.attempts());
}

TEST(M2StatusPolling, MissingReplyHasBoundedRetriesAndThenAdvances) {
    autolabor_driver::M2SafetyQueryRetryGate gate(3);
    gate.begin(0x19);
    gate.record_attempt();
    gate.record_attempt();
    EXPECT_FALSE(gate.should_advance());

    gate.record_attempt();
    EXPECT_TRUE(gate.should_advance());
    EXPECT_TRUE(gate.exhausted());
    EXPECT_EQ(3u, gate.attempts());

    gate.reset();
    EXPECT_FALSE(gate.active());
    EXPECT_TRUE(gate.set_max_attempts(4));
    EXPECT_FALSE(gate.set_max_attempts(0));
}

TEST(M2StatusPolling, CapsAttemptRateBelowTheReplyPathLimit) {
    EXPECT_DOUBLE_EQ(
        3.0, autolabor_driver::m2_status_query_rate_hz(1.0, 5, 3.0));
    EXPECT_DOUBLE_EQ(
        1.5, autolabor_driver::m2_status_query_rate_hz(0.3, 5, 3.0));
    EXPECT_DOUBLE_EQ(
        3.0, autolabor_driver::m2_status_query_rate_hz(2.0, 5, 3.0));
}

TEST(M2StatusPolling, QueryIntervalsDesynchronizeWithinABoundedBand) {
    autolabor_driver::M2QueryIntervalSchedule schedule;
    ASSERT_TRUE(schedule.configure(3.0, 0.20));

    double total = 0.0;
    double previous = 0.0;
    bool changed = false;
    constexpr int sample_count = 10000;
    for (int index = 0; index < sample_count; ++index) {
        const double interval = schedule.next_interval_sec();
        EXPECT_GE(interval, 0.8 / 3.0);
        EXPECT_LE(interval, 1.2 / 3.0);
        if (index > 0 && std::abs(interval - previous) > 1e-9) {
            changed = true;
        }
        previous = interval;
        total += interval;
    }
    EXPECT_TRUE(changed);
    EXPECT_NEAR(1.0 / 3.0, total / sample_count, 1e-5);
}

TEST(M2StatusPolling, InvalidJitterScheduleFailsClosed) {
    autolabor_driver::M2QueryIntervalSchedule schedule;
    EXPECT_DOUBLE_EQ(0.0, schedule.next_interval_sec());
    EXPECT_FALSE(schedule.configure(0.0, 0.20));
    EXPECT_FALSE(schedule.configure(3.0, -0.01));
    EXPECT_FALSE(schedule.configure(3.0, 0.46));
    EXPECT_FALSE(schedule.configure(
        std::numeric_limits<double>::quiet_NaN(), 0.20));
    EXPECT_TRUE(schedule.configure(3.0, 0.0));
    EXPECT_DOUBLE_EQ(1.0 / 3.0, schedule.next_interval_sec());
}

TEST(M2StatusPolling, InvalidConfigurationFailsClosed) {
    EXPECT_DOUBLE_EQ(
        0.0, autolabor_driver::m2_status_query_rate_hz(0.0, 5, 3.0));
    EXPECT_DOUBLE_EQ(
        0.0, autolabor_driver::m2_status_query_rate_hz(-1.0, 5, 3.0));
    EXPECT_DOUBLE_EQ(
        0.0, autolabor_driver::m2_status_query_rate_hz(1.0, 0, 3.0));
    EXPECT_DOUBLE_EQ(
        0.0, autolabor_driver::m2_status_query_rate_hz(1.0, 5, 0.0));
    EXPECT_DOUBLE_EQ(
        0.0,
        autolabor_driver::m2_status_query_rate_hz(
            std::numeric_limits<double>::infinity(), 5, 3.0));
    EXPECT_DOUBLE_EQ(
        0.0,
        autolabor_driver::m2_status_query_rate_hz(
            std::numeric_limits<double>::quiet_NaN(), 5, 3.0));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
