#include "autolabor_canbus_driver/m2_status_polling.h"

#include <gtest/gtest.h>

#include <limits>

TEST(M2StatusPolling, SpreadsOneSnapshotAcrossAllFields) {
    EXPECT_DOUBLE_EQ(
        4.0, autolabor_driver::m2_status_query_rate_hz(1.0, 4, 4.0));
    EXPECT_DOUBLE_EQ(
        2.0, autolabor_driver::m2_status_query_rate_hz(0.5, 4, 4.0));
    EXPECT_DOUBLE_EQ(
        4.0, autolabor_driver::m2_status_query_rate_hz(2.0, 4, 4.0));
}

TEST(M2StatusPolling, InvalidConfigurationFailsClosed) {
    EXPECT_DOUBLE_EQ(
        0.0, autolabor_driver::m2_status_query_rate_hz(0.0, 4, 4.0));
    EXPECT_DOUBLE_EQ(
        0.0, autolabor_driver::m2_status_query_rate_hz(-1.0, 4, 4.0));
    EXPECT_DOUBLE_EQ(
        0.0, autolabor_driver::m2_status_query_rate_hz(1.0, 0, 4.0));
    EXPECT_DOUBLE_EQ(
        0.0, autolabor_driver::m2_status_query_rate_hz(1.0, 4, 0.0));
    EXPECT_DOUBLE_EQ(
        0.0,
        autolabor_driver::m2_status_query_rate_hz(
            std::numeric_limits<double>::infinity(), 4, 4.0));
    EXPECT_DOUBLE_EQ(
        0.0,
        autolabor_driver::m2_status_query_rate_hz(
            std::numeric_limits<double>::quiet_NaN(), 4, 4.0));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
