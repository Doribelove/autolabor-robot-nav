#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "autolabor_canbus_driver/m2_twist_steering.h"

namespace autolabor_driver {
namespace {

TEST(M2TwistSteering, TwistValidationRejectsNonFiniteCriticalFields) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    EXPECT_TRUE(m2_twist_command_is_valid(0.3, -0.2));
    EXPECT_FALSE(m2_twist_command_is_valid(nan, 0.2));
    EXPECT_FALSE(m2_twist_command_is_valid(inf, 0.2));
    EXPECT_FALSE(m2_twist_command_is_valid(0.3, nan));
    EXPECT_FALSE(m2_twist_command_is_valid(0.3, -inf));
}

TEST(M2TwistSteering, ChassisValidationRequiresFinitePositiveControlParameters) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    EXPECT_TRUE(m2_chassis_parameters_are_valid(1.6, 0.5, 0.8, 0.65, 0.16));

    EXPECT_FALSE(m2_chassis_parameters_are_valid(nan, 0.5, 0.8, 0.65, 0.16));
    EXPECT_FALSE(m2_chassis_parameters_are_valid(inf, 0.5, 0.8, 0.65, 0.16));
    EXPECT_FALSE(m2_chassis_parameters_are_valid(0.0, 0.5, 0.8, 0.65, 0.16));
    EXPECT_FALSE(m2_chassis_parameters_are_valid(-1.6, 0.5, 0.8, 0.65, 0.16));

    EXPECT_FALSE(m2_chassis_parameters_are_valid(1.6, nan, 0.8, 0.65, 0.16));
    EXPECT_FALSE(m2_chassis_parameters_are_valid(1.6, 0.0, 0.8, 0.65, 0.16));
    EXPECT_FALSE(m2_chassis_parameters_are_valid(1.6, -0.5, 0.8, 0.65, 0.16));

    EXPECT_FALSE(m2_chassis_parameters_are_valid(1.6, 0.5, nan, 0.65, 0.16));
    EXPECT_FALSE(m2_chassis_parameters_are_valid(1.6, 0.5, 0.8, inf, 0.16));
    EXPECT_FALSE(m2_chassis_parameters_are_valid(1.6, 0.5, 0.8, 0.0, 0.16));
    EXPECT_FALSE(m2_chassis_parameters_are_valid(1.6, 0.5, 0.8, -0.65, 0.16));
    EXPECT_FALSE(m2_chassis_parameters_are_valid(1.6, 0.5, 0.8, 0.65, nan));
}

TEST(M2TwistSteering, CanBoundarySanitizesAndLimitsMotionControl) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    const M2SafeMotionControl unchanged =
        sanitize_m2_motion_control(0.4, -0.2, 0.5);
    EXPECT_TRUE(unchanged.input_valid);
    EXPECT_FALSE(unchanged.was_limited);
    EXPECT_DOUBLE_EQ(unchanged.relative_velocity, 0.4);
    EXPECT_DOUBLE_EQ(unchanged.steering_angle, -0.2);

    const M2SafeMotionControl limited =
        sanitize_m2_motion_control(1.4, -0.8, 0.5);
    EXPECT_TRUE(limited.input_valid);
    EXPECT_TRUE(limited.was_limited);
    EXPECT_DOUBLE_EQ(limited.relative_velocity, 1.0);
    EXPECT_DOUBLE_EQ(limited.steering_angle, -0.5);

    for (const M2SafeMotionControl invalid : {
             sanitize_m2_motion_control(nan, 0.2, 0.5),
             sanitize_m2_motion_control(0.2, inf, 0.5),
             sanitize_m2_motion_control(0.2, 0.1, nan),
             sanitize_m2_motion_control(0.2, 0.1, 0.0),
             sanitize_m2_motion_control(0.2, 0.1, -0.5)}) {
        EXPECT_FALSE(invalid.input_valid);
        EXPECT_FALSE(invalid.was_limited);
        EXPECT_DOUBLE_EQ(invalid.relative_velocity, 0.0);
        EXPECT_DOUBLE_EQ(invalid.steering_angle, 0.0);
    }
}

TEST(M2TwistSteering, LinearSpeedLimitPreservesForwardCurvature) {
    const double requested_linear = 2.7;
    const double limited_linear = 1.63284;
    const double requested_angular = 0.81;
    const double limited_angular = angular_velocity_after_linear_limit(
        requested_linear, limited_linear, requested_angular);

    EXPECT_NEAR(limited_angular / limited_linear,
                requested_angular / requested_linear,
                1e-12);
    EXPECT_LT(std::abs(limited_angular), std::abs(requested_angular));
}

TEST(M2TwistSteering, LinearSpeedLimitPreservesReverseCurvature) {
    const double requested_linear = -2.7;
    const double limited_linear = -1.63284;
    const double requested_angular = 0.81;
    const double limited_angular = angular_velocity_after_linear_limit(
        requested_linear, limited_linear, requested_angular);

    EXPECT_NEAR(limited_angular / limited_linear,
                requested_angular / requested_linear,
                1e-12);
    EXPECT_GT(limited_angular, 0.0);
}

TEST(M2TwistSteering, AngularVelocityIsUnchangedWithoutLinearClipping) {
    EXPECT_DOUBLE_EQ(
        angular_velocity_after_linear_limit(1.2, 1.2, -0.4), -0.4);
    EXPECT_DOUBLE_EQ(
        angular_velocity_after_linear_limit(0.0, 0.0, 0.4), 0.4);
}

TEST(M2TwistSteering, LinearSpeedLimitRejectsNonFiniteInputs) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    EXPECT_DOUBLE_EQ(
        angular_velocity_after_linear_limit(nan, 1.0, 0.4), 0.0);
    EXPECT_DOUBLE_EQ(
        angular_velocity_after_linear_limit(2.0, inf, 0.4), 0.0);
    EXPECT_DOUBLE_EQ(
        angular_velocity_after_linear_limit(2.0, 1.0, -inf), 0.0);
    EXPECT_DOUBLE_EQ(
        angular_velocity_after_linear_limit(2.0, 1.0, 0.4, nan), 0.0);
    EXPECT_DOUBLE_EQ(
        angular_velocity_after_linear_limit(2.0, 1.0, 0.4, -1e-4), 0.0);
}

TEST(M2TwistSteering, SameYawRateUsesOppositeSteeringInReverse) {
    const double forward = twist_to_front_steering(1.2, 0.45, 0.65, 1.0);
    const double reverse = twist_to_front_steering(-1.2, 0.45, 0.65, 1.0);

    EXPECT_GT(forward, 0.0);
    EXPECT_LT(reverse, 0.0);
    EXPECT_NEAR(forward, -reverse, 1e-12);
}

TEST(M2TwistSteering, SteeringReconstructsRequestedYawRate) {
    const double wheelbase = 0.65;
    const double max_steering = 1.0;

    for (const double linear_velocity : {1.4, -1.4}) {
        const double requested_angular_velocity = 0.35;
        const double steering = twist_to_front_steering(
            linear_velocity,
            requested_angular_velocity,
            wheelbase,
            max_steering);
        const double reconstructed_angular_velocity =
            linear_velocity * std::tan(steering) / wheelbase;

        EXPECT_NEAR(reconstructed_angular_velocity,
                    requested_angular_velocity,
                    1e-12);
    }
}

TEST(M2TwistSteering, ClampsSteeringToChassisLimit) {
    const double max_steering = 0.42;

    EXPECT_DOUBLE_EQ(
        twist_to_front_steering(0.1, 10.0, 0.65, max_steering),
        max_steering);
    EXPECT_DOUBLE_EQ(
        twist_to_front_steering(-0.1, 10.0, 0.65, max_steering),
        -max_steering);
    EXPECT_DOUBLE_EQ(
        twist_to_front_steering(0.1, -10.0, 0.65, max_steering),
        -max_steering);
    EXPECT_DOUBLE_EQ(
        twist_to_front_steering(-0.1, -10.0, 0.65, max_steering),
        max_steering);
}

TEST(M2TwistSteering, NearZeroSpeedPreservesSteeringPrepositionBehavior) {
    const double max_steering = 0.6;

    EXPECT_DOUBLE_EQ(
        twist_to_front_steering(0.0, 0.02, 0.65, max_steering),
        max_steering);
    EXPECT_DOUBLE_EQ(
        twist_to_front_steering(-5e-5, -0.02, 0.65, max_steering),
        -max_steering);
    EXPECT_DOUBLE_EQ(
        twist_to_front_steering(5e-5, 0.005, 0.65, max_steering),
        0.0);
}

TEST(M2TwistSteering, InvalidSteeringInputsFailClosed) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    EXPECT_DOUBLE_EQ(twist_to_front_steering(nan, 0.2, 0.65, 0.6), 0.0);
    EXPECT_DOUBLE_EQ(twist_to_front_steering(0.5, inf, 0.65, 0.6), 0.0);
    EXPECT_DOUBLE_EQ(twist_to_front_steering(0.5, 0.2, inf, 0.6), 0.0);
    EXPECT_DOUBLE_EQ(twist_to_front_steering(0.5, 0.2, 0.65, nan), 0.0);
    EXPECT_DOUBLE_EQ(twist_to_front_steering(0.5, 0.2, 0.0, 0.6), 0.0);
    EXPECT_DOUBLE_EQ(twist_to_front_steering(0.5, 0.2, -0.65, 0.6), 0.0);
    EXPECT_DOUBLE_EQ(twist_to_front_steering(0.5, 0.2, 0.65, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(twist_to_front_steering(0.5, 0.2, 0.65, -0.6), 0.0);
    EXPECT_DOUBLE_EQ(
        twist_to_front_steering(0.5, 0.2, 0.65, 0.6, nan, 0.01),
        0.0);
    EXPECT_DOUBLE_EQ(
        twist_to_front_steering(0.5, 0.2, 0.65, 0.6, 1e-4, -0.01),
        0.0);
}

}  // namespace
}  // namespace autolabor_driver

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
