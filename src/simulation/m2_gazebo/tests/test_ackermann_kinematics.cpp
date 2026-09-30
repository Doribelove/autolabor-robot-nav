#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <m2_gazebo/ackermann_kinematics.h>
#include <m2_gazebo/actuator_dynamics.h>
#include <m2_gazebo/trajectory_interpolation.h>
#include <m2_gazebo/v3_reactive_actor.h>

TEST(AckermannKinematics, ZeroAndLowSpeedAreFinite) {
  EXPECT_DOUBLE_EQ(0.0, m2_gazebo::twistToSteering(0.0, 1.0, 0.65, 0.4964));
  EXPECT_DOUBLE_EQ(0.0, m2_gazebo::twistToSteering(1e-5, 1.0, 0.65, 0.4964));
}

TEST(AckermannKinematics, SteeringSaturates) {
  EXPECT_NEAR(0.4964, m2_gazebo::twistToSteering(0.1, 10.0, 0.65, 0.4964), 1e-9);
  EXPECT_NEAR(-0.4964, m2_gazebo::twistToSteering(0.1, -10.0, 0.65, 0.4964), 1e-9);
}

TEST(AckermannKinematics, ReversePreservesRequestedYawSign) {
  const double steer = m2_gazebo::twistToSteering(-1.0, 0.4, 0.65, 0.4964);
  EXPECT_LT(steer, 0.0);
  const auto geometry = m2_gazebo::steeringGeometry(steer, -1.0, 0.65, 0.60, 0.4964);
  EXPECT_NEAR(0.4, geometry.yaw_rate, 1e-9);
}

TEST(AckermannKinematics, InnerWheelTurnsMore) {
  const auto left_turn = m2_gazebo::steeringGeometry(0.3, 1.0, 0.65, 0.60, 0.4964);
  EXPECT_GT(left_turn.left, left_turn.right);
  EXPECT_GT(left_turn.right, 0.0);
  const auto right_turn = m2_gazebo::steeringGeometry(-0.3, 1.0, 0.65, 0.60, 0.4964);
  EXPECT_LT(right_turn.right, right_turn.left);
  EXPECT_LT(right_turn.left, 0.0);
}

TEST(AckermannKinematics, RearElectronicDifferential) {
  double left = 0.0;
  double right = 0.0;
  m2_gazebo::rearWheelLinearVelocities(1.0, 0.5, 0.60, &left, &right);
  EXPECT_NEAR(0.85, left, 1e-12);
  EXPECT_NEAR(1.15, right, 1e-12);
}

TEST(ActuatorDynamics, AccelerationAndSteeringAreRateLimited) {
  m2_gazebo::ActuatorLimits limits;
  const auto next = m2_gazebo::advanceActuators(
      m2_gazebo::ActuatorState{}, 2.0, 0.4, false, false, 0.1, limits);
  EXPECT_GT(next.speed_mps, 0.0);
  EXPECT_LE(next.speed_mps, limits.max_acceleration_mps2 * 0.1 + 1e-12);
  EXPECT_GT(next.steering_rad, 0.0);
  EXPECT_LE(next.steering_rad, limits.max_steering_rate_radps * 0.1 + 1e-12);
}

TEST(ActuatorDynamics, BrakingDistanceIsNonZeroAndAnalytic) {
  EXPECT_NEAR(0.08 + 1.0 / 4.8,
              m2_gazebo::stoppingDistance(1.0, 0.08, 2.4), 1e-12);
  EXPECT_TRUE(std::isinf(m2_gazebo::stoppingDistance(1.0, 0.1, 0.0)));
}

TEST(ActuatorDynamics, ReverseCommandMustCrossZero) {
  m2_gazebo::ActuatorLimits limits;
  m2_gazebo::ActuatorState current;
  current.speed_mps = 0.5;
  const auto next = m2_gazebo::advanceActuators(
      current, -0.5, 0.0, false, false, 0.1, limits);
  EXPECT_GE(next.speed_mps, 0.0);
  EXPECT_LT(next.speed_mps, current.speed_mps);
}

TEST(ActuatorDynamics, BrakeAndEmergencyHaveOrderedAuthority) {
  m2_gazebo::ActuatorLimits limits;
  m2_gazebo::ActuatorState current;
  current.speed_mps = 1.0;
  const auto normal = m2_gazebo::advanceActuators(
      current, 0.0, 0.0, false, false, 0.1, limits);
  const auto brake = m2_gazebo::advanceActuators(
      current, 1.0, 0.0, true, false, 0.1, limits);
  const auto emergency = m2_gazebo::advanceActuators(
      current, 1.0, 0.0, false, true, 0.1, limits);
  EXPECT_GE(normal.speed_mps, brake.speed_mps);
  EXPECT_GE(brake.speed_mps, emergency.speed_mps);
}

TEST(ActuatorDynamics, DelayJitterIsDeterministicAndBounded) {
  const double first = m2_gazebo::deterministicSignedJitter(7, 42, 0.015);
  const double repeat = m2_gazebo::deterministicSignedJitter(7, 42, 0.015);
  EXPECT_DOUBLE_EQ(first, repeat);
  EXPECT_LE(std::abs(first), 0.015);
  EXPECT_NE(first, m2_gazebo::deterministicSignedJitter(8, 42, 0.015));
}

TEST(ActuatorDynamics, CandidateLimitsAreFailClosed) {
  m2_gazebo::ActuatorLimits valid;
  EXPECT_TRUE(m2_gazebo::validActuatorLimits(valid));
  valid.max_brake_deceleration_mps2 = 0.5;
  EXPECT_FALSE(m2_gazebo::validActuatorLimits(valid));
}

TEST(TrajectoryActor, RejectsInvalidAndInterpolatesYawAcrossPi) {
  std::vector<m2_gazebo::TrajectoryWaypoint> invalid{{0.0, 0.0, 0.0, 0.0}};
  EXPECT_FALSE(m2_gazebo::validTrajectory(invalid));
  const std::vector<m2_gazebo::TrajectoryWaypoint> points{
      {0.0, 0.0, -1.0, 3.0}, {2.0, 2.0, 1.0, -3.0}};
  ASSERT_TRUE(m2_gazebo::validTrajectory(points));
  const auto middle = m2_gazebo::interpolateTrajectory(points, 1.0, false);
  EXPECT_NEAR(1.0, middle.x_m, 1e-12);
  EXPECT_NEAR(0.0, middle.y_m, 1e-12);
  EXPECT_GT(std::abs(middle.yaw_rad), 3.0);
}

TEST(TrajectoryActor, LoopIsDeterministic) {
  const std::vector<m2_gazebo::TrajectoryWaypoint> points{
      {0.0, 0.0, 0.0, 0.0}, {2.0, 2.0, 0.0, 0.0}};
  const auto first = m2_gazebo::interpolateTrajectory(points, 0.5, true);
  const auto looped = m2_gazebo::interpolateTrajectory(points, 2.5, true);
  EXPECT_NEAR(first.x_m, looped.x_m, 1e-12);
  EXPECT_NEAR(first.y_m, looped.y_m, 1e-12);
}

TEST(V3ReactiveActor, PolicyParsingIsClosed) {
  m2_gazebo::V3ActorPolicy policy;
  EXPECT_TRUE(m2_gazebo::parseV3ActorPolicy("cautious", &policy));
  EXPECT_EQ(m2_gazebo::V3ActorPolicy::kCautious, policy);
  EXPECT_TRUE(m2_gazebo::parseV3ActorPolicy("assertive", &policy));
  EXPECT_EQ(m2_gazebo::V3ActorPolicy::kAssertive, policy);
  EXPECT_TRUE(m2_gazebo::parseV3ActorPolicy("hesitant", &policy));
  EXPECT_EQ(m2_gazebo::V3ActorPolicy::kHesitant, policy);
  EXPECT_FALSE(m2_gazebo::parseV3ActorPolicy("candidate_aware", &policy));
}

TEST(V3ReactiveActor, PoliciesRespondDifferentlyToObservedConflict) {
  m2_gazebo::V3ReactiveActorConfig cautious;
  cautious.policy = m2_gazebo::V3ActorPolicy::kCautious;
  m2_gazebo::V3ReactiveActorConfig assertive = cautious;
  assertive.policy = m2_gazebo::V3ActorPolicy::kAssertive;
  m2_gazebo::V3ReactiveActorConfig hesitant = cautious;
  hesitant.policy = m2_gazebo::V3ActorPolicy::kHesitant;
  m2_gazebo::V3ReactiveActorState initial;
  initial.speed_mps = 1.0;
  const auto cautious_next = m2_gazebo::advanceV3ReactiveActor(
      initial, cautious, 2.0, 2.0, 1.0, 1.0, 0.1);
  const auto assertive_next = m2_gazebo::advanceV3ReactiveActor(
      initial, assertive, 2.0, 2.0, 1.0, 1.0, 0.1);
  const auto hesitant_next = m2_gazebo::advanceV3ReactiveActor(
      initial, hesitant, 2.0, 2.0, 1.0, 1.0, 0.1);
  EXPECT_TRUE(cautious_next.responding);
  EXPECT_TRUE(assertive_next.responding);
  EXPECT_TRUE(hesitant_next.responding);
  EXPECT_LT(cautious_next.speed_mps, initial.speed_mps);
  EXPECT_GT(assertive_next.speed_mps, initial.speed_mps);
  EXPECT_NE(hesitant_next.speed_mps, assertive_next.speed_mps);
}

TEST(V3ReactiveActor, NoResponseWithoutObservedApproach) {
  m2_gazebo::V3ReactiveActorConfig config;
  m2_gazebo::V3ReactiveActorState initial;
  initial.speed_mps = config.base_speed_mps;
  const auto next = m2_gazebo::advanceV3ReactiveActor(
      initial, config, 2.0, 2.0, -0.1, 1.0, 0.1);
  EXPECT_FALSE(next.responding);
  EXPECT_FALSE(next.ego_approaching);
  EXPECT_NEAR(config.base_speed_mps, next.speed_mps, 1e-12);
}

TEST(V3ReactiveActor, SeedJitterIsDeterministicAndBounded) {
  const double first = m2_gazebo::v3SignedSeedJitter(6101U);
  EXPECT_DOUBLE_EQ(first, m2_gazebo::v3SignedSeedJitter(6101U));
  EXPECT_GE(first, -1.0);
  EXPECT_LE(first, 1.0);
  EXPECT_NE(first, m2_gazebo::v3SignedSeedJitter(6102U));
}

TEST(V3ReactiveActor, ResponseEnableGateIsClosedAndDeterministic) {
  EXPECT_TRUE(m2_gazebo::v3ResponseEnabled(0.0, 0.0));
  EXPECT_FALSE(m2_gazebo::v3ResponseEnabled(12.50, 12.55));
  EXPECT_FALSE(m2_gazebo::v3ResponseEnabled(12.549, 12.55));
  EXPECT_TRUE(m2_gazebo::v3ResponseEnabled(12.55, 12.55));
  EXPECT_FALSE(m2_gazebo::v3ResponseEnabled(
      std::numeric_limits<double>::quiet_NaN(), 12.55));
  EXPECT_FALSE(m2_gazebo::v3ResponseEnabled(12.55, -1.0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
