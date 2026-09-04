#include <gtest/gtest.h>

#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_via_point.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <teb_local_planner/timed_elastic_band.h>

namespace
{

class TestableTebOptimalPlanner : public teb_local_planner::TebOptimalPlanner
{
public:
  using teb_local_planner::TebOptimalPlanner::TebOptimalPlanner;
  using teb_local_planner::TebOptimalPlanner::AddEdgesKinematicsCarlike;
  using teb_local_planner::TebOptimalPlanner::AddEdgesViaPoints;
  using teb_local_planner::TebOptimalPlanner::AddTEBVertices;
};

class TestableTebLocalPlannerROS : public teb_local_planner::TebLocalPlannerROS
{
public:
  using teb_local_planner::TebLocalPlannerROS::saturateVelocityCommand;
  using teb_local_planner::TebLocalPlannerROS::enforceMotionDirectionCommand;
  using teb_local_planner::TebLocalPlannerROS::enforceMinimumTurningRadiusCommand;
  using teb_local_planner::TebLocalPlannerROS::motionDirectionChangeRequiresReinitialization;
};

} // namespace

TEST(TEBBasic, autoResizeLargeValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a large timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt + 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResizeSmallValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a small timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResize)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // modify the timediff in the middle and add a pose with a smaller timediff as the last one
  teb.TimeDiff(5) = dt + 2*dt_hysteresis;
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBKinematicsCarlike, PenalizesBackwardButNotForwardMotion)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.min_turning_radius = 1.2;

  teb_local_planner::VertexPose start(0., 0., 0.);
  teb_local_planner::VertexPose forward(1., 0., 0.);
  teb_local_planner::VertexPose backward(-1., 0., 0.);
  teb_local_planner::EdgeKinematicsCarlike edge;
  edge.setVertex(0, &start);
  edge.setTebConfig(cfg);

  edge.setVertex(1, &forward);
  edge.computeError();
  EXPECT_DOUBLE_EQ(0., edge.error()[1]);
  EXPECT_DOUBLE_EQ(0., edge.error()[2]);

  edge.setVertex(1, &backward);
  edge.computeError();
  EXPECT_DOUBLE_EQ(1., edge.error()[1]);
  EXPECT_DOUBLE_EQ(0., edge.error()[2]);
}

TEST(TEBKinematicsCarlike, KeepsTurningRadiusAsThirdConstraint)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.min_turning_radius = 1.2;

  teb_local_planner::VertexPose start(0., 0., 0.);
  teb_local_planner::VertexPose tight_turn(0.5, 0., 1.0);
  teb_local_planner::EdgeKinematicsCarlike edge;
  edge.setVertex(0, &start);
  edge.setVertex(1, &tight_turn);
  edge.setTebConfig(cfg);
  edge.computeError();

  EXPECT_DOUBLE_EQ(0., edge.error()[1]);
  EXPECT_NEAR(0.7, edge.error()[2], 1e-12);
}

TEST(TEBKinematicsCarlike, ReverseOnlyPenalizesForwardButNotReverseMotion)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.motion_direction_mode = -1;
  cfg.robot.min_turning_radius = 1.35;

  teb_local_planner::VertexPose start(0., 0., 0.);
  teb_local_planner::VertexPose forward(1., 0., 0.);
  teb_local_planner::VertexPose reverse(-1., 0., 0.);
  teb_local_planner::EdgeKinematicsCarlike edge;
  edge.setVertex(0, &start);
  edge.setTebConfig(cfg);

  edge.setVertex(1, &forward);
  edge.computeError();
  EXPECT_DOUBLE_EQ(1., edge.error()[1]);

  edge.setVertex(1, &reverse);
  edge.computeError();
  EXPECT_DOUBLE_EQ(0., edge.error()[1]);
}

TEST(TEBKinematicsCarlike, GraphUsesConfiguredForwardDriveWeight)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.min_turning_radius = 1.2;
  cfg.optim.weight_kinematics_nh = 11.;
  cfg.optim.weight_kinematics_forward_drive = 22.;
  cfg.optim.weight_kinematics_turning_radius = 33.;

  teb_local_planner::ObstContainer obstacles;
  TestableTebOptimalPlanner planner(cfg, &obstacles);
  planner.teb().addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  planner.teb().addPoseAndTimeDiff(
      teb_local_planner::PoseSE2(-1., 0., 0.), 0.3);
  planner.AddTEBVertices();
  planner.AddEdgesKinematicsCarlike();

  ASSERT_EQ(1u, planner.optimizer()->edges().size());
  auto* edge = dynamic_cast<teb_local_planner::EdgeKinematicsCarlike*>(
      *planner.optimizer()->edges().begin());
  ASSERT_NE(nullptr, edge);
  EXPECT_DOUBLE_EQ(11., edge->information()(0, 0));
  EXPECT_DOUBLE_EQ(22., edge->information()(1, 1));
  EXPECT_DOUBLE_EQ(33., edge->information()(2, 2));
  edge->computeError();
  EXPECT_DOUBLE_EQ(1., edge->error()[1]);
}

TEST(TEBVelocity, ZeroBackwardLimitKeepsStopFeasibleAndPenalizesReverse)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.max_vel_x_backwards = 0.;
  cfg.optim.penalty_epsilon = 0.03;

  teb_local_planner::VertexPose start(0., 0., 0.);
  teb_local_planner::VertexPose next(0., 0., 0.);
  teb_local_planner::VertexTimeDiff dt(1.0);
  teb_local_planner::EdgeVelocity edge;
  edge.setVertex(0, &start);
  edge.setVertex(1, &next);
  edge.setVertex(2, &dt);
  edge.setTebConfig(cfg);

  edge.computeError();
  EXPECT_DOUBLE_EQ(0., edge.error()[0]);

  next.pose().x() = -0.2;
  edge.computeError();
  EXPECT_GT(edge.error()[0], 0.);
}

TEST(TEBVelocity, ZeroBackwardLimitClampsResidualReverseCommand)
{
  double vx = -0.2;
  double vy = 0.;
  double omega = 0.1;

  TestableTebLocalPlannerROS::saturateVelocityCommand(
      vx, vy, omega, 0.8, 0., 0.6, 0., false);

  EXPECT_DOUBLE_EQ(0., vx);
  EXPECT_DOUBLE_EQ(0., vy);
  EXPECT_DOUBLE_EQ(0.1, omega);

  vx = -0.2;
  omega = 0.1;
  TestableTebLocalPlannerROS::saturateVelocityCommand(
      vx, vy, omega, 0.8, 0., 0.6, 0., true);

  EXPECT_DOUBLE_EQ(0., vx);
  EXPECT_DOUBLE_EQ(0., vy);
  EXPECT_DOUBLE_EQ(0., omega);
}

TEST(TEBVelocity, FixedGearRejectsOppositeDirectionCommand)
{
  double vx = -0.2;
  double vy = 0.0;
  double omega = 0.1;
  EXPECT_FALSE(TestableTebLocalPlannerROS::enforceMotionDirectionCommand(
      vx, vy, omega, 1));
  EXPECT_DOUBLE_EQ(0.0, vx);
  EXPECT_DOUBLE_EQ(0.0, omega);

  vx = -0.2;
  omega = 0.1;
  EXPECT_TRUE(TestableTebLocalPlannerROS::enforceMotionDirectionCommand(
      vx, vy, omega, -1));
  EXPECT_DOUBLE_EQ(-0.2, vx);
  EXPECT_DOUBLE_EQ(0.1, omega);

  vx = 0.2;
  EXPECT_FALSE(TestableTebLocalPlannerROS::enforceMotionDirectionCommand(
      vx, vy, omega, -1));
  EXPECT_DOUBLE_EQ(0.0, vx);
}

TEST(TEBVelocity, FixedGearChangeInvalidatesWarmStartBand)
{
  EXPECT_FALSE(TestableTebLocalPlannerROS::
      motionDirectionChangeRequiresReinitialization(-1, -1));
  EXPECT_FALSE(TestableTebLocalPlannerROS::
      motionDirectionChangeRequiresReinitialization(1, 1));
  EXPECT_TRUE(TestableTebLocalPlannerROS::
      motionDirectionChangeRequiresReinitialization(1, -1));
  EXPECT_TRUE(TestableTebLocalPlannerROS::
      motionDirectionChangeRequiresReinitialization(-1, 1));
  EXPECT_TRUE(TestableTebLocalPlannerROS::
      motionDirectionChangeRequiresReinitialization(0, -1));
}

TEST(TEBVelocity, TwistCommandHonorsHardAckermannTurningRadius)
{
  double omega = 0.60;
  TestableTebLocalPlannerROS::enforceMinimumTurningRadiusCommand(
      0.405, 1.35, omega);
  EXPECT_NEAR(0.30, omega, 1.0e-12);

  omega = -0.60;
  TestableTebLocalPlannerROS::enforceMinimumTurningRadiusCommand(
      -0.27, 1.35, omega);
  EXPECT_NEAR(-0.20, omega, 1.0e-12);

  omega = 0.40;
  TestableTebLocalPlannerROS::enforceMinimumTurningRadiusCommand(
      0.0, 1.35, omega);
  EXPECT_DOUBLE_EQ(0.0, omega);
}

TEST(TEBViaPointDirection, MeasuresLateralAndWrappedHeadingError)
{
  teb_local_planner::TebConfig cfg;
  teb_local_planner::VertexPose pose(2., 1.25, -M_PI + 0.1);
  teb_local_planner::EdgeViaPointDirection edge;
  edge.setVertex(0, &pose);
  edge.setParameters(cfg, Eigen::Vector2d(1., 1.), M_PI);

  edge.computeError();

  EXPECT_NEAR(-0.25, edge.error()[0], 1e-12);
  EXPECT_NEAR(0.1, edge.error()[1], 1e-12);
}

TEST(TEBViaPointDirection, GraphUsesConfiguredLateralAndHeadingWeights)
{
  teb_local_planner::TebConfig cfg;
  cfg.trajectory.via_points_ordered = true;
  cfg.optim.weight_viapoint = 0.;
  cfg.optim.weight_viapoint_lateral = 12.;
  cfg.optim.weight_viapoint_heading = 34.;

  teb_local_planner::ObstContainer obstacles;
  teb_local_planner::ViaPointContainer via_points;
  via_points.push_back(Eigen::Vector2d(0.5, 0.));
  via_points.push_back(Eigen::Vector2d(1.5, 0.));

  TestableTebOptimalPlanner planner(cfg, &obstacles);
  planner.setViaPoints(&via_points);
  planner.teb().addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(0.5, 0., 0.), 0.3);
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(1., 0., 0.), 0.3);
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(1.5, 0., 0.), 0.3);
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(2., 0., 0.), 0.3);
  planner.AddTEBVertices();
  planner.AddEdgesViaPoints();

  ASSERT_EQ(2u, planner.optimizer()->edges().size());
  for (const auto* optimizer_edge : planner.optimizer()->edges())
  {
    auto* edge = dynamic_cast<const teb_local_planner::EdgeViaPointDirection*>(optimizer_edge);
    ASSERT_NE(nullptr, edge);
    EXPECT_DOUBLE_EQ(12., edge->information()(0, 0));
    EXPECT_DOUBLE_EQ(34., edge->information()(1, 1));
  }
}

TEST(TEBViaPointDirection, ReverseGraphComparesVehicleHeadingAgainstTravelPlusPi)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.motion_direction_mode = -1;
  cfg.trajectory.via_points_ordered = true;
  cfg.optim.weight_viapoint = 0.;
  cfg.optim.weight_viapoint_lateral = 12.;
  cfg.optim.weight_viapoint_heading = 34.;

  teb_local_planner::ObstContainer obstacles;
  teb_local_planner::ViaPointContainer via_points;
  via_points.push_back(Eigen::Vector2d(-0.5, 0.));
  via_points.push_back(Eigen::Vector2d(-1.5, 0.));

  TestableTebOptimalPlanner planner(cfg, &obstacles);
  planner.setViaPoints(&via_points);
  planner.teb().addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(-0.5, 0., 0.), 0.3);
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(-1., 0., 0.), 0.3);
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(-1.5, 0., 0.), 0.3);
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(-2., 0., 0.), 0.3);
  planner.AddTEBVertices();
  planner.AddEdgesViaPoints();

  ASSERT_EQ(2u, planner.optimizer()->edges().size());
  for (auto* optimizer_edge : planner.optimizer()->edges())
  {
    auto* edge = dynamic_cast<teb_local_planner::EdgeViaPointDirection*>(optimizer_edge);
    ASSERT_NE(nullptr, edge);
    edge->computeError();
    EXPECT_NEAR(0., edge->error()[0], 1e-12);
    EXPECT_NEAR(0., edge->error()[1], 1e-12);
  }
}

namespace
{

double straightLineCommand(double initial_velocity,
                           double weight_optimaltime,
                           double weight_acc_lim_x,
                           double maximum_velocity = 1.20)
{
  teb_local_planner::TebConfig cfg;
  cfg.trajectory.dt_ref = 0.3;
  cfg.trajectory.dt_hysteresis = 0.03;
  cfg.trajectory.min_samples = 3;
  cfg.trajectory.max_samples = 500;
  cfg.robot.max_vel_x = maximum_velocity;
  cfg.robot.max_vel_x_backwards = 0.3;
  cfg.robot.max_vel_theta = 0.9;
  cfg.robot.acc_lim_x = 1.0;
  cfg.robot.acc_lim_theta = 0.7;
  cfg.robot.min_turning_radius = 1.35;
  cfg.optim.no_inner_iterations = 10;
  cfg.optim.no_outer_iterations = 6;
  cfg.optim.penalty_epsilon = 0.03;
  cfg.optim.weight_max_vel_x = 5.0;
  cfg.optim.weight_max_vel_theta = 10.0;
  cfg.optim.weight_acc_lim_x = weight_acc_lim_x;
  cfg.optim.weight_acc_lim_theta = 100.0;
  cfg.optim.weight_kinematics_nh = 1000.0;
  cfg.optim.weight_kinematics_forward_drive = 1000.0;
  cfg.optim.weight_kinematics_turning_radius = 700.0;
  cfg.optim.weight_optimaltime = weight_optimaltime;
  cfg.optim.weight_shortest_path = 4.0;
  cfg.optim.weight_obstacle = 50.0;
  cfg.optim.weight_inflation = 0.2;
  cfg.optim.weight_viapoint = 50.0;
  cfg.optim.weight_viapoint_lateral = 200.0;
  cfg.optim.weight_viapoint_heading = 100.0;

  teb_local_planner::ObstContainer obstacles;
  teb_local_planner::ViaPointContainer via_points;
  for (double x = 0.3; x < 7.5; x += 0.3)
    via_points.push_back(Eigen::Vector2d(x, 0.0));

  TestableTebOptimalPlanner planner(cfg, &obstacles,
      boost::make_shared<teb_local_planner::PointRobotFootprint>(),
      teb_local_planner::TebVisualizationPtr(), &via_points);
  geometry_msgs::Twist start_velocity;
  start_velocity.linear.x = initial_velocity;
  EXPECT_TRUE(planner.plan(teb_local_planner::PoseSE2(0.0, 0.0, 0.0),
                           teb_local_planner::PoseSE2(7.5, 0.0, 0.0),
                           &start_velocity, false));
  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
  EXPECT_TRUE(planner.getVelocityCommand(vx, vy, omega, 2));
  EXPECT_NEAR(0.0, vy, 1.0e-6);
  EXPECT_NEAR(0.0, omega, 1.0e-3);
  return vx;
}

}  // namespace

TEST(TEBStraightCruise, UsesWheelSpeedToReachConfiguredCeiling)
{
  const double falsely_stationary = straightLineCommand(0.0, 12.0, 10.0);
  double vx = straightLineCommand(0.80, 12.0, 10.0);

  // A pose-only odometry source that always reports zero twist keeps every
  // receding-horizon solve below cruise speed.  With the real M2 wheel speed,
  // the same obstacle-free straight reaches the configured ceiling; normal
  // ROS command saturation then provides the hard upper bound.
  EXPECT_LT(falsely_stationary, 0.80);
  EXPECT_GE(vx, 1.20);
  double vy = 0.0;
  double omega = 0.0;
  TestableTebLocalPlannerROS::saturateVelocityCommand(
      vx, vy, omega, 1.20, 0.30, 0.90, 0.0, true);
  EXPECT_NEAR(1.20, vx, 1.0e-9);
  EXPECT_DOUBLE_EQ(0.0, vy);
  EXPECT_NEAR(0.0, omega, 1.0e-3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
