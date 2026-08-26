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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
