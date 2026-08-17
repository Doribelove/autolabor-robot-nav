#include <gtest/gtest.h>

#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/timed_elastic_band.h>

namespace
{

class TestableTebOptimalPlanner : public teb_local_planner::TebOptimalPlanner
{
public:
  using teb_local_planner::TebOptimalPlanner::TebOptimalPlanner;
  using teb_local_planner::TebOptimalPlanner::AddEdgesKinematicsCarlike;
  using teb_local_planner::TebOptimalPlanner::AddTEBVertices;
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
