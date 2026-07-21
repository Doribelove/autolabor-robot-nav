#include <functional>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <m2_gazebo/trajectory_interpolation.h>

namespace gazebo {

class V2TrajectoryActorPlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model_->GetWorld();
    loop_ = sdf->HasElement("loop") && sdf->Get<bool>("loop");
    z_m_ = sdf->HasElement("z") ? sdf->Get<double>("z") : model_->WorldPose().Pos().Z();
    if (!sdf->HasElement("waypoint")) {
      gzerr << "V2TrajectoryActorPlugin requires at least two waypoints\n";
      return;
    }
    sdf::ElementPtr waypoint = sdf->GetElement("waypoint");
    while (waypoint) {
      m2_gazebo::TrajectoryWaypoint point;
      point.time_s = waypoint->Get<double>("time");
      point.x_m = waypoint->Get<double>("x");
      point.y_m = waypoint->Get<double>("y");
      point.yaw_rad = waypoint->Get<double>("yaw");
      points_.push_back(point);
      waypoint = waypoint->GetNextElement("waypoint");
    }
    if (!m2_gazebo::validTrajectory(points_)) {
      gzerr << "V2TrajectoryActorPlugin rejected invalid waypoint sequence\n";
      points_.clear();
      return;
    }
    start_time_s_ = world_->SimTime().Double();
    connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&V2TrajectoryActorPlugin::update, this));
  }

 private:
  void update() {
    if (points_.empty()) return;
    const double elapsed = world_->SimTime().Double() - start_time_s_;
    const auto point = m2_gazebo::interpolateTrajectory(points_, elapsed, loop_);
    model_->SetWorldPose(ignition::math::Pose3d(
        point.x_m, point.y_m, z_m_, 0.0, 0.0, point.yaw_rad));
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr connection_;
  std::vector<m2_gazebo::TrajectoryWaypoint> points_;
  double start_time_s_{0.0};
  double z_m_{0.5};
  bool loop_{false};
};

GZ_REGISTER_MODEL_PLUGIN(V2TrajectoryActorPlugin)

}  // namespace gazebo
