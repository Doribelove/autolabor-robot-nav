#include <cerrno>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iomanip>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class V3EpisodeRecorderPlugin : public WorldPlugin {
 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    if (!sdf->HasElement("ego_model") || !sdf->HasElement("actor_model") ||
        !sdf->HasElement("output_csv") || !sdf->HasElement("done_marker") ||
        !sdf->HasElement("duration")) {
      gzerr << "V3EpisodeRecorderPlugin missing required SDF fields\n";
      return;
    }
    ego_name_ = sdf->Get<std::string>("ego_model");
    actor_name_ = sdf->Get<std::string>("actor_model");
    final_csv_ = sdf->Get<std::string>("output_csv");
    done_marker_ = sdf->Get<std::string>("done_marker");
    duration_s_ = sdf->Get<double>("duration");
    sample_period_s_ = sdf->HasElement("sample_period")
        ? sdf->Get<double>("sample_period") : 0.05;
    if (!std::isfinite(duration_s_) || duration_s_ <= 0.0 ||
        !std::isfinite(sample_period_s_) || sample_period_s_ <= 0.0) {
      gzerr << "V3EpisodeRecorderPlugin rejected invalid timing\n";
      return;
    }
    temporary_csv_ = final_csv_ + ".active";
    stream_.open(temporary_csv_, std::ios::out | std::ios::trunc);
    if (!stream_) {
      gzerr << "V3EpisodeRecorderPlugin cannot open " << temporary_csv_ << "\n";
      return;
    }
    stream_ << "sim_time_s,ego_x_m,ego_y_m,ego_yaw_rad,"
               "actor_x_m,actor_y_m,actor_yaw_rad\n";
    stream_ << std::setprecision(12);
    start_time_s_ = world_->SimTime().Double();
    next_sample_s_ = start_time_s_;
    connection_ = event::Events::ConnectWorldUpdateEnd(
        std::bind(&V3EpisodeRecorderPlugin::update, this));
  }

 private:
  void update() {
    if (complete_ || !stream_) return;
    const double now_s = world_->SimTime().Double();
    physics::ModelPtr ego = world_->ModelByName(ego_name_);
    physics::ModelPtr actor = world_->ModelByName(actor_name_);
    if (!ego || !actor) return;
    if (now_s + 1e-12 >= next_sample_s_) {
      const auto ego_pose = ego->WorldPose();
      const auto actor_pose = actor->WorldPose();
      stream_ << now_s - start_time_s_ << ","
              << ego_pose.Pos().X() << "," << ego_pose.Pos().Y() << ","
              << ego_pose.Rot().Yaw() << ","
              << actor_pose.Pos().X() << "," << actor_pose.Pos().Y() << ","
              << actor_pose.Rot().Yaw() << "\n";
      next_sample_s_ += sample_period_s_;
    }
    if (now_s - start_time_s_ + 1e-12 < duration_s_) return;

    stream_.flush();
    stream_.close();
    if (std::rename(temporary_csv_.c_str(), final_csv_.c_str()) != 0) {
      gzerr << "V3EpisodeRecorderPlugin rename failed errno=" << errno << "\n";
      return;
    }
    std::ofstream marker(done_marker_, std::ios::out | std::ios::trunc);
    marker << "v3_episode_complete\n";
    marker.flush();
    marker.close();
    complete_ = true;
    world_->SetPaused(true);
  }

  physics::WorldPtr world_;
  event::ConnectionPtr connection_;
  std::ofstream stream_;
  std::string ego_name_;
  std::string actor_name_;
  std::string final_csv_;
  std::string temporary_csv_;
  std::string done_marker_;
  double duration_s_{0.0};
  double sample_period_s_{0.05};
  double start_time_s_{0.0};
  double next_sample_s_{0.0};
  bool complete_{false};
};

GZ_REGISTER_WORLD_PLUGIN(V3EpisodeRecorderPlugin)

}  // namespace gazebo
