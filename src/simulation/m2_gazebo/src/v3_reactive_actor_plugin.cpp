#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <m2_gazebo/v3_reactive_actor.h>

namespace gazebo {

class V3ReactiveActorPlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model_->GetWorld();
    if (!sdf->HasElement("ego_model") || !sdf->HasElement("policy") ||
        !sdf->HasElement("start") || !sdf->HasElement("end") ||
        !sdf->HasElement("conflict")) {
      gzerr << "V3ReactiveActorPlugin missing required SDF fields\n";
      return;
    }
    ego_model_name_ = sdf->Get<std::string>("ego_model");
    if (!m2_gazebo::parseV3ActorPolicy(
            sdf->Get<std::string>("policy"), &config_.policy)) {
      gzerr << "V3ReactiveActorPlugin rejected unknown policy\n";
      return;
    }
    config_.seed = sdf->HasElement("seed")
        ? sdf->Get<unsigned int>("seed") : 0U;
    if (sdf->HasElement("base_speed")) {
      config_.base_speed_mps = sdf->Get<double>("base_speed");
    }
    if (sdf->HasElement("response_distance")) {
      config_.response_distance_m = sdf->Get<double>("response_distance");
    }
    if (sdf->HasElement("response_time_margin")) {
      config_.response_time_margin_s =
          sdf->Get<double>("response_time_margin");
    }
    if (sdf->HasElement("initial_speed")) {
      initial_speed_mps_ = sdf->Get<double>("initial_speed");
    } else {
      initial_speed_mps_ = config_.base_speed_mps;
    }
    if (sdf->HasElement("time_offset")) {
      time_offset_s_ = sdf->Get<double>("time_offset");
    }
    if (sdf->HasElement("start_delay")) {
      start_delay_s_ = sdf->Get<double>("start_delay");
    }
    if (sdf->HasElement("ego_observation_hold")) {
      ego_observation_hold_s_ =
          sdf->Get<double>("ego_observation_hold");
    }
    if (sdf->HasElement("ego_observation_period")) {
      ego_observation_period_s_ =
          sdf->Get<double>("ego_observation_period");
    }
    if (sdf->HasElement("activation_ego_x")) {
      activation_ego_x_m_ = sdf->Get<double>("activation_ego_x");
      use_activation_ego_x_ = true;
    }
    if (sdf->HasElement("response_enable_time")) {
      response_enable_time_s_ =
          sdf->Get<double>("response_enable_time");
    }
    if (!m2_gazebo::validV3ReactiveActorConfig(config_)) {
      gzerr << "V3ReactiveActorPlugin rejected invalid configuration\n";
      return;
    }
    if (!std::isfinite(initial_speed_mps_) || initial_speed_mps_ < 0.0 ||
        !std::isfinite(time_offset_s_) || time_offset_s_ < 0.0 ||
        !std::isfinite(start_delay_s_) || start_delay_s_ < 0.0 ||
        !std::isfinite(ego_observation_hold_s_) ||
        ego_observation_hold_s_ < 0.0 ||
        !std::isfinite(ego_observation_period_s_) ||
        ego_observation_period_s_ < 0.0 ||
        ego_observation_period_s_ > ego_observation_hold_s_ ||
        !std::isfinite(response_enable_time_s_) ||
        response_enable_time_s_ < 0.0 ||
        (use_activation_ego_x_ && !std::isfinite(activation_ego_x_m_))) {
      gzerr << "V3ReactiveActorPlugin rejected initial state\n";
      return;
    }

    start_ = sdf->Get<ignition::math::Vector2d>("start");
    end_ = sdf->Get<ignition::math::Vector2d>("end");
    conflict_ = sdf->Get<ignition::math::Vector2d>("conflict");
    const ignition::math::Vector2d delta = end_ - start_;
    path_length_m_ = delta.Length();
    if (!std::isfinite(path_length_m_) || path_length_m_ <= 0.0) {
      gzerr << "V3ReactiveActorPlugin rejected zero-length path\n";
      return;
    }
    direction_ = delta / path_length_m_;
    yaw_rad_ = std::atan2(direction_.Y(), direction_.X());
    z_m_ = sdf->HasElement("z")
        ? sdf->Get<double>("z") : model_->WorldPose().Pos().Z();
    state_.speed_mps = initial_speed_mps_;
    last_time_s_ = world_->SimTime().Double();
    activated_ = start_delay_s_ <= 0.0 && !use_activation_ego_x_;
    activation_time_s_ = last_time_s_;
    physics::ModelPtr ego = world_->ModelByName(ego_model_name_);
    if (ego) {
      const ignition::math::Vector2d ego_position(
          ego->WorldPose().Pos().X(), ego->WorldPose().Pos().Y());
      last_ego_distance_m_ = (ego_position - conflict_).Length();
      sampled_ego_distance_m_ = last_ego_distance_m_;
      sampled_ego_time_s_ = last_time_s_;
      have_ego_distance_ = true;
    }
    connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&V3ReactiveActorPlugin::update, this));
  }

 private:
  void update() {
    physics::ModelPtr ego = world_->ModelByName(ego_model_name_);
    if (!ego) return;
    const double now_s = world_->SimTime().Double();
    const double dt_s = now_s - last_time_s_;
    if (dt_s <= 0.0 || !std::isfinite(dt_s)) return;
    const ignition::math::Vector2d ego_position(
        ego->WorldPose().Pos().X(), ego->WorldPose().Pos().Y());
    const double ego_distance_m = (ego_position - conflict_).Length();
    if (!activated_ &&
        (now_s < start_delay_s_ ||
         (use_activation_ego_x_ &&
          ego_position.X() < activation_ego_x_m_))) {
      const double sampled_dt_s = now_s - sampled_ego_time_s_;
      if (have_ego_distance_ &&
          ego_observation_period_s_ > 0.0 &&
          sampled_dt_s >= ego_observation_period_s_) {
        held_ego_closing_speed_mps_ =
            (sampled_ego_distance_m_ - ego_distance_m) / sampled_dt_s;
        sampled_ego_distance_m_ = ego_distance_m;
        sampled_ego_time_s_ = now_s;
      } else if (!have_ego_distance_) {
        sampled_ego_distance_m_ = ego_distance_m;
        sampled_ego_time_s_ = now_s;
      }
      have_ego_distance_ = true;
      last_ego_distance_m_ = ego_distance_m;
      last_time_s_ = now_s;
      model_->SetWorldPose(ignition::math::Pose3d(
          start_.X(), start_.Y(), z_m_, 0.0, 0.0, yaw_rad_));
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      model_->SetAngularVel(ignition::math::Vector3d::Zero);
      return;
    }
    if (!activated_) {
      activated_ = true;
      activation_time_s_ = now_s;
      have_ego_distance_ = true;
      last_ego_distance_m_ = ego_distance_m;
    }
    double ego_closing_speed_mps = 0.0;
    if (ego_observation_hold_s_ > 0.0 && have_ego_distance_) {
      const double sampled_dt_s = now_s - sampled_ego_time_s_;
      const double sample_period_s = ego_observation_period_s_ > 0.0
          ? ego_observation_period_s_
          : std::min(0.05, 0.5 * ego_observation_hold_s_);
      if (sampled_dt_s >= sample_period_s) {
        held_ego_closing_speed_mps_ =
            (sampled_ego_distance_m_ - ego_distance_m) / sampled_dt_s;
        sampled_ego_distance_m_ = ego_distance_m;
        sampled_ego_time_s_ = now_s;
      }
      if (now_s - sampled_ego_time_s_ <= ego_observation_hold_s_) {
        ego_closing_speed_mps = held_ego_closing_speed_mps_;
      }
    } else if (have_ego_distance_) {
      ego_closing_speed_mps =
          (last_ego_distance_m_ - ego_distance_m) / dt_s;
    }
    have_ego_distance_ = true;
    last_ego_distance_m_ = ego_distance_m;

    const double actor_distance_m = std::max(
        0.0, (conflict_ - (start_ + direction_ * state_.progress_m)).Dot(
                 direction_));
    const bool was_responding = state_.responding;
    const bool response_enabled = m2_gazebo::v3ResponseEnabled(
        now_s, response_enable_time_s_);
    state_ = m2_gazebo::advanceV3ReactiveActor(
        state_, config_, actor_distance_m, ego_distance_m,
        response_enabled ? ego_closing_speed_mps : 0.0,
        time_offset_s_ + std::max(0.0, now_s - activation_time_s_), dt_s);
    const double interaction_time_s =
        std::max(0.0, now_s - activation_time_s_);
    const int diagnostic_bucket =
        static_cast<int>(std::floor(interaction_time_s / 0.05));
    if (interaction_time_s <= 0.60 &&
        diagnostic_bucket != last_diagnostic_bucket_) {
      last_diagnostic_bucket_ = diagnostic_bucket;
      gzerr << "V3ReactiveActorPlugin diagnostic"
            << " interaction_time=" << interaction_time_s
            << " actor_distance=" << actor_distance_m
            << " ego_distance=" << ego_distance_m
            << " ego_closing_speed=" << ego_closing_speed_mps
            << " response_enabled=" << (response_enabled ? "true" : "false")
            << " actor_ttc=" << state_.actor_ttc_s
            << " ego_ttc=" << state_.ego_ttc_s
            << " responding=" << (state_.responding ? "true" : "false")
            << " actor_speed=" << state_.speed_mps << "\n";
    }
    if (state_.responding != was_responding) {
      gzmsg << "V3ReactiveActorPlugin response="
            << (state_.responding ? "on" : "off")
            << " sim_time=" << now_s
            << " actor_distance=" << actor_distance_m
            << " ego_distance=" << ego_distance_m
            << " ego_closing_speed=" << ego_closing_speed_mps
            << " actor_ttc=" << state_.actor_ttc_s
            << " ego_ttc=" << state_.ego_ttc_s
            << " actor_speed=" << state_.speed_mps << "\n";
    }
    state_.progress_m = std::min(state_.progress_m, path_length_m_);
    const ignition::math::Vector2d position =
        start_ + direction_ * state_.progress_m;
    model_->SetWorldPose(ignition::math::Pose3d(
        position.X(), position.Y(), z_m_, 0.0, 0.0, yaw_rad_));
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
    last_time_s_ = now_s;
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr connection_;
  std::string ego_model_name_;
  m2_gazebo::V3ReactiveActorConfig config_;
  m2_gazebo::V3ReactiveActorState state_;
  ignition::math::Vector2d start_;
  ignition::math::Vector2d end_;
  ignition::math::Vector2d conflict_;
  ignition::math::Vector2d direction_;
  double path_length_m_{0.0};
  double yaw_rad_{0.0};
  double z_m_{0.5};
  double last_time_s_{0.0};
  double initial_speed_mps_{0.0};
  double time_offset_s_{0.0};
  double start_delay_s_{0.0};
  double ego_observation_hold_s_{0.0};
  double ego_observation_period_s_{0.0};
  double activation_ego_x_m_{0.0};
  double activation_time_s_{0.0};
  double response_enable_time_s_{0.0};
  double last_ego_distance_m_{0.0};
  double sampled_ego_distance_m_{0.0};
  double sampled_ego_time_s_{0.0};
  double held_ego_closing_speed_mps_{0.0};
  int last_diagnostic_bucket_{-1};
  bool use_activation_ego_x_{false};
  bool activated_{false};
  bool have_ego_distance_{false};
};

GZ_REGISTER_MODEL_PLUGIN(V3ReactiveActorPlugin)

}  // namespace gazebo
