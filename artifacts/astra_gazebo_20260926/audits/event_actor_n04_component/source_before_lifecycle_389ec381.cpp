#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <m2_gazebo/v3_reactive_actor.h>
#include <m2_gazebo/astra_event_actor_gate_n04.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace gazebo {

class AstraEventActorN04Plugin : public ModelPlugin {
 public:
  ~AstraEventActorN04Plugin() override {
    connection_.reset(); running_=false; queue_.disable();
    if(worker_.joinable()) worker_.join();
  }
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    try { loadChecked(model,sdf); }
    catch(const std::exception& e) {gzerr << "N04 actor load rejected: " << e.what() << "\n";}
  }
  void loadChecked(physics::ModelPtr model, sdf::ElementPtr sdf) {
    model_ = model;
    world_ = model_->GetWorld();
    if (!sdf->HasElement("ego_model") || !sdf->HasElement("policy") ||
        !sdf->HasElement("start") || !sdf->HasElement("end") ||
        !sdf->HasElement("conflict") || !sdf->HasElement("expected_run_id") ||
        !sdf->HasElement("expected_seed") || !sdf->HasElement("expected_method") ||
        !sdf->HasElement("gate_topic") || !sdf->HasElement("max_gate_transport_delay_ns") ||
        !sdf->HasElement("gate_delay")) {
      gzerr << "AstraEventActorN04Plugin missing required SDF fields\n";
      return;
    }
    ego_model_name_ = sdf->Get<std::string>("ego_model");
    if (!m2_gazebo::parseV3ActorPolicy(
            sdf->Get<std::string>("policy"), &config_.policy)) {
      gzerr << "AstraEventActorN04Plugin rejected unknown policy\n";
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
    // Reject obsolete absolute/world-x gate fields rather than silently using
    // two inconsistent timing protocols in a supposedly common environment.
    if (sdf->HasElement("start_delay") || sdf->HasElement("activation_ego_x") || sdf->HasElement("gate_route_x") ||
        sdf->HasElement("route_origin") || sdf->HasElement("route_yaw")) {
      gzerr << "AstraEventActorN04Plugin rejected legacy gate fields\n";
      return;
    }
    identity_={sdf->Get<std::string>("expected_run_id"),
      sdfInteger(sdf,"expected_seed"),sdf->Get<std::string>("expected_method")};
    const auto limit=sdfInteger(sdf,"max_gate_transport_delay_ns");
    gate_delay_s_=sdf->Get<double>("gate_delay");
    if(!m2_gazebo::validN04Identity(identity_) || limit!=m2_gazebo::kN04TransportLimitNs ||
       sdf->Get<std::string>("gate_topic")!="/astra/navigation/gate" || !ros::isInitialized()) {
      gzerr << "N04 gate configuration/ROS rejected\n"; return;
    }
    if (sdf->HasElement("ego_observation_hold")) {
      ego_observation_hold_s_ =
          sdf->Get<double>("ego_observation_hold");
    }
    if (sdf->HasElement("ego_observation_period")) {
      ego_observation_period_s_ =
          sdf->Get<double>("ego_observation_period");
    }
    if (sdf->HasElement("response_enable_time")) {
      response_enable_time_s_ =
          sdf->Get<double>("response_enable_time");
    }
    if (!m2_gazebo::validV3ReactiveActorConfig(config_)) {
      gzerr << "AstraEventActorN04Plugin rejected invalid configuration\n";
      return;
    }
    if (!std::isfinite(initial_speed_mps_) || initial_speed_mps_ < 0.0 ||
        !std::isfinite(time_offset_s_) || time_offset_s_ < 0.0 ||
        !std::isfinite(gate_delay_s_) || gate_delay_s_ < 0.0 || gate_delay_s_ > 22.0 ||
        !std::isfinite(ego_observation_hold_s_) ||
        ego_observation_hold_s_ < 0.0 ||
        !std::isfinite(ego_observation_period_s_) ||
        ego_observation_period_s_ < 0.0 ||
        ego_observation_period_s_ > ego_observation_hold_s_ ||
        !std::isfinite(response_enable_time_s_) ||
        response_enable_time_s_ < 0.0) {
      gzerr << "AstraEventActorN04Plugin rejected initial state\n";
      return;
    }

    start_ = sdf->Get<ignition::math::Vector2d>("start");
    end_ = sdf->Get<ignition::math::Vector2d>("end");
    conflict_ = sdf->Get<ignition::math::Vector2d>("conflict");
    if (!std::isfinite(start_.X()) || !std::isfinite(start_.Y()) ||
        !std::isfinite(end_.X()) || !std::isfinite(end_.Y()) ||
        !std::isfinite(conflict_.X()) || !std::isfinite(conflict_.Y())) {
      gzerr << "AstraEventActorN04Plugin rejected nonfinite world path\n";
      return;
    }
    const ignition::math::Vector2d delta = end_ - start_;
    path_length_m_ = delta.Length();
    if (!std::isfinite(path_length_m_) || path_length_m_ <= 0.0) {
      gzerr << "AstraEventActorN04Plugin rejected zero-length path\n";
      return;
    }
    direction_ = delta / path_length_m_;
    yaw_rad_ = std::atan2(direction_.Y(), direction_.X());
    z_m_ = sdf->HasElement("z")
        ? sdf->Get<double>("z") : model_->WorldPose().Pos().Z();
    if (!std::isfinite(z_m_)) {
      gzerr << "AstraEventActorN04Plugin rejected nonfinite actor height\n";
      return;
    }
    gate_.reset(new m2_gazebo::N04EventGate(
        std::llround(gate_delay_s_*1e9),limit));
    node_.reset(new ros::NodeHandle());
    node_->setCallbackQueue(&queue_);
    status_pub_=node_->advertise<std_msgs::String>("/astra/actors/"+model_->GetName()+"/gate_status",10,true);
    gate_sub_=node_->subscribe("/astra/navigation/gate",10,&AstraEventActorN04Plugin::receive,this);
    running_=true;
    worker_=std::thread([this](){while(running_) queue_.callAvailable(ros::WallDuration(.01));});
    // Internal initial speed retains the original reaction rule, while no
    // physical pose progress is allowed before the relative-delay boundary.
    state_.speed_mps = initial_speed_mps_;
    model_->SetWorldPose(ignition::math::Pose3d(
        start_.X(), start_.Y(), z_m_, 0.0, 0.0, yaw_rad_));
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
    last_time_s_ = world_->SimTime().Double();
    gate_->observeWithoutEgo(nowNs());
    publishStatus();
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
        std::bind(&AstraEventActorN04Plugin::update, this));
  }

 private:
  static int64_t sdfInteger(sdf::ElementPtr sdf,const char* field) {
    auto value=sdf->Get<std::string>(field);
    if(value.empty() || value.find_first_not_of("0123456789")!=std::string::npos) throw std::invalid_argument("SDF integer syntax");
    return std::stoll(value);
  }
  int64_t nowNs() const {auto t=world_->SimTime();return int64_t(t.sec)*1000000000LL+t.nsec;}
  // This callback has no Gazebo reads/writes and does not advance the actor.
  void receive(const std_msgs::String::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      const int64_t receipt=ros::Time::now().toNSec();
      if(receipt<last_receipt_ros_ns_) throw std::invalid_argument("receipt_clock_regression");
      last_receipt_ros_ns_=receipt;
      auto value=m2_gazebo::parseN04Event(message->data,identity_);
      if(!event_.is_null()) {
        if(value!=event_) throw std::invalid_argument("conflicting_duplicate_gate");
        ++duplicates_; return;
      }
      event_=value; receipt_ros_ns_=receipt;
      receipt_steady_ns_=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    } catch(const std::exception& e) {if(pending_fault_.empty()) pending_fault_=e.what();}
  }
  void consumeEvent() {
    bool changed=false;
    {std::lock_guard<std::mutex> lock(mutex_);
      if(!pending_fault_.empty()) gate_->fault(pending_fault_);
      if(!consumed_ && !event_.is_null()) {
        gate_->accept(m2_gazebo::n04Integer(event_.at("source_ns")),nowNs());consumed_=true;changed=true;
      }
      if(duplicates_!=published_duplicates_) {changed=true;published_duplicates_=duplicates_;}
    }
    if(changed) publishStatus();
  }
  void publishStatus() {
    std::lock_guard<std::mutex> lock(mutex_);
    m2_gazebo::N04Json j={{"schema","astra_event_actor_gate_n04_v1"},{"run_id",identity_.run_id},
      {"seed",identity_.seed},{"method",identity_.method},{"model_name",model_->GetName()},
      {"status_source_ns",nowNs()},{"fault",!gate_->valid()},{"reason",gate_->reason()},
      {"gate_delay_ns",gate_->delay()},{"max_gate_transport_delay_ns",m2_gazebo::kN04TransportLimitNs},
      {"duplicate_count",duplicates_},{"receipt_clock_authority","descriptive_ros_clock_only"},
      {"first_begin_authority","Gazebo_WorldUpdateBegin_SimTime"}};
    if(gate_->source()>0) {j["gate_source_ns"]=gate_->source();j["first_begin_ns"]=gate_->firstBegin();
      j["transport_delay_ns"]=gate_->firstBegin()-gate_->source();j["activation_source_ns"]=gate_->source()+gate_->delay();}
    else for(auto key:{"gate_source_ns","first_begin_ns","transport_delay_ns","activation_source_ns"})j[key]=0;
    j["first_active_begin_ns"]=gate_->firstActive()>0?m2_gazebo::N04Json(gate_->firstActive()):m2_gazebo::N04Json(0);
    j["receipt_ros_ns"]=event_.is_null()?m2_gazebo::N04Json(0):m2_gazebo::N04Json(receipt_ros_ns_);
    j["receipt_steady_ns"]=event_.is_null()?m2_gazebo::N04Json(0):m2_gazebo::N04Json(receipt_steady_ns_);
    std_msgs::String message; message.data=j.dump();status_pub_.publish(message);last_status_ns_=nowNs();
  }
  void gateFault(double now_s, const char* reason) {
    gate_->fault(reason);
    if (!gate_fault_logged_) {
      publishStatus();
      gzerr << "AstraEventActorN04Plugin gate_fault model=" << model_->GetName()
            << " sim_time=" << now_s << " reason=" << reason
            << " requires_new_episode=true\n";
      gate_fault_logged_ = true;
    }
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  void update() {
    const double now_s = world_->SimTime().Double();
    consumeEvent();
    if(nowNs()-last_status_ns_>=20000000LL) publishStatus();
    physics::ModelPtr ego = world_->ModelByName(ego_model_name_);
    if (!ego) {
      if (have_ego_distance_ || consumed_) {
        gateFault(now_s, "ego_absent_after_observation_or_gate_event");
      } else if (!gate_->observeWithoutEgo(nowNs())) {
        gateFault(now_s, "clock_fault_before_ego_spawn");
      }
      last_time_s_ = now_s;
      return;
    }
    const double dt_s = now_s - last_time_s_;
    const ignition::math::Vector2d ego_position(
        ego->WorldPose().Pos().X(), ego->WorldPose().Pos().Y());
    const auto gate = gate_->update(nowNs(), ego_position.X(), ego_position.Y());
    if (!gate.valid) {
      gateFault(now_s, "invalid_clock_or_ego_pose");
      return;
    }
    if(gate.newly_crossed || gate.newly_active) publishStatus();
    if (dt_s <= 0.0 || !std::isfinite(dt_s)) return;
    const double ego_distance_m = (ego_position - conflict_).Length();
    if (!gate.active) {
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
    if (gate.newly_active) {
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
    state_ = m2_gazebo::advanceAstraGatedActor(
        state_, config_, actor_distance_m, ego_distance_m,
        response_enabled ? ego_closing_speed_mps : 0.0, time_offset_s_, gate);
    const double interaction_time_s = gate.interaction_time_s;
    const int diagnostic_bucket =
        static_cast<int>(std::floor(interaction_time_s / 0.05));
    if (interaction_time_s <= 0.60 &&
        diagnostic_bucket != last_diagnostic_bucket_) {
      last_diagnostic_bucket_ = diagnostic_bucket;
      gzerr << "AstraEventActorN04Plugin diagnostic model=" << model_->GetName()
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
      gzmsg << "AstraEventActorN04Plugin model=" << model_->GetName() << " response="
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
  std::unique_ptr<m2_gazebo::N04EventGate> gate_;
  m2_gazebo::N04Identity identity_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::CallbackQueue queue_; ros::Subscriber gate_sub_; ros::Publisher status_pub_;
  std::thread worker_; std::atomic<bool> running_{false}; std::mutex mutex_;
  m2_gazebo::N04Json event_; std::string pending_fault_;
  int64_t last_status_ns_{0};
  int64_t receipt_ros_ns_{0},receipt_steady_ns_{0},last_receipt_ros_ns_{0};
  uint64_t duplicates_{0},published_duplicates_{0}; bool consumed_{false};
  double gate_delay_s_{0.0};
  double ego_observation_hold_s_{0.0};
  double ego_observation_period_s_{0.0};
  double response_enable_time_s_{0.0};
  double last_ego_distance_m_{0.0};
  double sampled_ego_distance_m_{0.0};
  double sampled_ego_time_s_{0.0};
  double held_ego_closing_speed_mps_{0.0};
  int last_diagnostic_bucket_{-1};
  bool gate_fault_logged_{false};
  bool have_ego_distance_{false};
};

GZ_REGISTER_MODEL_PLUGIN(AstraEventActorN04Plugin)

}  // namespace gazebo
