// Simulation truth for the independent evaluator only. No control subscriptions.
// WorldUpdateEnd supplies one physics timestamp for all bodies in a packet.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/WorldState.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace gazebo {
namespace {
geometry_msgs::Vector3 vectorMessage(const ignition::math::Vector3d& v) {
  geometry_msgs::Vector3 out;
  out.x = v.X(); out.y = v.Y(); out.z = v.Z();
  return out;
}
geometry_msgs::Pose poseMessage(const ignition::math::Pose3d& p) {
  geometry_msgs::Pose out;
  out.position.x = p.Pos().X(); out.position.y = p.Pos().Y(); out.position.z = p.Pos().Z();
  out.orientation.x = p.Rot().X(); out.orientation.y = p.Rot().Y();
  out.orientation.z = p.Rot().Z(); out.orientation.w = p.Rot().W();
  return out;
}
geometry_msgs::Twist twistMessage(const ignition::math::Vector3d& v,
                                  const ignition::math::Vector3d& w) {
  geometry_msgs::Twist out;
  out.linear = vectorMessage(v); out.angular = vectorMessage(w);
  return out;
}
}  // namespace

class AstraDenseEvidencePlugin : public WorldPlugin {
 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) {
      gzerr << "AstraDenseEvidencePlugin requires gazebo_ros_api_plugin; no evidence published\n";
      return;
    }
    world_ = world;
    state_hz_ = sdf->HasElement("state_hz") ? sdf->Get<double>("state_hz") : 50.0;
    if (!std::isfinite(state_hz_) || state_hz_ <= 0 || state_hz_ > 500) {
      gzerr << "AstraDenseEvidencePlugin invalid state_hz\n";
      return;
    }
    contacts_ = world_->Physics()->GetContactManager();
    // Retain all contacts, including those with no per-link contact sensor.
    // This changes evidence collection only, not collision or drive parameters.
    contacts_->SetNeverDropContacts(true);
    node_.reset(new ros::NodeHandle("/astra"));
    models_pub_ = node_->advertise<gazebo_msgs::WorldState>("world_models", 200);
    links_pub_ = node_->advertise<gazebo_msgs::WorldState>("world_links", 200);
    dense_models_pub_ = node_->advertise<gazebo_msgs::WorldState>("physics_models", 2000);
    dense_links_pub_ = node_->advertise<gazebo_msgs::WorldState>("physics_links", 2000);
    contacts_pub_ = node_->advertise<gazebo_msgs::ContactsState>("physics_contacts", 2000);
    metadata_pub_ = node_->advertise<std_msgs::String>("physics_metadata", 1, true);
    std::ostringstream metadata;
    metadata.precision(17);
    metadata << "{\"schema\":\"astra_physics_evidence_dense_v2\",\"physics_type\":\""
             << world_->Physics()->GetType() << "\",\"max_step_s\":"
             << world_->Physics()->GetMaxStepSize() << ",\"state_hz\":" << state_hz_
             << ",\"phase\":\"WorldUpdateEnd\",\"contacts_every_physics_step\":true,"
             << "\"all_collision_pairs_retained\":true,\"empty_contact_packets_published\":true,"
             << "\"pose_frame\":\"world\",\"wrench_array\":\"not_reported\","
             << "\"twist_source\":\"Gazebo_body_velocity_may_differ_from_teleport_derivative\","
             << "\"all_body_states_every_physics_step\":true,\"dense_models_topic\":\"/astra/physics_models\",\"dense_links_topic\":\"/astra/physics_links\",\"planner_input_authorized\":false}";
    std_msgs::String message; message.data = metadata.str(); metadata_pub_.publish(message);
    connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(&AstraDenseEvidencePlugin::update, this));
  }

 private:
  void update() {
    const auto now = world_->SimTime();
    const double seconds = now.Double();
    if (seconds <= last_step_s_) {
      if (seconds < last_step_s_) { ++epoch_; last_state_s_ = -1.0; }
      else return;
    }
    last_step_s_ = seconds;
    std_msgs::Header header;
    header.seq = ++sequence_;
    header.stamp = ros::Time(now.sec, now.nsec);
    header.frame_id = "world";
    gazebo_msgs::ContactsState packet; packet.header = header;
    for (unsigned int i = 0; i < contacts_->GetContactCount(); ++i) {
      const auto* contact = contacts_->GetContact(i);
      if (!contact || !contact->collision1 || !contact->collision2 || contact->count <= 0) continue;
      gazebo_msgs::ContactState state;
      state.collision1_name = contact->collision1->GetScopedName();
      state.collision2_name = contact->collision2->GetScopedName();
      std::ostringstream info;
      info.precision(17);
      info << "source_contact_time_s=" << contact->time.Double() << ";epoch=" << epoch_
           << ";positions_normals_world;forces_omitted";
      state.info = info.str();
      const int count = std::min(contact->count, int(MAX_CONTACT_JOINTS));
      for (int k = 0; k < count; ++k) {
        state.contact_positions.push_back(vectorMessage(contact->positions[k]));
        state.contact_normals.push_back(vectorMessage(contact->normals[k]));
        state.depths.push_back(contact->depths[k]);
      }
      // No zero-filled wrench array: force semantics are not measured here.
      packet.states.push_back(state);
    }
    contacts_pub_.publish(packet);
    const bool regular_due = last_state_s_ < 0.0 || seconds-last_state_s_ >= 1.0/state_hz_-1e-9;
    gazebo_msgs::WorldState models, links;
    models.header = links.header = header;
    for (const auto& model : world_->Models()) {
      models.name.push_back(model->GetName());
      models.pose.push_back(poseMessage(model->WorldPose()));
      models.twist.push_back(twistMessage(model->WorldLinearVel(), model->WorldAngularVel()));
      for (const auto& link : model->GetLinks()) {
        links.name.push_back(link->GetScopedName());
        links.pose.push_back(poseMessage(link->WorldPose()));
        links.twist.push_back(twistMessage(link->WorldLinearVel(), link->WorldAngularVel()));
      }
    }
    // All body poses and twists at every solver step permit independent
    // integration/sweep auditing. Low-rate topics retain their original phase.
    dense_models_pub_.publish(models);
    dense_links_pub_.publish(links);
    if (regular_due) {
      last_state_s_ = seconds;
      models_pub_.publish(models);
      links_pub_.publish(links);
    }
  }

  physics::WorldPtr world_;
  physics::ContactManager* contacts_{nullptr};
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Publisher models_pub_, links_pub_, contacts_pub_, metadata_pub_;
  ros::Publisher dense_models_pub_, dense_links_pub_;
  event::ConnectionPtr connection_;
  double state_hz_{50.0}, last_state_s_{-1.0}, last_step_s_{-1.0};
  std::uint32_t sequence_{0}, epoch_{0};
};
GZ_REGISTER_WORLD_PLUGIN(AstraDenseEvidencePlugin)
}  // namespace gazebo
