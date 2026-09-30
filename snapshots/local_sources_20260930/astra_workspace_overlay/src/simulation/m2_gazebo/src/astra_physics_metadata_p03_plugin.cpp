// P03 evaluator-only evidence. No physics setters, ROS subscriptions/services,
// command publishing, hashing or filesystem I/O. Root binds loaded binaries and
// the authorized run's launch/node/service-use closure outside this plugin.
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/ode/ode.h>
#include <gazebo_msgs/WorldState.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace gazebo {
namespace {
std::string quoted(const std::string& value) {
  std::ostringstream out;out<<'"';
  for(unsigned char c:value) {
    if(c=='"'||c=='\\') out<<'\\'<<c;
    else if(c<32) out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;
    else out<<c;
  }
  out<<'"';return out.str();
}
void number(std::ostream& out,double value) {
  if(std::isfinite(value)) out<<std::setprecision(17)<<value;else out<<"null";
}
void vec(std::ostream& out,const ignition::math::Vector3d& value) {
  out<<'[';number(out,value.X());out<<',';number(out,value.Y());out<<',';number(out,value.Z());out<<']';
}
void poseJson(std::ostream& out,const ignition::math::Pose3d& value) {
  out<<"{\"xyz\":";vec(out,value.Pos());out<<",\"xyzw\":[";
  const double q[]={value.Rot().X(),value.Rot().Y(),value.Rot().Z(),value.Rot().W()};
  for(size_t i=0;i<4;++i){if(i)out<<',';number(out,q[i]);}out<<"]}";
}
geometry_msgs::Pose poseMessage(const ignition::math::Pose3d& p) {
  geometry_msgs::Pose value;value.position.x=p.Pos().X();value.position.y=p.Pos().Y();value.position.z=p.Pos().Z();
  value.orientation.x=p.Rot().X();value.orientation.y=p.Rot().Y();value.orientation.z=p.Rot().Z();value.orientation.w=p.Rot().W();return value;
}
geometry_msgs::Twist twistMessage(const ignition::math::Vector3d& v,const ignition::math::Vector3d& w) {
  geometry_msgs::Twist value;value.linear.x=v.X();value.linear.y=v.Y();value.linear.z=v.Z();
  value.angular.x=w.X();value.angular.y=w.Y();value.angular.z=w.Z();return value;
}
std::string pluginRoster(sdf::ElementPtr model) {
  std::ostringstream out;out<<'[';bool first=true;
  if(model&&model->HasElement("plugin")) for(auto p=model->GetElement("plugin");p;p=p->GetNextElement("plugin")) {
    if(!first)out<<',';
    first=false;
    out<<"{\"name\":"<<quoted(p->Get<std::string>("name"))<<",\"filename\":"<<quoted(p->Get<std::string>("filename"))<<'}';
  }
  out<<']';return out.str();
}
}  // namespace

class AstraPhysicsMetadataP03Plugin : public WorldPlugin {
 public:
  ~AstraPhysicsMetadataP03Plugin() override {pre_.reset();end_.reset();}
  void Load(physics::WorldPtr world,sdf::ElementPtr sdf) override {
    if(!ros::isInitialized()||world->Physics()->GetType()!="ode") {
      gzerr<<"P03 passive evidence requires initialized ROS and ODE; no evidence qualified\n";return;
    }
    world_=world;ode_=boost::dynamic_pointer_cast<physics::ODEPhysics>(world_->Physics());
    if(!ode_){gzerr<<"P03 ODE implementation cast failed\n";return;}
    if(sdf->HasElement("run_manifest_sha256")) declared_manifest_=sdf->Get<std::string>("run_manifest_sha256");
    node_.reset(new ros::NodeHandle("/astra/p03"));
    pre_models_=node_->advertise<gazebo_msgs::WorldState>("pre_models",2000);
    pre_links_=node_->advertise<gazebo_msgs::WorldState>("pre_links",2000);
    end_models_=node_->advertise<gazebo_msgs::WorldState>("end_models",2000);
    end_links_=node_->advertise<gazebo_msgs::WorldState>("end_links",2000);
    metadata_=node_->advertise<std_msgs::String>("runtime_metadata",10,true);
    status_=node_->advertise<std_msgs::String>("step_status",2000);
    pre_=event::Events::ConnectBeforePhysicsUpdate(std::bind(&AstraPhysicsMetadataP03Plugin::before,this));
    end_=event::Events::ConnectWorldUpdateEnd(std::bind(&AstraPhysicsMetadataP03Plugin::after,this));
  }
 private:
  struct Snapshot {
    gazebo_msgs::WorldState models,links;
    std::string config,enabled;
    bool finite{true};
  };
  Snapshot capture() const {
    Snapshot result;const auto time=world_->SimTime();
    result.models.header.stamp=result.links.header.stamp=ros::Time(time.sec,time.nsec);
    result.models.header.frame_id=result.links.header.frame_id="world";
    std::ostringstream config,enabled;
    config<<"{\"physics_type\":\"ode\",\"max_step_s\":";number(config,world_->Physics()->GetMaxStepSize());
    config<<",\"solver_type\":"<<quoted(world_->Physics()->GetSDF()->GetElement("ode")->GetElement("solver")->Get<std::string>("type"));
    config<<",\"world_erp\":";number(config,dWorldGetERP(ode_->GetWorldId()));
    config<<",\"world_cfm\":";number(config,dWorldGetCFM(ode_->GetWorldId()));
    config<<",\"quickstep_iterations\":"<<dWorldGetQuickStepNumIterations(ode_->GetWorldId());
    result.finite=result.finite&&std::isfinite(world_->Physics()->GetMaxStepSize())&&
      std::isfinite(dWorldGetERP(ode_->GetWorldId()))&&std::isfinite(dWorldGetCFM(ode_->GetWorldId()));
    config<<",\"models\":[";enabled<<'[';bool first_model=true,first_state=true;
    for(const auto& model:world_->Models()) {
      if(!first_model)config<<',';
      first_model=false;
      const auto p=model->WorldPose();const auto v=model->WorldLinearVel(),w=model->WorldAngularVel();
      result.finite=result.finite&&p.IsFinite()&&v.IsFinite()&&w.IsFinite();
      result.models.name.push_back(model->GetName());result.models.pose.push_back(poseMessage(p));result.models.twist.push_back(twistMessage(v,w));
      config<<"{\"name\":"<<quoted(model->GetName())<<",\"static\":"<<(model->IsStatic()?"true":"false")
        <<",\"plugins\":"<<pluginRoster(model->GetSDF())<<",\"links\":[";bool first_link=true;
      for(const auto& link:model->GetLinks()) {
        if(!first_link)config<<',';
        first_link=false;
        const auto lp=link->WorldPose();const auto lv=link->WorldLinearVel(),lw=link->WorldAngularVel();
        result.finite=result.finite&&lp.IsFinite()&&lv.IsFinite()&&lw.IsFinite();
        result.links.name.push_back(link->GetScopedName());result.links.pose.push_back(poseMessage(lp));result.links.twist.push_back(twistMessage(lv,lw));
        config<<"{\"name\":"<<quoted(link->GetScopedName());
        const auto inertial=link->GetInertial();
        if(inertial) {
          config<<",\"mass_kg\":";number(config,inertial->Mass());config<<",\"com_local_m\":";vec(config,inertial->CoG());
          config<<",\"inertia_pose\":{\"xyz\":";vec(config,inertial->Pose().Pos());
          config<<",\"rpy\":";vec(config,inertial->Pose().Rot().Euler());config<<'}';
          config<<",\"inertia_ixx_iyy_izz_ixy_ixz_iyz\":[";
          const double tensor[]={inertial->IXX(),inertial->IYY(),inertial->IZZ(),inertial->IXY(),inertial->IXZ(),inertial->IYZ()};
          for(size_t i=0;i<6;++i){if(i)config<<',';number(config,tensor[i]);result.finite=result.finite&&std::isfinite(tensor[i]);}
          config<<']';result.finite=result.finite&&std::isfinite(inertial->Mass())&&inertial->Pose().IsFinite();
        } else config<<",\"mass_kg\":null,\"com_local_m\":null";
        auto ode_link=boost::dynamic_pointer_cast<physics::ODELink>(link);const dBodyID body=ode_link?ode_link->GetODEId():nullptr;
        config<<",\"has_ode_body\":"<<(body?"true":"false");
        if(body) {
          dMass native_mass;dBodyGetMass(body,&native_mass);
          config<<",\"native_ode_mass_kg\":";number(config,native_mass.mass);
          config<<",\"native_ode_mass_center\":[";
          for(size_t i=0;i<3;++i){if(i)config<<',';number(config,native_mass.c[i]);result.finite=result.finite&&std::isfinite(native_mass.c[i]);}config<<']';
          config<<",\"native_ode_inertia_rowmajor_stride4\":[";
          for(size_t i=0;i<12;++i){if(i)config<<',';number(config,native_mass.I[i]);result.finite=result.finite&&std::isfinite(native_mass.I[i]);}config<<']';
          result.finite=result.finite&&std::isfinite(native_mass.mass);
          dVector3 axis;dBodyGetFiniteRotationAxis(body,axis);
          config<<",\"finite_rotation_mode\":"<<dBodyGetFiniteRotationMode(body)<<",\"finite_rotation_axis\":[";
          for(size_t i=0;i<3;++i){if(i)config<<',';number(config,axis[i]);result.finite=result.finite&&std::isfinite(axis[i]);}config<<']';
          config<<",\"kinematic\":"<<(dBodyIsKinematic(body)?"true":"false")<<",\"auto_disable\":"<<(dBodyGetAutoDisableFlag(body)?"true":"false");
          config<<",\"linear_damping\":";number(config,dBodyGetLinearDamping(body));
          config<<",\"angular_damping\":";number(config,dBodyGetAngularDamping(body));
          config<<",\"linear_damping_threshold\":";number(config,dBodyGetLinearDampingThreshold(body));
          config<<",\"angular_damping_threshold\":";number(config,dBodyGetAngularDampingThreshold(body));
          result.finite=result.finite&&std::isfinite(dBodyGetLinearDamping(body))&&std::isfinite(dBodyGetAngularDamping(body))&&
            std::isfinite(dBodyGetLinearDampingThreshold(body))&&std::isfinite(dBodyGetAngularDampingThreshold(body));
          const double max_angular=dBodyGetMaxAngularSpeed(body);
          result.finite=result.finite&&(std::isfinite(max_angular)||(std::isinf(max_angular)&&max_angular>0));
          config<<",\"max_angular_speed_radps\":";number(config,max_angular);
          config<<",\"max_angular_speed_infinite\":"<<(std::isinf(max_angular)&&max_angular>0?"true":"false");
          if(!first_state)enabled<<',';
          first_state=false;
          enabled<<"{\"name\":"<<quoted(link->GetScopedName())<<",\"enabled\":"<<(dBodyIsEnabled(body)?"true":"false");
          const auto native_position=dBodyGetPosition(body),native_quaternion=dBodyGetQuaternion(body);
          enabled<<",\"native_com_world_m\":[";
          for(size_t i=0;i<3;++i){if(i)enabled<<',';number(enabled,native_position[i]);result.finite=result.finite&&std::isfinite(native_position[i]);}
          enabled<<"],\"native_quaternion_xyzw\":[";
          const size_t order[]={1,2,3,0};
          for(size_t i=0;i<4;++i){if(i)enabled<<',';number(enabled,native_quaternion[order[i]]);result.finite=result.finite&&std::isfinite(native_quaternion[order[i]]);}
          enabled<<"]}";
        }
        config<<",\"collisions\":[";bool first_collision=true;
        for(const auto& collision:link->GetCollisions()) {
          if(!first_collision)config<<',';
          first_collision=false;
          config<<"{\"name\":"<<quoted(collision->GetScopedName())<<",\"relative_pose\":";
          const auto cp=collision->RelativePose();poseJson(config,cp);result.finite=result.finite&&cp.IsFinite();
          const auto shape=collision->GetShape();
          const auto box=boost::dynamic_pointer_cast<physics::BoxShape>(shape);
          const auto cylinder=boost::dynamic_pointer_cast<physics::CylinderShape>(shape);
          if(box) {
            config<<",\"kind\":\"box\",\"size_xyz_m\":";vec(config,box->Size());result.finite=result.finite&&box->Size().IsFinite();
          } else if(cylinder) {
            config<<",\"kind\":\"cylinder\",\"radius_m\":";number(config,cylinder->GetRadius());
            config<<",\"length_m\":";number(config,cylinder->GetLength());
            result.finite=result.finite&&std::isfinite(cylinder->GetRadius())&&std::isfinite(cylinder->GetLength());
          } else config<<",\"kind\":\"other\",\"shape_type\":"<<collision->GetShapeType();
          config<<'}';
        }
        config<<"]}";
      }
      config<<"]}";
    }
    config<<"]}";enabled<<']';result.config=config.str();result.enabled=enabled.str();return result;
  }
  void configuration(const Snapshot& snap,const char* phase) {
    if(snap.config==last_config_)return;
    last_config_=snap.config;++revision_;
    std::ostringstream out;out<<"{\"schema\":\"astra_physics_runtime_p03_v1\",\"revision\":"<<revision_
      <<",\"stamp_ns\":"<<snap.links.header.stamp.toNSec()<<",\"phase\":"<<quoted(phase)
      <<",\"epoch\":"<<epoch_<<",\"valid_numeric\":"<<(snap.finite?"true":"false")
      <<",\"run_manifest_sha256_declared_not_authenticated\":"<<quoted(declared_manifest_)
      <<",\"planner_input_authorized\":false,\"passive_observer\":true,\"published_twist_is_not_used_as_path_bound\":true,\"config\":"<<snap.config<<'}';
    std_msgs::String message;message.data=out.str();metadata_.publish(message);
  }
  void before() {
    const auto time=world_->SimTime();const int64_t stamp=int64_t(time.sec)*1000000000LL+time.nsec;
    if(stamp<=0||(last_pre_ns_>=0&&stamp<=last_pre_ns_)) {++epoch_;clock_fault_=true;}
    last_pre_ns_=stamp;pre_snapshot_=capture();configuration(pre_snapshot_,"BeforePhysicsUpdate");pre_revision_=revision_;have_pre_=true;
    pre_models_.publish(pre_snapshot_.models);pre_links_.publish(pre_snapshot_.links);
  }
  void after() {
    const auto snap=capture();configuration(snap,"WorldUpdateEnd");
    const uint64_t stamp=snap.links.header.stamp.toNSec();
    const bool paired=have_pre_&&pre_snapshot_.links.header.stamp.toNSec()==stamp;
    const bool same_roster=paired&&pre_snapshot_.links.name==snap.links.name&&pre_snapshot_.models.name==snap.models.name;
    const bool stable_config=paired&&pre_revision_==revision_;
    const bool valid=paired&&same_roster&&stable_config&&snap.finite&&pre_snapshot_.finite&&!clock_fault_;
    end_models_.publish(snap.models);end_links_.publish(snap.links);
    std::ostringstream out;out<<"{\"schema\":\"astra_physics_step_p03_v1\",\"stamp_ns\":"<<stamp<<",\"step_index\":"<<sequence_++
      <<",\"epoch\":"<<epoch_<<",\"valid\":"<<(valid?"true":"false")<<",\"pre_end_same_source_stamp\":"<<(paired?"true":"false")
      <<",\"same_roster\":"<<(same_roster?"true":"false")<<",\"config_stable_within_step\":"<<(stable_config?"true":"false")
      <<",\"pre_revision\":"<<pre_revision_<<",\"end_revision\":"<<revision_
      <<",\"pre_enabled\":"<<(paired?pre_snapshot_.enabled:"[]")<<",\"end_enabled\":"<<snap.enabled<<'}';
    std_msgs::String message;message.data=out.str();status_.publish(message);have_pre_=false;
  }
  physics::WorldPtr world_;physics::ODEPhysicsPtr ode_;std::unique_ptr<ros::NodeHandle> node_;
  event::ConnectionPtr pre_,end_;ros::Publisher pre_models_,pre_links_,end_models_,end_links_,metadata_,status_;
  Snapshot pre_snapshot_;std::string last_config_,declared_manifest_;
  uint64_t revision_{0},pre_revision_{0},epoch_{0},sequence_{0};int64_t last_pre_ns_{-1};bool have_pre_{false},clock_fault_{false};
};
GZ_REGISTER_WORLD_PLUGIN(AstraPhysicsMetadataP03Plugin)
}  // namespace gazebo
