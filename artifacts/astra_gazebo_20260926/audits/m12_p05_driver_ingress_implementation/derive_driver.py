#!/usr/bin/env python3
"""Derive new measurement-only P05 plugin; never modify P02 source."""
from pathlib import Path
import hashlib,json
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';G=W/'src/simulation/m2_gazebo'
old=G/'src/astra_physical_ackermann_p02_plugin.cpp';new=G/'src/astra_physical_ackermann_p05_plugin.cpp';text=old.read_text();oldsha=hashlib.sha256(old.read_bytes()).hexdigest();assert not new.exists()
text=text.replace('AstraPhysicalAckermannP02Plugin','AstraPhysicalAckermannP05Plugin')
needle='    ros::NodeHandle root,driver("m2_driver");\n';assert text.count(needle)==1;text=text.replace(needle,needle+'    // Passive command-ingress evidence exists before any callback can arrive.\n    ingress_pub_=driver.advertise<diagnostic_msgs::DiagnosticArray>("command_ingress",1000);\n')
start=text.index('  void cmd(');end=text.index('  void brake(',start)
replacement='''  // Diagnostic-only observation of the original callback/enqueue decision.
  // This is driver acceptance time, not relay publication or force activation.
  void publishCommandIngress(const common::Time& receipt_time,
      const std::string& input_topic,double requested_linear,double requested_angular,
      bool accepted,const std::string& disposition,double activation_time=0.) {
    const std::uint32_t event_sequence=++ingress_sequence_;
    if (receipt_time.sec<0 || receipt_time.nsec<0 || receipt_time.nsec>=1000000000) {
      ROS_ERROR("Astra P05 ingress source time cannot be represented; event omitted, episode evidence incomplete");
      return; // Never fabricate a replacement physical source timestamp.
    }
    diagnostic_msgs::DiagnosticArray message;
    message.header.seq=event_sequence;
    message.header.stamp=ros::Time(receipt_time.sec,receipt_time.nsec);
    message.header.frame_id="world";
    diagnostic_msgs::DiagnosticStatus status;
    status.name="astra_physical_command_ingress";status.hardware_id="simulation-only";
    status.level=accepted?diagnostic_msgs::DiagnosticStatus::OK:diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message=disposition;
    auto number=[](double value) {std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();};
    auto add=[&status](const std::string& key,const std::string& value) {
      diagnostic_msgs::KeyValue item;item.key=key;item.value=value;status.values.push_back(item);
    };
    add("input_topic",input_topic);
    add("requested_linear_x",number(requested_linear));
    add("requested_angular_z",number(requested_angular));
    add("accepted",accepted?"true":"false");
    add("disposition",disposition);
    add("backend_fault",fault_?"true":"false");
    add("activation_time_s",accepted?number(activation_time):"");
    message.status.push_back(status);ingress_pub_.publish(message);
  }
  void cmd(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(msg->linear.x)||!std::isfinite(msg->angular.z)) {
      fault_=true;pending_.clear();
      publishCommandIngress(world_->SimTime(),"cmd_vel",msg->linear.x,msg->angular.z,false,"rejected_invalid_command");
      return;
    }
    const double vx=m2_gazebo::clamp(msg->linear.x,-max_speed_,max_speed_);
    enqueue(vx,m2_gazebo::twistToSteering(vx,msg->angular.z,wheelbase_,max_steer_),"cmd_vel",msg->linear.x,msg->angular.z);
  }
  void ack(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(msg->linear.x)||!std::isfinite(msg->angular.z)) {
      fault_=true;pending_.clear();
      publishCommandIngress(world_->SimTime(),"ackerman_vel",msg->linear.x,msg->angular.z,false,"rejected_invalid_command");
      return;
    }
    enqueue(m2_gazebo::clamp(msg->linear.x,-max_speed_,max_speed_),m2_gazebo::clamp(msg->angular.z,-max_steer_,max_steer_),"ackerman_vel",msg->linear.x,msg->angular.z);
  }
  void enqueue(double speed,double steering,const std::string& input_topic,
      double requested_linear,double requested_angular) {
    if (fault_) {
      publishCommandIngress(world_->SimTime(),input_topic,requested_linear,requested_angular,false,"rejected_existing_fault");
      return;
    }
    // Exactly one read supplies both the old control receipt and new Header.
    const auto receipt_time=world_->SimTime();
    const double now=receipt_time.Double();
    if (!std::isfinite(now)|| (received_&&now<last_receipt_)) {
      fault_=true;pending_.clear();
      publishCommandIngress(receipt_time,input_topic,requested_linear,requested_angular,false,"rejected_clock_fault");
      return;
    }
    received_=true;last_receipt_=now;
    const double activate=now+delay_+m2_gazebo::deterministicSignedJitter(sequence_++,seed_,jitter_);
    last_activation_=std::max(last_activation_,activate);
    pending_.push_back({speed,steering,now,last_activation_});
    if (pending_.size()>512U) {fault_=true;pending_.clear();ROS_ERROR("Astra physical command queue overflow; episode invalid");}
    if (fault_) publishCommandIngress(receipt_time,input_topic,requested_linear,requested_angular,false,"rejected_queue_overflow");
    else publishCommandIngress(receipt_time,input_topic,requested_linear,requested_angular,true,
      last_activation_>now?"accepted_deferred":"accepted_queued",last_activation_);
  }
'''
text=text[:start]+replacement+text[end:]
needle='metadata_pub_,status_pub_;';assert text.count(needle)==1;text=text.replace(needle,'metadata_pub_,status_pub_,ingress_pub_;')
needle='  bool status_clock_fault_logged_{false};';assert text.count(needle)==1;text=text.replace(needle,'  std::uint32_t ingress_sequence_{0U};\n'+needle)
new.write_text(text)
files=[(old,new)]
for rel in ('urdf/m2_physical_p02.urdf.xacro','urdf/m2_physical_p02_gazebo.xacro','launch/spawn_m2_physical_p02.launch'):
 src=G/rel;dst=G/rel.replace('p02','p05');assert not dst.exists();dst.write_text(src.read_text().replace('physical_p02','physical_p05').replace('ackermann_p02','ackermann_p05'));files.append((src,dst))
assert hashlib.sha256(old.read_bytes()).hexdigest()==oldsha
manifest={'scope':'new measurement-only files; no CMake changes, no build, no ROS/Gazebo','parent_driver_sha256':oldsha,'derived_files':[{'parent':str(a),'parent_sha256':hashlib.sha256(a.read_bytes()).hexdigest(),'new':str(b),'new_sha256':hashlib.sha256(b.read_bytes()).hexdigest()} for a,b in files],'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'command':'/usr/bin/python3 '+str(Path(__file__).resolve())}
(D/'derivation_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))
