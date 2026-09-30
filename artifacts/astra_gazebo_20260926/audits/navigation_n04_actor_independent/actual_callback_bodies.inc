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
