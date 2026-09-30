#pragma once
// N04 event authority; pure helper, no ROS or Gazebo access.
#include <m2_gazebo/astra_actor_gate.h>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace m2_gazebo {
using N04Json = nlohmann::json;
constexpr int64_t kN04TransportLimitNs = 50000000;
struct N04Identity { std::string run_id; int64_t seed; std::string method; };
inline bool validN04Identity(const N04Identity& x) {
  const std::set<std::string> methods={"astra_analytic_route_v1", "actual_teb_common_route", "dwa_common_route", "frozen_r2_v060_sensor_lifecycle_v1", "astra_pct_route_v1"};
  return !x.run_id.empty() && x.run_id.size() <= 160 && x.run_id.find("UNBOUND")==std::string::npos &&
      x.seed>0 && x.seed<=2147483647 && methods.count(x.method);
}
inline int64_t n04Integer(const N04Json& j) {
  if (!j.is_number_integer() || j.is_boolean()) throw std::invalid_argument("integer required");
  if (j.is_number_unsigned() && j.get<uint64_t>()>uint64_t(INT64_MAX)) throw std::invalid_argument("integer overflow");
  return j.get<int64_t>();
}
inline N04Json parseN04Event(const std::string& raw, const N04Identity& expected) {
  if (!validN04Identity(expected) || raw.size()>8192) throw std::invalid_argument("invalid identity or oversized gate");
  std::vector<std::set<std::string>> keys;
  auto callback=[&keys](int, N04Json::parse_event_t ev, N04Json& parsed) {
    if(ev==N04Json::parse_event_t::object_start) keys.emplace_back();
    if(ev==N04Json::parse_event_t::key && !keys.back().insert(parsed.get<std::string>()).second) throw std::invalid_argument("duplicate JSON key");
    if(ev==N04Json::parse_event_t::object_end) keys.pop_back();
    return true;
  };
  auto j=N04Json::parse(raw,callback);
  const std::set<std::string> fields={"schema","run_id","seed","method","source_ns","source","pose_map","body_twist","activation_source_ns","goal_pose","deadline_source_ns","deadline_ns"};
  if(!j.is_object() || j.size()!=fields.size()) throw std::invalid_argument("gate schema fields");
  for(auto it=j.begin();it!=j.end();++it) if(!fields.count(it.key())) throw std::invalid_argument("unknown gate field");
  if(j.at("schema")!="astra_common_navigation_v1" || j.at("run_id")!=expected.run_id ||
     n04Integer(j.at("seed"))!=expected.seed || j.at("method")!=expected.method ||
     j.at("source")!="actual_odom_header_first_crossing") throw std::invalid_argument("gate identity or authority");
  const auto source=n04Integer(j.at("source_ns"));
  const auto activation=n04Integer(j.at("activation_source_ns"));
  if(source<=0 || source>1000000000000LL || source%2000000 || activation<5000000000LL || activation>source ||
     n04Integer(j.at("deadline_source_ns"))!=source+22000000000LL || n04Integer(j.at("deadline_ns"))!=source+22000000000LL)
    throw std::invalid_argument("gate source or deadline");
  for(const auto* field:{"pose_map","body_twist","goal_pose"}) {
    const auto& a=j.at(field); if(!a.is_array() || a.size()!=3) throw std::invalid_argument("gate vector shape");
    for(const auto& x:a) if(!x.is_number() || !std::isfinite(x.get<double>())) throw std::invalid_argument("nonfinite gate vector");
  }
  return j;
}

// Called only by Begin. Callback evidence is queued separately. The first
// receiving Begin consumes zero elapsed interval: no unreceived time is moved.
class N04EventGate {
 public:
  N04EventGate(int64_t delay_ns,int64_t limit_ns):delay_ns_(delay_ns),limit_ns_(limit_ns) {
    if(delay_ns<0 || delay_ns>22000000000LL || limit_ns!=kN04TransportLimitNs) fault("invalid_gate_configuration");
  }
  void fault(const std::string& why) { if(reason_.empty()) reason_=why; }
  bool valid() const {return reason_.empty();}
  const std::string& reason() const {return reason_;}
  bool observeWithoutEgo(int64_t now) { if(have_ego_) fault("ego_disappeared_after_observation"); return tick(now); }
  bool accept(int64_t source,int64_t now) {
    if(!valid()) return false;
    if(seen_) { if(source!=source_) fault("conflicting_gate_source"); return valid(); }
    if(source<=0 || now<source || now-source>limit_ns_) {fault("gate_transport_delay_or_future_source");return false;}
    seen_=true; source_=source; first_begin_=now; return true;
  }
  AstraGateStep update(int64_t now,double x,double y) {
    AstraGateStep g;
    int64_t previous=have_time_?last_:now;
    if(!std::isfinite(x)||!std::isfinite(y)) fault("nonfinite_ego_pose");
    have_ego_=true;
    if(!tick(now)) {g.valid=false;return g;}
    g.gate_seen=seen_;
    if(!seen_) return g;
    g.newly_crossed=!reported_; reported_=true;
    g.gate_time_s=source_*1e-9; g.activation_time_s=(source_+delay_ns_)*1e-9;
    g.active=now>=source_+delay_ns_; g.newly_active=g.active&&!active_; active_=g.active;
    if(g.newly_active) first_active_=now;
    if(g.active) {
      g.active_dt_s=std::max<int64_t>(0,now-std::max({previous,source_+delay_ns_,first_begin_}))*1e-9;
      g.interaction_time_s=std::max<int64_t>(0,now-source_-delay_ns_)*1e-9;
    }
    return g;
  }
  int64_t source()const{return source_;} int64_t firstBegin()const{return first_begin_;}
  int64_t firstActive()const{return first_active_;} int64_t delay()const{return delay_ns_;}
 private:
  bool tick(int64_t now) { if(now<0 || (have_time_ && now<last_)) fault("source_clock_regression");last_=now;have_time_=true;return valid(); }
  int64_t delay_ns_,limit_ns_,last_{0},source_{0},first_begin_{0},first_active_{0};
  bool have_time_{false},have_ego_{false},seen_{false},reported_{false},active_{false}; std::string reason_;
};
} // namespace m2_gazebo
