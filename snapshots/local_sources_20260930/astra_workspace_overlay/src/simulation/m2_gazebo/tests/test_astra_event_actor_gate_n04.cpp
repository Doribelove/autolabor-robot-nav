#include <m2_gazebo/astra_event_actor_gate_n04.h>
#include <cassert>
#include <iostream>
#include <random>
using namespace m2_gazebo;
int main() {
  int checks=0;auto check=[&](bool x){assert(x);++checks;};
  N04Identity id{"fixture_n04",263001001,"astra_analytic_route_v1"};
  N04Json payload={{"schema","astra_common_navigation_v1"},{"run_id",id.run_id},{"seed",id.seed},{"method",id.method},
    {"source_ns",7000000000LL},{"source","actual_odom_header_first_crossing"},{"pose_map",{-4.,0.,0.}},
    {"body_twist",{.8,0.,0.}},{"activation_source_ns",5000000000LL},{"goal_pose",{5.,0.,0.}},
    {"deadline_source_ns",29000000000LL},{"deadline_ns",29000000000LL}};
  check(parseN04Event(payload.dump(),id)==payload);
  auto reject=[&](const std::string& x){try {parseN04Event(x,id);}catch(const std::exception&){++checks;return;}assert(false);};
  for(const auto* key:{"seed","source_ns","activation_source_ns","deadline_ns","deadline_source_ns"}) {
    auto p=payload;p[key]=p[key].get<double>();reject(p.dump());p[key]=true;reject(p.dump());
  }
  for(const auto* key:{"schema","run_id","method","source"}) {auto p=payload;p[key]="wrong";reject(p.dump());}
  auto p=payload;p["source_ns"]=7000000001LL;reject(p.dump());p=payload;p["pose_map"]={0.,0.};reject(p.dump());
  p=payload;p["body_twist"][0]="0.8";reject(p.dump());p=payload;p["unknown"]=0;reject(p.dump());
  reject("{\"seed\":1,"+payload.dump().substr(1));
  check(!validN04Identity({"UNBOUND",1,id.method}));check(!validN04Identity({"x",1,"dwa"}));
  N04EventGate late(0,kN04TransportLimitNs);late.observeWithoutEgo(6998000000LL);
  check(late.accept(7000000000LL,7040000000LL));
  auto g=late.update(7040000000LL,-4.,0.);check(g.active&&g.active_dt_s==0.&&g.newly_crossed&&g.newly_active);
  check(std::abs(g.interaction_time_s-.04)<1e-12);g=late.update(7042000000LL,-4.,0.);check(std::abs(g.active_dt_s-.002)<1e-12);
  check(!late.update(7040000000LL,-4.,0.).valid);check(!late.update(7050000000LL,-4.,0.).valid);
  for(auto offset:{-1LL,50000001LL}) {N04EventGate f(0,kN04TransportLimitNs);check(!f.accept(7000000000LL,7000000000LL+offset));}
  N04EventGate edge(0,kN04TransportLimitNs);check(edge.accept(7000000000LL,7050000000LL));check(edge.update(7050000000LL,0,0).active_dt_s==0.);
  N04EventGate delayed(200000000,kN04TransportLimitNs);delayed.update(6998000000LL,0,0);delayed.accept(7000000000LL,7002000000LL);
  check(!delayed.update(7002000000LL,0,0).active);check(!delayed.update(7198000000LL,0,0).active);
  check(delayed.update(7200000000LL,0,0).active_dt_s==0.);check(std::abs(delayed.update(7202000000LL,0,0).active_dt_s-.002)<1e-12);
  check(!delayed.observeWithoutEgo(7204000000LL));
  N04EventGate nan(0,kN04TransportLimitNs);check(!nan.update(0,NAN,0).valid);
  N04EventGate duplicate(0,kN04TransportLimitNs);check(duplicate.accept(7000000000LL,7002000000LL));check(duplicate.accept(7000000000LL,8000000000LL));check(!duplicate.accept(7020000000LL,8000000000LL));
  // Independently sum the permitted integer intervals; no pre-receipt time.
  std::mt19937 rng(260926);for(int trial=0;trial<300;++trial) {
    int64_t lag=(rng()%26)*2000000LL, delay=(rng()%201)*2000000LL,first=7000000000LL+lag;
    N04EventGate gate(delay,kN04TransportLimitNs);gate.update(6998000000LL,0,0);check(gate.accept(7000000000LL,first));double integrated=0;
    for(int64_t t=first;t<=7600000000LL;t+=2000000LL) integrated+=gate.update(t,0,0).active_dt_s;
    check(std::abs(integrated-(7600000000LL-std::max<int64_t>(first,7000000000LL+delay))*1e-9)<1e-12);
  }
  // New gate delegates the exact old reaction update, not a replacement policy.
  V3ReactiveActorConfig cfg;V3ReactiveActorState state;state.speed_mps=.75;
  AstraGateStep step;step.active=true;step.active_dt_s=.002;step.interaction_time_s=.4;
  auto a=advanceAstraGatedActor(state,cfg,1.2,2.4,.8,.2,step);
  auto b=advanceV3ReactiveActor(state,cfg,1.2,2.4,.8,.2+.4,.002);
  check(a.speed_mps==b.speed_mps&&a.progress_m==b.progress_m&&a.responding==b.responding);
  std::cout << "checks="<<checks<<" passed\n";
}
