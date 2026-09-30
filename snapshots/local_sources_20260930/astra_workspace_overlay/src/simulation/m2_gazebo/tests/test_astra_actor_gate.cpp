#include <m2_gazebo/astra_actor_gate.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace m2_gazebo;
static bool close(double x,double y) { return std::abs(x-y)<1e-10; }
static void same(const V3ReactiveActorState& a,const V3ReactiveActorState& b) {
  assert(a.speed_mps==b.speed_mps);assert(a.progress_m==b.progress_m);
  assert(a.responding==b.responding);assert(a.ego_approaching==b.ego_approaching);
  assert(a.actor_ttc_s==b.actor_ttc_s);assert(a.ego_ttc_s==b.ego_ttc_s);
}

int main() {
  // An absolute simulation time exceeding any old start delay cannot release
  // an actor whose ego has never crossed the spatial route gate.
  AstraRouteFrame identity;
  AstraGateRelativeActivation gate(identity,-4.02,.25);
  V3ReactiveActorConfig cfg;V3ReactiveActorState state;state.speed_mps=1.;
  for (int i=0;i<100;++i) {
    auto step=gate.update(100.+i*.02,-4.03,0.);
    assert(step.valid&&!step.gate_seen&&!step.active);
    state=advanceAstraGatedActor(state,cfg,2.,2.,1.,0.,step);
    assert(state.progress_m==0.);
  }
  auto cross=gate.update(102.,-4.02,0.);assert(cross.newly_crossed&&!cross.active);
  // Gate is latched even when the ego reverses behind it while delay elapses.
  auto waiting=gate.update(102.20,-5.,0.);assert(waiting.gate_seen&&!waiting.active);
  auto active=gate.update(102.30,-5.,0.);
  assert(active.newly_active&&close(active.gate_time_s,102.));
  assert(close(active.activation_time_s,102.25));
  assert(close(active.active_dt_s,.05));
  state=advanceAstraGatedActor(state,cfg,2.,2.,0.,0.,active);
  assert(close(state.progress_m,.05));
  auto duplicate=gate.update(102.30,-5.,0.);assert(duplicate.active&&duplicate.active_dt_s==0.);
  auto fault=gate.update(102.29,-5.,0.);assert(!fault.valid);
  assert(!gate.update(103.,0.,0.).valid);

  AstraGateRelativeActivation zero(identity,-4.,0.);
  assert(!zero.update(5.,-4.1,0.).active);
  auto exact=zero.update(5.02,-4.,0.);
  assert(exact.active&&exact.newly_active&&exact.active_dt_s==0.);
  assert(close(zero.update(5.04,-3.99,0.).active_dt_s,.02));
  AstraGateRelativeActivation invalid(identity,-4.,-1.);
  assert(!invalid.update(0.,0.,0.).valid);
  AstraGateRelativeActivation nan(identity,-4.,0.);
  assert(!nan.update(0.,std::numeric_limits<double>::quiet_NaN(),0.).valid);

  // Robot spawn can legitimately occur long after plugin Load. Missing an
  // already sampled robot is a permanent fault, never a large catch-up step.
  AstraGateRelativeActivation spawning(identity,-4.,.2);
  assert(spawning.observeWithoutEgo(10.));
  assert(spawning.observeWithoutEgo(20.));
  assert(spawning.update(20.01,-5.4,0.).valid);
  assert(!spawning.observeWithoutEgo(20.02));
  assert(!spawning.update(30.,-3.,0.).valid);
  AstraGateRelativeActivation load_clock(identity,-4.,0.);
  assert(load_clock.observeWithoutEgo(10.));
  assert(!load_clock.update(9.,-5.,0.).valid);
  assert(!load_clock.update(11.,-3.,0.).valid);
  AstraGateRelativeActivation missing_clock(identity,-4.,0.);
  assert(missing_clock.observeWithoutEgo(10.));
  assert(!missing_clock.observeWithoutEgo(9.));

  // Global SE2 translation/rotation must not change route gate chronology.
  AstraRouteFrame transformed;transformed.origin_x_m=3.7;transformed.origin_y_m=-2.1;transformed.yaw_rad=1.17;
  AstraGateRelativeActivation plain(identity,-4.02,.6),rotated(transformed,-4.02,.6);
  for (int i=0;i<150;++i) {
    const double rx=-5.4+i*.02,ry=.13;
    const double wx=transformed.origin_x_m+std::cos(transformed.yaw_rad)*rx-std::sin(transformed.yaw_rad)*ry;
    const double wy=transformed.origin_y_m+std::sin(transformed.yaw_rad)*rx+std::cos(transformed.yaw_rad)*ry;
    assert(close(transformed.routeX(wx,wy),rx));
    const auto a=plain.update(i*.02,rx,ry),b=rotated.update(i*.02,wx,wy);
    assert(a.gate_seen==b.gate_seen&&a.active==b.active&&close(a.active_dt_s,b.active_dt_s));
  }

  // Active update is bit-for-bit the legacy rule across all three policies,
  // both response/non-response contexts, hesitant phases, and several seeds.
  int comparisons=0;
  for (auto policy:{V3ActorPolicy::kCautious,V3ActorPolicy::kAssertive,V3ActorPolicy::kHesitant}) {
    for (double ego_speed:{0.,.8,1.4}) {
      cfg.policy=policy;cfg.seed=263503375;cfg.base_speed_mps=.75;
      V3ReactiveActorState legacy,current;legacy.speed_mps=current.speed_mps=.75;
      for (int i=0;i<500;++i) {
        AstraGateStep step;step.active=true;step.active_dt_s=.002;step.interaction_time_s=i*.002;
        const double actor_distance=std::max(.01,2.-current.progress_m);
        const double ego_distance=std::max(.01,2.5-ego_speed*i*.002);
        legacy=advanceV3ReactiveActor(legacy,cfg,actor_distance,ego_distance,ego_speed,step.interaction_time_s,.002);
        current=advanceAstraGatedActor(current,cfg,actor_distance,ego_distance,ego_speed,0.,step);
        same(legacy,current);++comparisons;
        auto other_seed=cfg;other_seed.seed=7;
        same(advanceV3ReactiveActor(current,cfg,actor_distance,ego_distance,ego_speed,step.interaction_time_s,.002),
             advanceV3ReactiveActor(current,other_seed,actor_distance,ego_distance,ego_speed,step.interaction_time_s,.002));
      }
    }
  }
  std::cout<<"PASS: relative gate, delay latch, zero-delay boundary, no preactivation progress, partial-step clipping, duplicate time, clock fault including Load-to-first-update, first ego spawn allowed, ego disappearance permanently faults, nonfinite input, SE2 chronology, "<<comparisons<<" exact inherited-rule comparisons and unused-response-seed invariance\n";
}
