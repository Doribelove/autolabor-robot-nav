#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <m2_gazebo/v3_reactive_actor.h>

namespace m2_gazebo {

struct AstraRouteFrame {
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  double yaw_rad{0.0};

  bool valid() const {
    return std::isfinite(origin_x_m) && std::isfinite(origin_y_m) &&
           std::isfinite(yaw_rad);
  }
  double routeX(double world_x_m, double world_y_m) const {
    return std::cos(yaw_rad) * (world_x_m - origin_x_m) +
           std::sin(yaw_rad) * (world_y_m - origin_y_m);
  }
};

struct AstraGateStep {
  bool valid{true};
  bool gate_seen{false};
  bool active{false};
  bool newly_crossed{false};
  bool newly_active{false};
  double gate_time_s{std::numeric_limits<double>::quiet_NaN()};
  double activation_time_s{std::numeric_limits<double>::quiet_NaN()};
  double active_dt_s{0.0};
  double interaction_time_s{0.0};
};

// The first observed route-x crossing is latched. There is no interpolation
// from potentially stale poses and no wall-clock or absolute-time activation.
// A clock regression or nonfinite sample permanently faults this instance.
class AstraGateRelativeActivation {
 public:
  AstraGateRelativeActivation(const AstraRouteFrame& route,
                              double gate_route_x_m, double delay_s)
      : route_(route), gate_route_x_m_(gate_route_x_m), delay_s_(delay_s) {
    valid_ = route_.valid() && std::isfinite(gate_route_x_m_) &&
             std::isfinite(delay_s_) && delay_s_ >= 0.0;
  }

  AstraGateStep update(double now_s, double world_x_m, double world_y_m) {
    AstraGateStep result;
    if (!valid_ || !std::isfinite(now_s) || !std::isfinite(world_x_m) ||
        !std::isfinite(world_y_m) || (have_time_ && now_s < last_time_s_)) {
      valid_ = false;
      result.valid = false;
      return result;
    }
    const double previous_s = have_time_ ? last_time_s_ : now_s;
    last_time_s_ = now_s;
    have_time_ = true;
    have_ego_sample_ = true;
    if (!gate_seen_ && route_.routeX(world_x_m, world_y_m) >= gate_route_x_m_) {
      gate_seen_ = true;
      gate_time_s_ = now_s;
      result.newly_crossed = true;
    }
    result.gate_seen = gate_seen_;
    if (!gate_seen_) return result;
    result.gate_time_s = gate_time_s_;
    result.activation_time_s = gate_time_s_ + delay_s_;
    result.active = now_s + 1e-12 >= result.activation_time_s;
    result.newly_active = result.active && !active_;
    active_ = result.active;
    if (result.active) {
      // Do not integrate any part of a physics step before activation.
      result.active_dt_s = std::max(0.0, now_s -
          std::max(previous_s, result.activation_time_s));
      result.interaction_time_s = std::max(0.0, now_s - result.activation_time_s);
    }
    return result;
  }

  // Prime this at plugin Load, then keep time anchored while the robot has
  // not spawned yet. Losing an already observed ego invalidates the actor;
  // no unobserved interval may later be integrated as one large motion step.
  bool observeWithoutEgo(double now_s) {
    if (!valid_ || have_ego_sample_ || !std::isfinite(now_s) ||
        (have_time_ && now_s < last_time_s_)) {
      valid_ = false;
      return false;
    }
    last_time_s_ = now_s;
    have_time_ = true;
    return true;
  }

  void invalidate() { valid_ = false; }

 private:
  AstraRouteFrame route_;
  double gate_route_x_m_;
  double delay_s_;
  double gate_time_s_{0.0};
  double last_time_s_{0.0};
  bool valid_{true};
  bool have_time_{false};
  bool have_ego_sample_{false};
  bool gate_seen_{false};
  bool active_{false};
};

// All reactive policy mathematics are inherited exactly. Only the interval
// over which physical progress is permitted is new common-environment logic.
inline V3ReactiveActorState advanceAstraGatedActor(
    const V3ReactiveActorState& current,
    const V3ReactiveActorConfig& config,
    double actor_distance_m, double ego_distance_m,
    double ego_closing_speed_mps, double time_offset_s,
    const AstraGateStep& gate) {
  if (!gate.valid || !gate.active || gate.active_dt_s <= 0.0) return current;
  return advanceV3ReactiveActor(current, config, actor_distance_m,
      ego_distance_m, ego_closing_speed_mps,
      time_offset_s + gate.interaction_time_s, gate.active_dt_s);
}

}  // namespace m2_gazebo
