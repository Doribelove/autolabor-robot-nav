#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace m2_gazebo {

enum class V3ActorPolicy {
  kCautious,
  kAssertive,
  kHesitant,
};

struct V3ReactiveActorConfig {
  V3ActorPolicy policy{V3ActorPolicy::kCautious};
  std::uint32_t seed{0U};
  double base_speed_mps{1.0};
  double cautious_speed_mps{0.12};
  double assertive_speed_mps{1.35};
  double hesitant_speed_mps{0.32};
  double response_distance_m{4.0};
  double response_time_margin_s{1.35};
  double acceleration_mps2{1.8};
  double deceleration_mps2{2.4};
};

struct V3ReactiveActorState {
  double speed_mps{0.0};
  double progress_m{0.0};
  bool responding{false};
  bool ego_approaching{false};
  double ego_ttc_s{INFINITY};
  double actor_ttc_s{INFINITY};
};

inline bool parseV3ActorPolicy(
    const std::string& value, V3ActorPolicy* policy) {
  if (policy == nullptr) return false;
  if (value == "cautious") {
    *policy = V3ActorPolicy::kCautious;
    return true;
  }
  if (value == "assertive") {
    *policy = V3ActorPolicy::kAssertive;
    return true;
  }
  if (value == "hesitant") {
    *policy = V3ActorPolicy::kHesitant;
    return true;
  }
  return false;
}

inline bool validV3ReactiveActorConfig(const V3ReactiveActorConfig& config) {
  return std::isfinite(config.base_speed_mps) &&
         std::isfinite(config.cautious_speed_mps) &&
         std::isfinite(config.assertive_speed_mps) &&
         std::isfinite(config.hesitant_speed_mps) &&
         std::isfinite(config.response_distance_m) &&
         std::isfinite(config.response_time_margin_s) &&
         std::isfinite(config.acceleration_mps2) &&
         std::isfinite(config.deceleration_mps2) &&
         config.base_speed_mps > 0.0 &&
         config.cautious_speed_mps >= 0.0 &&
         config.cautious_speed_mps < config.base_speed_mps &&
         config.assertive_speed_mps > config.base_speed_mps &&
         config.hesitant_speed_mps >= 0.0 &&
         config.hesitant_speed_mps < config.base_speed_mps &&
         config.response_distance_m > 0.0 &&
         config.response_time_margin_s > 0.0 &&
         config.acceleration_mps2 > 0.0 &&
         config.deceleration_mps2 > 0.0;
}

inline double v3SignedSeedJitter(std::uint32_t seed) {
  std::uint32_t value = seed ^ 0x9e3779b9U;
  value ^= value >> 16U;
  value *= 0x7feb352dU;
  value ^= value >> 15U;
  value *= 0x846ca68bU;
  value ^= value >> 16U;
  return 2.0 * (static_cast<double>(value % 10001U) / 10000.0) - 1.0;
}

inline bool v3ResponseEnabled(double sim_time_s,
                              double response_enable_time_s) {
  return std::isfinite(sim_time_s) &&
      std::isfinite(response_enable_time_s) &&
      response_enable_time_s >= 0.0 &&
      sim_time_s + 1e-12 >= response_enable_time_s;
}

inline V3ReactiveActorState advanceV3ReactiveActor(
    const V3ReactiveActorState& current,
    const V3ReactiveActorConfig& config,
    double actor_distance_to_conflict_m,
    double ego_distance_to_conflict_m,
    double ego_closing_speed_mps,
    double sim_time_s,
    double dt_s) {
  V3ReactiveActorState next = current;
  if (!validV3ReactiveActorConfig(config) ||
      !std::isfinite(actor_distance_to_conflict_m) ||
      !std::isfinite(ego_distance_to_conflict_m) ||
      !std::isfinite(ego_closing_speed_mps) ||
      !std::isfinite(sim_time_s) || !std::isfinite(dt_s) || dt_s <= 0.0) {
    return next;
  }

  // V3-01-R2 removes the uncalibrated threshold jitter.  Hidden policy
  // diversity remains in the policy dynamics; stochastic actor parameters
  // require a later, separately calibrated stage.
  const double jitter = 0.0 * v3SignedSeedJitter(config.seed);
  const double response_margin =
      config.response_time_margin_s * (1.0 + jitter);
  next.ego_approaching =
      ego_closing_speed_mps > 0.05 &&
      ego_distance_to_conflict_m <= config.response_distance_m;
  next.ego_ttc_s = next.ego_approaching
      ? ego_distance_to_conflict_m / ego_closing_speed_mps
      : INFINITY;
  next.actor_ttc_s = actor_distance_to_conflict_m > 0.0
      ? actor_distance_to_conflict_m /
          std::max(config.base_speed_mps, 1e-6)
      : 0.0;
  next.responding = next.ego_approaching &&
      actor_distance_to_conflict_m > 0.0 &&
      std::abs(next.ego_ttc_s - next.actor_ttc_s) <= response_margin;

  double target_speed = config.base_speed_mps;
  if (next.responding) {
    if (config.policy == V3ActorPolicy::kCautious) {
      target_speed = config.cautious_speed_mps;
    } else if (config.policy == V3ActorPolicy::kAssertive) {
      target_speed = config.assertive_speed_mps;
    } else {
      const int phase =
          static_cast<int>(std::floor(std::max(0.0, sim_time_s) / 0.55));
      target_speed =
          (phase % 2 == 0) ? config.hesitant_speed_mps
                           : 0.78 * config.base_speed_mps;
    }
  }

  const double delta = target_speed - current.speed_mps;
  const double limit = (delta >= 0.0 ? config.acceleration_mps2
                                     : config.deceleration_mps2) * dt_s;
  next.speed_mps =
      current.speed_mps + std::max(-limit, std::min(limit, delta));
  next.speed_mps = std::max(0.0, next.speed_mps);
  next.progress_m =
      current.progress_m + 0.5 * (current.speed_mps + next.speed_mps) * dt_s;
  return next;
}

}  // namespace m2_gazebo
