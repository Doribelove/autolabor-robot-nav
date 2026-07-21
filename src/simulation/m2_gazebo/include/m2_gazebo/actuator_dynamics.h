#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace m2_gazebo {

struct ActuatorLimits {
  double speed_time_constant_s{0.22};
  double steering_time_constant_s{0.18};
  double max_acceleration_mps2{1.20};
  double max_deceleration_mps2{1.60};
  double max_brake_deceleration_mps2{2.40};
  double max_emergency_deceleration_mps2{3.00};
  double max_steering_rate_radps{0.80};
};

struct ActuatorState {
  double speed_mps{0.0};
  double steering_rad{0.0};
};

inline double clampStep(double current, double desired, double maximum_step) {
  return current + std::max(-maximum_step, std::min(desired - current, maximum_step));
}

inline double firstOrderDesired(double current, double target, double dt,
                                double time_constant) {
  if (time_constant <= 0.0) return target;
  return current + (target - current) * (1.0 - std::exp(-dt / time_constant));
}

inline bool isAcceleration(double current, double target) {
  return current * target >= 0.0 && std::abs(target) > std::abs(current);
}

inline ActuatorState advanceActuators(
    const ActuatorState& current, double target_speed, double target_steering,
    bool brake_active, bool emergency_active, double dt,
    const ActuatorLimits& limits) {
  if (dt <= 0.0) return current;

  // A direction change is two physical phases: decelerate to zero, then
  // accelerate in the new gear.  This prevents an instantaneous F/R flip.
  double effective_speed_target = target_speed;
  if (current.speed_mps * target_speed < 0.0 && std::abs(current.speed_mps) > 1e-6) {
    effective_speed_target = 0.0;
  }
  if (brake_active || emergency_active) effective_speed_target = 0.0;

  double speed_rate = limits.max_deceleration_mps2;
  if (emergency_active) {
    speed_rate = limits.max_emergency_deceleration_mps2;
  } else if (brake_active) {
    speed_rate = limits.max_brake_deceleration_mps2;
  } else if (isAcceleration(current.speed_mps, effective_speed_target)) {
    speed_rate = limits.max_acceleration_mps2;
  }

  ActuatorState next;
  const double speed_desired = firstOrderDesired(
      current.speed_mps, effective_speed_target, dt,
      limits.speed_time_constant_s);
  next.speed_mps = clampStep(
      current.speed_mps, speed_desired, std::max(0.0, speed_rate) * dt);
  if ((current.speed_mps > 0.0 && next.speed_mps < 0.0) ||
      (current.speed_mps < 0.0 && next.speed_mps > 0.0) ||
      std::abs(next.speed_mps) < 1e-9) {
    next.speed_mps = 0.0;
  }

  const double steering_desired = firstOrderDesired(
      current.steering_rad, target_steering, dt,
      limits.steering_time_constant_s);
  next.steering_rad = clampStep(
      current.steering_rad, steering_desired,
      std::max(0.0, limits.max_steering_rate_radps) * dt);
  return next;
}

inline double stoppingDistance(double speed_mps, double latency_s,
                               double deceleration_mps2) {
  const double speed = std::abs(speed_mps);
  if (deceleration_mps2 <= 0.0) return INFINITY;
  return speed * std::max(0.0, latency_s) +
         speed * speed / (2.0 * deceleration_mps2);
}

// SplitMix64 gives a deterministic, platform-independent jitter sample.  It is
// deliberately stateless so reset + seed + sequence always reproduces a run.
inline double deterministicSignedJitter(std::uint64_t sequence,
                                        std::uint64_t seed,
                                        double amplitude_s) {
  if (amplitude_s <= 0.0) return 0.0;
  std::uint64_t value = sequence + seed + 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  value ^= value >> 31U;
  const double unit = static_cast<double>(value >> 11U) /
                      static_cast<double>(1ULL << 53U);
  return (2.0 * unit - 1.0) * amplitude_s;
}

inline bool validActuatorLimits(const ActuatorLimits& limits) {
  return limits.speed_time_constant_s > 0.0 &&
         limits.steering_time_constant_s > 0.0 &&
         limits.max_acceleration_mps2 > 0.0 &&
         limits.max_deceleration_mps2 > 0.0 &&
         limits.max_brake_deceleration_mps2 >= limits.max_deceleration_mps2 &&
         limits.max_emergency_deceleration_mps2 >= limits.max_brake_deceleration_mps2 &&
         limits.max_steering_rate_radps > 0.0;
}

}  // namespace m2_gazebo
