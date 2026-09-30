#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <m2_gazebo/ackermann_kinematics.h>

namespace m2_gazebo {

inline std::array<double, 4> physicalWheelSpeeds(
    double vx, const SteeringGeometry& steer, double wheelbase,
    double track, double radius) {
  const double left_x = vx - steer.yaw_rate * track / 2.0;
  const double right_x = vx + steer.yaw_rate * track / 2.0;
  const double front_y = steer.yaw_rate * wheelbase;
  return {{(left_x*std::cos(steer.left)+front_y*std::sin(steer.left))/radius,
           (right_x*std::cos(steer.right)+front_y*std::sin(steer.right))/radius,
           left_x/radius, right_x/radius}};
}

inline double actualCenterSteering(double left, double right,
                                   double wheelbase, double track) {
  const double tl=std::tan(left), tr=std::tan(right);
  const double dl=wheelbase+track*tl/2.0, dr=wheelbase-track*tr/2.0;
  if (!std::isfinite(tl) || !std::isfinite(tr) ||
      std::abs(dl)<1e-9 || std::abs(dr)<1e-9) return NAN;
  return std::atan(wheelbase*(tl/dl+tr/dr)/2.0);
}

struct BoundedPI {
  double kp{4.0};
  double ki{2.0};
  double integral_limit{1.5};  // Nm contribution, not an unbounded error sum.
  double integral{0.0};

  double update(double target, double measured, double feedforward,
                double dt, double torque_limit) {
    if (!std::isfinite(target) || !std::isfinite(measured) ||
        !std::isfinite(feedforward) || !std::isfinite(dt) || dt<=0.0 ||
        !std::isfinite(torque_limit) || torque_limit<=0.0) return 0.0;
    const double error=target-measured;
    const double candidate=clamp(integral+ki*error*dt,-integral_limit,integral_limit);
    const double unsaturated=feedforward+kp*error+candidate;
    // Conditional integration permits unwinding but not saturation wind-up.
    if (std::abs(unsaturated)<=torque_limit || unsaturated*error<=0.0)
      integral=candidate;
    return clamp(feedforward+kp*error+integral,-torque_limit,torque_limit);
  }
};

inline double boundedSteeringTorque(double reference, double reference_rate,
    double actual, double actual_rate, double kp, double kd, double limit) {
  if (!std::isfinite(reference) || !std::isfinite(reference_rate) ||
      !std::isfinite(actual) || !std::isfinite(actual_rate)) return 0.0;
  return clamp(kp*(reference-actual)+kd*(reference_rate-actual_rate),-limit,limit);
}

// Brake/timeout/emergency/fault never track a residual positive speed target.
// With valid positive kp/limit, torque * actual_rate <= 0 at every update;
// a stopped wheel receives exactly zero effort, irrespective of PI history.
inline double passiveWheelBrake(double actual_rate, double kp, double limit) {
  if (!std::isfinite(actual_rate) || !std::isfinite(kp) || kp<=0.0 ||
      !std::isfinite(limit) || limit<=0.0) return 0.0;
  return clamp(-kp*actual_rate,-limit,limit);
}
}  // namespace m2_gazebo
