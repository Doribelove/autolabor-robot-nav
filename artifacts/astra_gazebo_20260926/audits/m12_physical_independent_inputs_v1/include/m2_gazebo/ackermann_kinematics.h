#pragma once

#include <algorithm>
#include <cmath>

namespace m2_gazebo {

struct SteeringGeometry {
  double center{0.0};
  double left{0.0};
  double right{0.0};
  double yaw_rate{0.0};
};

inline double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

// Convert a body-twist command to the bicycle-model center steering angle.
// Dividing by signed velocity is intentional: a negative velocity requires the
// opposite steering sign to retain the requested body yaw-rate sign.
inline double twistToSteering(double velocity, double yaw_rate,
                              double wheelbase, double max_steer,
                              double velocity_epsilon = 1e-3) {
  if (std::abs(velocity) < velocity_epsilon || std::abs(yaw_rate) < 1e-9) {
    return 0.0;
  }
  return clamp(std::atan(wheelbase * yaw_rate / velocity),
               -max_steer, max_steer);
}

inline SteeringGeometry steeringGeometry(double center, double velocity,
                                          double wheelbase, double track,
                                          double max_steer) {
  SteeringGeometry result;
  result.center = clamp(center, -max_steer, max_steer);
  if (std::abs(result.center) < 1e-9) {
    return result;
  }

  const double curvature = std::tan(result.center) / wheelbase;
  const double radius = 1.0 / std::abs(curvature);
  const double inner_denominator = std::max(1e-6, radius - track / 2.0);
  const double outer_denominator = radius + track / 2.0;
  const double inner = std::atan(wheelbase / inner_denominator);
  const double outer = std::atan(wheelbase / outer_denominator);
  if (result.center > 0.0) {
    result.left = inner;
    result.right = outer;
  } else {
    result.left = -outer;
    result.right = -inner;
  }
  result.yaw_rate = velocity * curvature;
  return result;
}

inline void rearWheelLinearVelocities(double center_velocity, double yaw_rate,
                                      double track, double* left,
                                      double* right) {
  *left = center_velocity - yaw_rate * track / 2.0;
  *right = center_velocity + yaw_rate * track / 2.0;
}

}  // namespace m2_gazebo
