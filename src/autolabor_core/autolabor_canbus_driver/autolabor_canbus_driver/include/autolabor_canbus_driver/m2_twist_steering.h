#ifndef AUTOLABOR_CANBUS_DRIVER_M2_TWIST_STEERING_H
#define AUTOLABOR_CANBUS_DRIVER_M2_TWIST_STEERING_H

#include <cmath>

namespace autolabor_driver {

inline bool m2_twist_command_is_valid(double linear_velocity,
                                      double angular_velocity) {
    return std::isfinite(linear_velocity) &&
           std::isfinite(angular_velocity);
}

inline bool m2_chassis_control_parameters_are_valid(
    double max_speed,
    double max_steering_angle,
    double wheelbase) {
    return std::isfinite(max_speed) && max_speed > 0.0 &&
           std::isfinite(max_steering_angle) && max_steering_angle > 0.0 &&
           std::isfinite(wheelbase) && wheelbase > 0.0;
}

inline bool m2_chassis_parameters_are_valid(double max_speed,
                                            double max_steering_angle,
                                            double robot_width,
                                            double wheelbase,
                                            double wheel_radius) {
    return m2_chassis_control_parameters_are_valid(
               max_speed, max_steering_angle, wheelbase) &&
           std::isfinite(robot_width) &&
           std::isfinite(wheel_radius);
}

struct M2SafeMotionControl {
    double relative_velocity;
    double steering_angle;
    bool input_valid;
    bool was_limited;
};

// Final CAN-boundary defense.  This helper is deliberately independent of the
// upstream Twist conversion so future callers cannot serialize NaN/Inf or an
// out-of-range normalized velocity into a MotionCtrl frame.
inline M2SafeMotionControl sanitize_m2_motion_control(
    double relative_velocity,
    double steering_angle,
    double max_steering_angle) {
    if (!std::isfinite(relative_velocity) ||
        !std::isfinite(steering_angle) ||
        !std::isfinite(max_steering_angle) ||
        max_steering_angle <= 0.0) {
        return M2SafeMotionControl{0.0, 0.0, false, false};
    }

    double safe_velocity = relative_velocity;
    if (safe_velocity > 1.0) safe_velocity = 1.0;
    if (safe_velocity < -1.0) safe_velocity = -1.0;

    double safe_steering = steering_angle;
    if (safe_steering > max_steering_angle) {
        safe_steering = max_steering_angle;
    }
    if (safe_steering < -max_steering_angle) {
        safe_steering = -max_steering_angle;
    }

    return M2SafeMotionControl{
        safe_velocity,
        safe_steering,
        true,
        safe_velocity != relative_velocity || safe_steering != steering_angle};
}

// Keep an Ackermann Twist's curvature (omega / v) unchanged when an upstream
// command exceeds the chassis linear-speed limit.  The usual M2 clamp keeps
// the sign of v, so the scale factor is positive for both forward and reverse
// commands.  At zero requested speed there is no curvature to preserve; keep
// omega so the existing steering pre-position behavior remains available.
inline double angular_velocity_after_linear_limit(
    double requested_linear_velocity,
    double limited_linear_velocity,
    double requested_angular_velocity,
    double zero_linear_epsilon = 1e-12) {
    if (!std::isfinite(requested_linear_velocity) ||
        !std::isfinite(limited_linear_velocity) ||
        !std::isfinite(requested_angular_velocity) ||
        !std::isfinite(zero_linear_epsilon) ||
        zero_linear_epsilon < 0.0) {
        return 0.0;
    }

    if (std::abs(requested_linear_velocity) <= zero_linear_epsilon) {
        return requested_angular_velocity;
    }

    const double scale =
        limited_linear_velocity / requested_linear_velocity;
    if (std::abs(scale) < 1.0) {
        return requested_angular_velocity * scale;
    }
    return requested_angular_velocity;
}

// Convert a standard base-frame Twist into the front steering angle used by
// the M2 bicycle model: omega = v * tan(steering) / wheelbase.
//
// At effectively zero speed an Ackermann chassis cannot realize a yaw rate.
// Preserve the driver's existing convention of pre-positioning the steering
// at its limit in the requested yaw direction.
inline double twist_to_front_steering(double linear_velocity,
                                      double angular_velocity,
                                      double wheelbase,
                                      double max_steering_angle,
                                      double zero_linear_epsilon = 1e-4,
                                      double zero_angular_epsilon = 1e-2) {
    // A malformed command or chassis geometry must never put NaN/Inf into a
    // CAN motion-control frame.  Reject negative geometry/threshold values as
    // configuration errors instead of silently changing their meaning.
    if (!std::isfinite(linear_velocity) ||
        !std::isfinite(angular_velocity) ||
        !std::isfinite(wheelbase) ||
        !std::isfinite(max_steering_angle) ||
        !std::isfinite(zero_linear_epsilon) ||
        !std::isfinite(zero_angular_epsilon) ||
        wheelbase <= 0.0 || max_steering_angle <= 0.0 ||
        zero_linear_epsilon < 0.0 || zero_angular_epsilon < 0.0) {
        return 0.0;
    }

    const double steering_limit = max_steering_angle;
    const double linear_epsilon = zero_linear_epsilon;
    const double angular_epsilon = zero_angular_epsilon;

    if (std::abs(linear_velocity) <= linear_epsilon) {
        if (angular_velocity > angular_epsilon) return steering_limit;
        if (angular_velocity < -angular_epsilon) return -steering_limit;
        return 0.0;
    }

    const double steering =
        std::atan(angular_velocity * wheelbase / linear_velocity);
    if (steering > steering_limit) return steering_limit;
    if (steering < -steering_limit) return -steering_limit;
    return steering;
}

}  // namespace autolabor_driver

#endif  // AUTOLABOR_CANBUS_DRIVER_M2_TWIST_STEERING_H
