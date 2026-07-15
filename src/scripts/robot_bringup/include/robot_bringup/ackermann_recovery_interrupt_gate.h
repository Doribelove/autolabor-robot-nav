#ifndef ROBOT_BRINGUP_ACKERMANN_RECOVERY_INTERRUPT_GATE_H
#define ROBOT_BRINGUP_ACKERMANN_RECOVERY_INTERRUPT_GATE_H

#include <cmath>

namespace robot_bringup
{
namespace recovery_detail
{

/**
 * One-shot hand-off for an interrupt received immediately before a recovery
 * run becomes active.
 *
 * The owner provides monotonic seconds and serializes calls. A pending event
 * is consumed by the next run even when it has expired, so an old goal/cancel
 * can never latch and reject every later recovery attempt.
 */
class PendingInterruptGate
{
public:
  void record(double monotonic_now)
  {
    pending_ = true;
    recorded_at_ = monotonic_now;
  }

  bool consumeIfFresh(double monotonic_now, double maximum_age)
  {
    if (!pending_)
    {
      return false;
    }

    pending_ = false;
    if (!std::isfinite(monotonic_now) || !std::isfinite(recorded_at_) ||
        !std::isfinite(maximum_age) || maximum_age < 0.0)
    {
      return false;
    }

    // A monotonic clock should not move backwards. If it nevertheless does,
    // fail safe for this one run instead of losing a newly received cancel.
    const double age = monotonic_now - recorded_at_;
    return age <= maximum_age;
  }

private:
  bool pending_{false};
  double recorded_at_{0.0};
};

}  // namespace recovery_detail
}  // namespace robot_bringup

#endif  // ROBOT_BRINGUP_ACKERMANN_RECOVERY_INTERRUPT_GATE_H
