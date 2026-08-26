#ifndef AUTOLABOR_CANBUS_DRIVER_M2_STATUS_POLLING_H
#define AUTOLABOR_CANBUS_DRIVER_M2_STATUS_POLLING_H

#include "autolabor_canbus_driver/Autocan.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace autolabor_driver {

inline std::array<uint8_t, 4> m2_safety_status_types() {
    return {{
        Autocan::Vcu::HardEmergency,
        Autocan::Vcu::SoftEmergency,
        Autocan::Vcu::GamepadEmergency,
        Autocan::Common::State,
    }};
}

class M2SafetyQueryRetryGate {
public:
    explicit M2SafetyQueryRetryGate(std::size_t max_attempts = 4)
        : max_attempts_(max_attempts > 0 ? max_attempts : 1) {}

    bool set_max_attempts(std::size_t max_attempts) {
        if (max_attempts == 0 || active_) {
            return false;
        }
        max_attempts_ = max_attempts;
        return true;
    }

    void begin(uint8_t msg_type) {
        active_ = true;
        pending_msg_type_ = msg_type;
        attempts_ = 0;
        reply_seen_ = false;
    }

    void record_attempt() {
        if (active_) {
            ++attempts_;
        }
    }

    bool observe(uint8_t msg_type) {
        if (!active_ || msg_type != pending_msg_type_) {
            return false;
        }
        reply_seen_ = true;
        return true;
    }

    bool active() const { return active_; }
    bool reply_seen() const { return reply_seen_; }
    uint8_t pending_msg_type() const { return pending_msg_type_; }
    std::size_t attempts() const { return attempts_; }
    std::size_t max_attempts() const { return max_attempts_; }

    bool should_advance() const {
        return active_ && (reply_seen_ || attempts_ >= max_attempts_);
    }

    bool exhausted() const {
        return active_ && !reply_seen_ && attempts_ >= max_attempts_;
    }

    void reset() {
        active_ = false;
        pending_msg_type_ = 0;
        attempts_ = 0;
        reply_seen_ = false;
    }

private:
    std::size_t max_attempts_;
    bool active_ = false;
    uint8_t pending_msg_type_ = 0;
    std::size_t attempts_ = 0;
    bool reply_seen_ = false;
};

// A fixed-period query timer can repeatedly land in the VCU's broadcast
// handling window.  Advance by the golden-ratio conjugate so successive
// intervals cover the configured jitter band without using nondeterministic
// randomness.  The long-run mean remains the requested query period.
class M2QueryIntervalSchedule {
public:
    bool configure(double query_rate_hz, double jitter_fraction) {
        if (!std::isfinite(query_rate_hz) || query_rate_hz <= 0.0 ||
                !std::isfinite(jitter_fraction) || jitter_fraction < 0.0 ||
                jitter_fraction > 0.45) {
            return false;
        }
        base_interval_sec_ = 1.0 / query_rate_hz;
        jitter_fraction_ = jitter_fraction;
        phase_ = 0.0;
        configured_ = true;
        return true;
    }

    double next_interval_sec() {
        if (!configured_) {
            return 0.0;
        }
        constexpr double golden_ratio_conjugate = 0.6180339887498948482;
        phase_ += golden_ratio_conjugate;
        if (phase_ >= 1.0) {
            phase_ -= 1.0;
        }
        const double signed_phase = 2.0 * phase_ - 1.0;
        return base_interval_sec_ *
            (1.0 + jitter_fraction_ * signed_phase);
    }

private:
    double base_interval_sec_ = 0.0;
    double jitter_fraction_ = 0.0;
    double phase_ = 0.0;
    bool configured_ = false;
};

// Convert a requested safety-snapshot rate into paced single-query timer ticks
// without exceeding the measured sustainable VCU reply rate.
inline double m2_status_query_rate_hz(double snapshot_rate_hz,
                                      std::size_t query_slots_per_snapshot,
                                      double transport_rate_limit_hz) {
    if (!std::isfinite(snapshot_rate_hz) || snapshot_rate_hz <= 0.0 ||
            query_slots_per_snapshot == 0 ||
            !std::isfinite(transport_rate_limit_hz) ||
            transport_rate_limit_hz <= 0.0) {
        return 0.0;
    }
    const double query_rate_hz =
        snapshot_rate_hz * static_cast<double>(query_slots_per_snapshot);
    return std::isfinite(query_rate_hz) && query_rate_hz > 0.0
        ? std::min(query_rate_hz, transport_rate_limit_hz) : 0.0;
}

}  // namespace autolabor_driver

#endif  // AUTOLABOR_CANBUS_DRIVER_M2_STATUS_POLLING_H
