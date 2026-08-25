#ifndef AUTOLABOR_CANBUS_DRIVER_M2_STATUS_POLLING_H
#define AUTOLABOR_CANBUS_DRIVER_M2_STATUS_POLLING_H

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace autolabor_driver {

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
