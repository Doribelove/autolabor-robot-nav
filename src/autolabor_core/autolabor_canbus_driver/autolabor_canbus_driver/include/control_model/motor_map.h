//
// Created by ydrml on 2019/3/20.
//

#ifndef PM1_SDK_MOTOR_MAP_H
#define PM1_SDK_MOTOR_MAP_H

#include "model.h"
#include "pi.h"

#define CALCULATE_K(RESOLUTION, RATIO) (4 * (RESOLUTION) * (RATIO))

#define RAD_OF(PULSES, K) ((PULSES) * (K))
#define PULSES_OF(RAD, K) ((int)((RAD) / (K)))

const float default_wheel_k = static_cast<const float>(2 * M_PI / CALCULATE_K(400, 20));
const float default_rudder_k = static_cast<const float>(2 * M_PI / 16384);

#endif //PM1_SDK_MOTOR_MAP_H
