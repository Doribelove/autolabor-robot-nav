//
// Created by ydrml on 2019/3/20.
//

#include <math.h>
#include "control_model/optimization.h"

struct physical optimize(const struct physical *target,
                         const struct physical *current,
                         float optimize_width,
                         float acceleration) {
	struct physical result = {0, current->rudder};
	// 等待后轮转动
	if (!isnan(target->rudder))
		result.speed = target->speed * fmaxf(0, 1 - fabsf(target->rudder - current->rudder) / optimize_width);
	// 动态调速
	result.speed     = result.speed > current->speed
	                   ? fminf(current->speed + acceleration, result.speed)
	                   : fmaxf(current->speed - acceleration, result.speed);
	return result;
}
