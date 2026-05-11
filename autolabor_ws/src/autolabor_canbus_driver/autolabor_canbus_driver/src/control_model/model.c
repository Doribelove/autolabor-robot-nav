//
// Created by ydrml on 2019/3/15.
//

#include <math.h>
#include "control_model/model.h"

struct wheels physical_to_wheels(
    const struct physical *physical,
    const struct chassis_config_t *config) {
    struct wheels result;

    if (physical->speed == 0) {
        // 对于舵轮来说是奇点，无法恢复
        result.left = 0;
        result.right = 0;

    } else if (physical->rudder > 0) {
        // 右转，左轮速度快
        float r = -config->length / tanf(physical->rudder);
        float k = (r + config->width / 2) / (r - config->width / 2);

        result.left = physical->speed;
        result.right = physical->speed * k;

    } else if (physical->rudder < 0) {
        // 左转，右轮速度快
        float r = -config->length / tanf(physical->rudder);
        float k = (r - config->width / 2) / (r + config->width / 2);

        result.left = physical->speed * k;
        result.right = physical->speed;

    } else {
        // 直走
        result.left = physical->speed;
        result.right = physical->speed;
    }

    return result;
}

struct physical wheels_to_physical(
    const struct wheels *wheels,
    const struct chassis_config_t *config) {
    struct physical result;

    float left = fabsf(wheels->left),
        right = fabsf(wheels->right);

    if (left == right) {
        // 绝对值相等（两条对角线）
        if (left == 0) {
            // 奇点
            result.speed = 0;
            result.rudder = NAN;

        } else if (wheels->left > wheels->right) {
            // 副对角线
            result.speed = left;
            result.rudder = +pi_f / 2;

        } else if (wheels->left < wheels->right) {
            // 副对角线
            result.speed = right;
            result.rudder = -pi_f / 2;

        } else {
            // 主对角线
            result.speed = wheels->left;
            result.rudder = 0;
        }

    } else {
        if (left > right) {
            // 右转，左轮速度快
            float k = wheels->right / wheels->left;
            float r = config->width / 2 * (k + 1) / (k - 1);

            result.speed = wheels->left;
            result.rudder = -atanf(config->length / r);

        } else {
            // 左转，右轮速度快
            float k = wheels->left / wheels->right;
            float r = config->width / 2 * (1 + k) / (1 - k);

            result.speed = wheels->right;
            result.rudder = -atanf(config->length / r);
        }
    }

    return result;
}

struct velocity physical_to_velocity(
    const struct physical *physical,
    const struct chassis_config_t *config) {
    struct wheels temp = physical_to_wheels(physical, config);
    return wheels_to_velocity(&temp, config);
}

struct physical velocity_to_physical(
    const struct velocity *velocity,
    const struct chassis_config_t *config) {
    struct wheels temp = velocity_to_wheels(velocity, config);
    return wheels_to_physical(&temp, config);
}

struct wheels velocity_to_wheels(
    const struct velocity *velocity,
    const struct chassis_config_t *config) {
    struct wheels result = {
        (velocity->v - config->width / 2 * velocity->w) / config->radius,
        (velocity->v + config->width / 2 * velocity->w) / config->radius
    };
    return result;
}

struct velocity wheels_to_velocity(
    const struct wheels *wheels,
    const struct chassis_config_t *config) {
    struct velocity result = {
        config->radius * (wheels->right + wheels->left) / 2,
        config->radius * (wheels->right - wheels->left) / config->width
    };
    return result;
}

void legalize_physical(struct physical *data, float max_wheel_speed) {
    data->speed *= fminf(1, max_wheel_speed / fabsf(data->speed));
}

void legalize_wheels(struct wheels *data, float max_wheel_speed) {
    float ratio = fminf(1, fminf(max_wheel_speed / fabsf(data->left),
                                 max_wheel_speed / fabsf(data->right)));
    data->left *= ratio;
    data->right *= ratio;
}

void legalize_velocity(struct velocity *data,
                       const struct chassis_config_t *chassis,
                       float max_wheel_speed) {
    struct physical temp = velocity_to_physical(data, chassis);
    float ratio = fminf(1, max_wheel_speed / fabsf(temp.speed));
    data->v *= ratio;
    data->w *= ratio;
}
