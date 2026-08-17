//
// Created by ydrml on 2019/3/14.
//

#ifndef PM1_SDK_CHASSIS_CONFIG_T_H
#define PM1_SDK_CHASSIS_CONFIG_T_H

struct chassis_config_t {
	float width,  // 轮距
	      length, // 轴距
	      radius; // 轮半径
};

extern const float                   pi_f;
extern const struct chassis_config_t default_config;


#endif // PM1_SDK_CHASSIS_CONFIG_T_H
