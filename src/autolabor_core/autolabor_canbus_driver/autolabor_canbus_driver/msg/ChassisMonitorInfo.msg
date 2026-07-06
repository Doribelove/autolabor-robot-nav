# ChassisMonitorStatus.msg
# 描述车辆控制单元（VCU）的监控状态

# TCU状态
uint8 tcu_state
uint8 tcu_timeout
uint8 tcu_stuck

# 左ECU状态
uint8 lecu_state
uint8 lecu_timeout
uint8 lecu_stuck
uint8 lecu_brake

# 右ECU状态
uint8 recu_state
uint8 recu_timeout
uint8 recu_stuck
uint8 recu_brake