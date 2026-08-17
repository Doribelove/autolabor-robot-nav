# autolabor_canbus_driver/CanBusMessage Message
## Raw Message Definition
~~~
uint8 node_type  # 节点类型
uint8 node_seq   # 节点序号
uint8 msg_type   # CAN消息类型
uint8[] payload  # CAN消息内容(指令数据时为空)
~~~

## Compact Message Definition
~~~
uint8 node_type
uint8 node_seq
uint8 msg_type
uint8[] payload
~~~
