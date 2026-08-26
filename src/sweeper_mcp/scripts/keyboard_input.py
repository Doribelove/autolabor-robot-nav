#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘输入节点 —— 模拟语音识别结果。

从标准输入逐行读取文本,发布到 /voice/text (std_msgs/String)。
这就是未来语音识别模块要替换的接口点:
只要语音识别节点同样向 /voice/text 发布 std_msgs/String,下游任务分解节点零改动。

运行: rosrun sweeper_mcp keyboard_input.py
"""

import sys

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("keyboard_input", anonymous=True)
    topic = rospy.get_param("~topic", "/voice/text")
    pub = rospy.Publisher(topic, String, queue_size=10)

    # 等待 publisher 与下游订阅者连接建立,避免首条消息丢失。
    rospy.sleep(0.3)

    rospy.loginfo("键盘输入节点已启动,输入文本回车发送(话题: %s),Ctrl+D 退出。", topic)
    print(">>> 请输入清扫指令(回车发送, Ctrl+D 退出):", flush=True)

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            try:
                line = input("> ")
            except EOFError:
                break
            except KeyboardInterrupt:
                break

            line = line.strip()
            if not line:
                continue
            if rospy.is_shutdown():
                break
            pub.publish(String(line))
            rospy.loginfo("已发送: %s", line)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("键盘输入节点退出。")


if __name__ == "__main__":
    main()
