#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import json
from datetime import datetime

import rospy
import rospkg
from std_msgs.msg import String


class ERIRecorder(object):
    """
    记录两类数据：
    1) /sderi/episode_summary → episode_summary.csv
    2) /sderi/transition       → transitions.csv （所有 episode 共用一个文件）
    """

    def __init__(self):
        rp = rospkg.RosPack()
        pkg_dir = rp.get_path("zjr_planner")

        # === 根目录: zjr_planner/eri_logs/run_<timestamp> ===
        run_ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.run_dir = os.path.join(pkg_dir, "eri_logs", f"run_{run_ts}")
        os.makedirs(self.run_dir, exist_ok=True)

        # === 1) SUMMARY CSV ===
        self.summary_path = os.path.join(self.run_dir, "episode_summary.csv")
        self.summary_fields = [
            "episode_id",
            "term_type",
            "time_sec",
            "steps",
            "collisions",
            "sum_reward",
        ]
        with open(self.summary_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.summary_fields)
            writer.writeheader()

        # === 2) TRANSITIONS CSV ===
        self.trans_path = os.path.join(self.run_dir, "transitions.csv")
        # 这里字段直接跟 eri_runtime_logger 里的 trans dict 对齐
        self.trans_fields = [
            "episode_id",
            "tau",
            "rho",
            "eri_rule",
            "eri_nn",
            "eri_act",
            "api_type",
            "alpha",
            "beta",
            "band",
            "acted_by",
            "dt",
            "progress",
            "collision",
            "reward",
            "done",
        ]
        with open(self.trans_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.trans_fields)
            writer.writeheader()

        # === ROS 订阅 ===
        rospy.Subscriber("/sderi/episode_summary", String, self.on_summary,    queue_size=100)
        rospy.Subscriber("/sderi/transition",       String, self.on_transition, queue_size=1000)
        # 如果你以前的 trainer 是用 /sderi/train/transitions，可以顺手也监听一下：
        # rospy.Subscriber("/sderi/train/transitions", String, self.on_transition, queue_size=1000)

        rospy.loginfo("[eri_recorder] summary     -> %s", self.summary_path)
        rospy.loginfo("[eri_recorder] transitions -> %s", self.trans_path)

    # ----------------------------------------------------
    # 1) episode_summary 回调
    # ----------------------------------------------------
    def on_summary(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("[eri_recorder] bad summary JSON: %s", e)
            return

        row = {k: data.get(k) for k in self.summary_fields}
        with open(self.summary_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.summary_fields)
            writer.writerow(row)

        rospy.loginfo(
            "[eri_recorder] summary ep=%s term_type=%s saved",
            data.get("episode_id"), data.get("term_type")
        )

    # ----------------------------------------------------
    # 2) transition 回调
    # ----------------------------------------------------
    def on_transition(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("[eri_recorder] bad transition JSON: %s", e)
            return

        row = {k: data.get(k) for k in self.trans_fields}
        with open(self.trans_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.trans_fields)
            writer.writerow(row)

        # 你如果觉得太吵可以关掉这行，只保留偶尔 debug 用：
        # rospy.loginfo("[eri_recorder] wrote one transition ep=%s", data.get("episode_id"))


if __name__ == "__main__":
    rospy.init_node("eri_recorder")
    ERIRecorder()
    rospy.spin()
