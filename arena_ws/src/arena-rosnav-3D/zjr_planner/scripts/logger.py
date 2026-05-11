#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, time, signal
import rospy
from std_msgs.msg import Float32, Int32, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped

class SderiLogger:
    def __init__(self):
        # --- params (with sensible defaults) ---
        self.save_root = rospy.get_param("~save_root",
            os.path.join(os.path.dirname(__file__), "data/ros_data"))
        self.run_tag   = rospy.get_param("~run_tag",
            time.strftime("%Y%m%d_%H%M%S"))
        self.world     = rospy.get_param("~world", "unknown_world")
        self.scenario  = rospy.get_param("~scenario_file", "unknown.json")
        self.epoch     = int(rospy.get_param("~epoch", 0))

        # file & dir
        self.run_dir = os.path.join(self.save_root, self.run_tag)
        os.makedirs(self.run_dir, exist_ok=True)
        self.path_jsonl = os.path.join(self.run_dir, "samples.jsonl")
        self.fp = open(self.path_jsonl, "a", buffering=1)  # line-buffered

        # latest values (updated by callbacks)
        self.features   = None      # [tau, rho]
        self.eri_label  = None      # float
        self.band_idx   = None      # int
        self.goal_xy    = None      # optional; fill if you publish it
        self.start_xy   = None      # optional
        self.step       = 0

        # --- subs ---
        rospy.Subscriber("/sderi/features_debug", Float32MultiArray, self.cb_features, queue_size=50)
        rospy.Subscriber("/sderi/eri_debug",      Float32,           self.cb_eri,      queue_size=50)
        rospy.Subscriber("/sderi/band_idx_debug", Int32,             self.cb_band,     queue_size=50)
        rospy.Subscriber("/sderi/subgoal_debug",  PoseStamped,       self.cb_subgoal,  queue_size=50)
        # if you have topics for start/goal, add them here and set self.start_xy / self.goal_xy

        # graceful shutdown
        signal.signal(signal.SIGINT, self._shutdown)
        signal.signal(signal.SIGTERM, self._shutdown)
        rospy.loginfo(f"[sderi_logger] Writing to {self.path_jsonl}")

    # --- callbacks ---
    def cb_features(self, msg: Float32MultiArray):
        # ensure list of floats
        try:
            self.features = [float(v) for v in msg.data]
        except Exception as e:
            rospy.logwarn(f"[sderi_logger] bad features: {e}")

    def cb_eri(self, msg: Float32):
        self.eri_label = float(msg.data)

    def cb_band(self, msg: Int32):
        self.band_idx = int(msg.data)

    def cb_subgoal(self, msg: PoseStamped):
        # write only when we have necessary signals
        if self.features is None or self.eri_label is None:
            return
        row = {
            "t": rospy.Time.now().to_sec(),
            "step": self.step,
            "epoch": self.epoch,
            "world": self.world,
            "scenario_file": self.scenario,
            "features": self.features,         # [tau, rho]
            "eri_label": self.eri_label,       # teacher ERI (currently rule/blend)
            "subgoal": {
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y)
            },
            "band_idx": self.band_idx,
            "goal": self.goal_xy,              # optional
            "start": self.start_xy             # optional
        }
        try:
            self.fp.write(json.dumps(row) + "\n")
            self.step += 1
        except Exception as e:
            rospy.logerr(f"[sderi_logger] write error: {e}")

    def _shutdown(self, *args):
        try:
            self.fp.flush()
            self.fp.close()
        except Exception:
            pass
        rospy.signal_shutdown("logger stop")

if __name__ == "__main__":
    rospy.init_node("sderi_logger", anonymous=False)
    SderiLogger()
    rospy.spin()
