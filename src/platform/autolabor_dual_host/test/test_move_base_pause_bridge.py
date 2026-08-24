#!/usr/bin/env python3

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "move_base_pause_bridge_under_test",
    str(PACKAGE_ROOT / "scripts" / "move_base_pause_bridge.py"),
)
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class MoveBasePauseBridgeTest(unittest.TestCase):
    @staticmethod
    def _bridge():
        bridge = BRIDGE.MoveBasePauseBridge.__new__(BRIDGE.MoveBasePauseBridge)
        bridge.lock = threading.RLock()
        bridge.paused = False
        bridge.coverage_active = False
        bridge.retained_pose = None
        bridge.retained_goal_id = ""
        bridge.reissue_on_resume = True
        bridge.simple_goal_request_topic = "/move_base_simple/goal"
        bridge.simple_goal_output_topic = "/navigation_goal/accepted"
        bridge.goal_pub = _Publisher()
        bridge.cancel_pub = _Publisher()
        bridge.paused_pub = _Publisher()
        bridge.status_pub = _Publisher()
        return bridge

    def test_simple_goal_is_forwarded_only_without_pause_or_coverage(self):
        bridge = self._bridge()
        request = PoseStamped()
        request.header.frame_id = "map"
        request.pose.position.x = 2.0
        bridge._simple_goal_callback(request)
        self.assertEqual(1, len(bridge.goal_pub.messages))
        self.assertEqual(2.0, bridge.goal_pub.messages[0].pose.position.x)

        bridge.paused = True
        bridge._simple_goal_callback(request)
        self.assertEqual(1, len(bridge.goal_pub.messages))

        bridge.paused = False
        bridge.coverage_active = True
        bridge._simple_goal_callback(request)
        self.assertEqual(1, len(bridge.goal_pub.messages))
        self.assertIn("coverage owns move_base", bridge.status_pub.messages[-1].data)

    def test_coverage_clears_and_does_not_retain_segment_endpoint(self):
        bridge = self._bridge()
        ordinary = PoseStamped()
        ordinary.pose.position.x = 1.0
        bridge.retained_pose = ordinary
        bridge.retained_goal_id = "ordinary-goal"
        bridge._coverage_callback(Bool(data=True))
        self.assertIsNone(bridge.retained_pose)
        self.assertEqual("", bridge.retained_goal_id)

        action_goal = SimpleNamespace(
            goal=SimpleNamespace(target_pose=PoseStamped()),
            goal_id=SimpleNamespace(id="coverage-segment"),
        )
        bridge._goal_callback(action_goal)
        self.assertIsNone(bridge.retained_pose)
        self.assertEqual("", bridge.retained_goal_id)


if __name__ == "__main__":
    unittest.main()
