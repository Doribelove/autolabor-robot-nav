#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest
from unittest import mock


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "wait_for_topics.py"
SPEC = importlib.util.spec_from_file_location("wait_for_topics", SCRIPT_PATH)
WAIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WAIT)


class TopicWaitTest(unittest.TestCase):
    def run_wait(self, responses, times=(0.0,)):
        with mock.patch.object(WAIT, "rospy") as ros, \
                mock.patch.object(WAIT.rosgraph, "Master") as master, \
                mock.patch.object(WAIT.time, "time", side_effect=times):
            params = {"~topics": "/gps/fix,/gps/pose,/gps/odom", "~timeout": 1.0,
                      "~poll_period": 0.2}
            ros.get_param.side_effect = lambda name, default: params.get(name, default)
            ros.get_name.return_value = "/wait_for_topics_test"
            ros.is_shutdown.return_value = False
            master.return_value.getPublishedTopics.side_effect = responses
            result = WAIT.main()
        return result, master, ros

    @staticmethod
    def published(*names):
        return [(name, "test/Message") for name in names]

    def test_multiple_topics_share_one_graph_query(self):
        result, master, _ = self.run_wait([
            self.published("/gps/fix", "/gps/pose", "/gps/odom")
        ])

        self.assertEqual(result, 0)
        master.assert_called_once_with("/wait_for_topics_test")
        master.return_value.getPublishedTopics.assert_called_once_with("")

    def test_master_retry_and_partial_readiness_are_preserved(self):
        result, master, ros = self.run_wait([
            RuntimeError("master temporarily unavailable"),
            self.published("/gps/fix"),
            self.published("/gps/pose", "/gps/odom"),
        ], times=(0.0, 0.1, 0.3))

        self.assertEqual(result, 0)
        self.assertEqual(master.return_value.getPublishedTopics.call_count, 3)
        self.assertEqual(ros.sleep.call_count, 2)

    def test_timeout_still_reports_only_missing_topics(self):
        result, _, ros = self.run_wait([
            self.published("/gps/fix", "/gps/pose")
        ], times=(0.0, 1.1))

        self.assertEqual(result, 3)
        ros.logerr.assert_called_once_with("Timed out waiting for topics: %s", "/gps/odom")


if __name__ == "__main__":
    unittest.main()
