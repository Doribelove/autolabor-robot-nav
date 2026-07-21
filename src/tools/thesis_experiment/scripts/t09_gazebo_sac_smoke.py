#!/usr/bin/env python3
"""Run the Semantic-Eta side of the shared T09/T10 Gazebo SAC smoke."""

import sys

import rospy
import yaml

from thesis_experiment.sac_gazebo_smoke import GazeboSacSmoke


def main():
    rospy.init_node("t09_gazebo_sac_smoke", anonymous=False)
    report = GazeboSacSmoke("T09").run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
