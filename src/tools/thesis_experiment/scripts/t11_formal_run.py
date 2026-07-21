#!/usr/bin/env python3
"""Run one frozen T11 training seed or safety-ablation evaluation."""

import sys

import rospy
import yaml

from thesis_experiment.t11_training import T11FormalRunner


def main():
    rospy.init_node("t11_formal_run", anonymous=False)
    report = T11FormalRunner().run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
