#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import stat
import sys

import rospy


def main():
    rospy.init_node("check_can", anonymous=True)
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    require_write = bool(rospy.get_param("~require_write", True))

    if not os.path.exists(port):
        rospy.logerr("CAN check failed: %s does not exist", port)
        return 2

    mode = os.stat(port).st_mode
    is_char = stat.S_ISCHR(mode)
    can_read = os.access(port, os.R_OK)
    can_write = os.access(port, os.W_OK)

    if not is_char:
        rospy.logerr("CAN check failed: %s is not a character device", port)
        return 3
    if not can_read:
        rospy.logerr("CAN check failed: %s is not readable", port)
        return 4
    if require_write and not can_write:
        rospy.logerr("CAN check failed: %s is not writable", port)
        return 5

    rospy.loginfo("CAN check passed: %s readable=%s writable=%s", port, can_read, can_write)
    return 0


if __name__ == "__main__":
    sys.exit(main())
