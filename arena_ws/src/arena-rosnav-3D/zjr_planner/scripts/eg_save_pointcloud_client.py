#!/usr/bin/env python3
import rospy
from zjr_planner.srv import SaveScan

def request_save_pointcloud(index=1,
                            scan_topic="scan",
                            width=100,
                            height=100,
                            resolution=0.1,
                            timeout=10.0):
    rospy.wait_for_service('/save_pointcloud')
    try:
        save_pointcloud = rospy.ServiceProxy('/save_pointcloud', SaveScan)
        resp = save_pointcloud(index, scan_topic, width, height, resolution, timeout)
        if resp.success:
            print(f"[INFO] Saved pointcloud to: {resp.message}")
        else:
            print(f"[ERROR] Failed to save pointcloud: {resp.message}")
    except rospy.ServiceException as e:
        print(f"[ERROR] Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('save_pointcloud_client', anonymous=True)
    # 调用一次服务
    request_save_pointcloud(index=1,
                            scan_topic="scan",
                            width=100,
                            height=100,
                            resolution=0.1,
                            timeout=10.0)
