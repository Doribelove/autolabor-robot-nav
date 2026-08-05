#!/usr/bin/env python3

from pathlib import Path
import os
import unittest
import xml.etree.ElementTree as ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
GUI_SOURCE = (PACKAGE_ROOT / "src" / "main_window.cpp").read_text(encoding="utf-8")
GUI_HEADER = (
    PACKAGE_ROOT / "include" / "autolabor_operator_gui" / "main_window.h"
).read_text(encoding="utf-8")
RVIZ_CONFIG = (PACKAGE_ROOT / "config" / "operator_navigation.rviz").read_text(
    encoding="utf-8"
)
ALL_IN_ONE = WORKSPACE_ROOT / "scripts" / "operator_all_in_one.sh"
ALL_IN_ONE_TEXT = ALL_IN_ONE.read_text(encoding="utf-8")
FAST_LIO_ALL_IN_ONE = (
    WORKSPACE_ROOT / "scripts" / "operator_fast_lio_all_in_one.sh"
)
FAST_LIO_ALL_IN_ONE_TEXT = FAST_LIO_ALL_IN_ONE.read_text(encoding="utf-8")


class OperatorGuiContractTest(unittest.TestCase):
    def test_embedded_rviz_has_2d_navigation_goal_tool(self):
        tools = RVIZ_CONFIG[RVIZ_CONFIG.index("  Tools:") :]
        self.assertIn("- Class: rviz/SetGoal", tools)
        set_goal = tools[tools.index("- Class: rviz/SetGoal") :]
        self.assertIn("Topic: /move_base_simple/goal", set_goal.split("Value:", 1)[0])
        self.assertIn("Fixed Frame: camera_init", RVIZ_CONFIG)

    def test_heading_and_manual_wgs84_goal_are_visible(self):
        self.assertIn('node_->subscribe("/gps/heading"', GUI_SOURCE)
        self.assertIn("北 0°，顺时针", GUI_SOURCE)
        self.assertIn("发送 GPS 目标点", GUI_SOURCE)
        self.assertIn('advertise<sensor_msgs::NavSatFix>("/gps/goal_fix"', GUI_SOURCE)
        self.assertIn("sendManualGpsGoal", GUI_HEADER)

    def test_localization_odometry_topic_is_selectable_per_mode(self):
        launch_root = ElementTree.parse(
            str(PACKAGE_ROOT / "launch" / "operator_gui.launch")
        ).getroot()
        launch_args = {
            element.attrib["name"]: element.attrib.get("default")
            for element in launch_root.findall("arg")
        }
        self.assertEqual("GPS", launch_args["navigation_mode_label"])
        self.assertEqual("/gps/odom", launch_args["odom_topic"])
        self.assertIn(
            'node_->param<std::string>("odom_topic", odom_topic_, "/gps/odom")',
            GUI_SOURCE,
        )
        self.assertIn("node_->subscribe(odom_topic_", GUI_SOURCE)
        self.assertNotIn('node_->subscribe("/gps/odom"', GUI_SOURCE)
        self.assertIn("Topic: /Odometry", RVIZ_CONFIG)

    def test_master_probe_commits_state_before_initializing_rviz(self):
        handler = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::handleMasterProbeFinished()") :
            GUI_SOURCE.index("void MainWindow::setupRosInterfaces()")
        ]
        self.assertLess(
            handler.index("previous_probe_online_ = result.online;"),
            handler.index("setupRosInterfaces();"),
        )

    def test_camera_yolo_and_runtime_imaging_controls_are_integrated(self):
        for topic in (
            "/fod_camera/image_raw",
            "/fod/debug/image",
            "/fod/detections",
        ):
            self.assertIn(topic, GUI_SOURCE)
        self.assertIn("dynamic_reconfigure::Reconfigure", GUI_SOURCE)
        self.assertIn('"/zed2/zed_node/set_parameters"', GUI_SOURCE)
        self.assertIn('"/zed2/zed_node/parameter_updates"', GUI_SOURCE)
        self.assertNotIn("hikrobot_mvs_camera", GUI_SOURCE)
        self.assertIn("立即单独启动", GUI_SOURCE)

    def test_visual_motion_uses_safe_mode_arbiter_only(self):
        self.assertIn(
            '"/fod_navigation_mode/set_fod_enabled"',
            GUI_SOURCE,
        )
        self.assertNotIn('advertise<geometry_msgs::Twist>', GUI_SOURCE)
        self.assertNotIn('"/fod_visual_servo/set_enabled"', GUI_SOURCE)
        self.assertNotIn('advertise<', GUI_SOURCE.split("setupRosInterfaces", 1)[0])
        self.assertIn("小于 5 m", GUI_SOURCE)
        self.assertIn("连续 1 秒", GUI_SOURCE)
        self.assertIn("直行 0.5 m", GUI_SOURCE)

    def test_manifest_declares_new_ros_interfaces(self):
        root = ElementTree.parse(str(PACKAGE_ROOT / "package.xml")).getroot()
        dependencies = {element.text for element in root.findall("depend")}
        self.assertTrue(
            {"autolabor_fod_msgs", "diagnostic_msgs", "dynamic_reconfigure"}
            <= dependencies
        )

    def test_all_in_one_opens_gui_before_navigation_readiness(self):
        self.assertTrue(os.access(str(ALL_IN_ONE), os.X_OK))
        self.assertIn("TERMINAL_MODE=same NAV_START_RVIZ=false", ALL_IN_ONE_TEXT)
        bringup = ALL_IN_ONE_TEXT.index('"$SCRIPT_DIR/bringup.sh" gps')
        vision = ALL_IN_ONE_TEXT.index(
            "roslaunch autolabor_fod_vision zed_fod_detection.launch"
        )
        rabbitmq = ALL_IN_ONE_TEXT.index('"$SCRIPT_DIR/rabbitmq_gps_goal_bridge.py"')
        gui = ALL_IN_ONE_TEXT.index('"$SCRIPT_DIR/operator_gui.sh"')
        readiness_wait = ALL_IN_ONE_TEXT.index(
            "waiting for the complete navigation readiness gate in the background"
        )
        self.assertLess(bringup, vision)
        self.assertLess(bringup, rabbitmq)
        self.assertLess(bringup, gui)
        self.assertLess(vision, readiness_wait)
        self.assertLess(rabbitmq, readiness_wait)
        self.assertLess(gui, readiness_wait)
        before_bringup = ALL_IN_ONE_TEXT[:bringup]
        self.assertIn("if ! ensure_ros_master; then", before_bringup)
        readiness_loop = ALL_IN_ONE_TEXT[readiness_wait:]
        self.assertIn('process_is_running "$GUI_PID"', readiness_loop)

    def test_all_in_one_keeps_degraded_gui_after_navigation_failure(self):
        self.assertIn("OPERATOR_START_VISION", ALL_IN_ONE_TEXT)
        self.assertIn("OPERATOR_START_RABBITMQ", ALL_IN_ONE_TEXT)
        self.assertIn("continuing in degraded-console mode", ALL_IN_ONE_TEXT)
        self.assertIn("ensure_ros_master", ALL_IN_ONE_TEXT)
        failure_start = ALL_IN_ONE_TEXT.index(
            "WARNING: $NAV_DISPLAY_NAME bringup exited before it became ready"
        )
        failure_end = ALL_IN_ONE_TEXT.index("    break", failure_start)
        failure_branch = ALL_IN_ONE_TEXT[failure_start:failure_end]
        self.assertNotIn('exit "$bringup_status"', failure_branch)
        self.assertIn("ensure_ros_master", failure_branch)

    def test_fast_lio_all_in_one_selects_odometry_and_safe_speed(self):
        self.assertTrue(os.access(str(FAST_LIO_ALL_IN_ONE), os.X_OK))
        self.assertIn("OPERATOR_NAV_MODE=fast_lio", FAST_LIO_ALL_IN_ONE_TEXT)
        self.assertIn(
            'exec "$SCRIPT_DIR/operator_all_in_one.sh" "$@"',
            FAST_LIO_ALL_IN_ONE_TEXT,
        )
        self.assertIn('"$SCRIPT_DIR/bringup.sh" fast_lio', ALL_IN_ONE_TEXT)
        self.assertIn('ODOM_TOPIC="/Odometry"', ALL_IN_ONE_TEXT)
        self.assertIn(
            'navigation_mode_label:="$NAV_DISPLAY_NAME"',
            ALL_IN_ONE_TEXT,
        )
        self.assertIn('FAST_LIO_NAV_MAX_VEL_X="$NAV_MAX_SPEED"', ALL_IN_ONE_TEXT)
        self.assertIn("start_gps_error_monitor:=false", ALL_IN_ONE_TEXT)


if __name__ == "__main__":
    unittest.main()
