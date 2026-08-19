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
FAST_LIO_ALL_IN_ONE = WORKSPACE_ROOT / "scripts" / "operator_fast_lio_all_in_one.sh"
FAST_LIO_ALL_IN_ONE_TEXT = FAST_LIO_ALL_IN_ONE.read_text(encoding="utf-8")
NVIDIA_UI = WORKSPACE_ROOT / "scripts" / "nvidia_ui.sh"
NVIDIA_UI_TEXT = NVIDIA_UI.read_text(encoding="utf-8")
RECORD_ROSBAG = WORKSPACE_ROOT / "scripts" / "record_rosbag.sh"
RECORD_ROSBAG_TEXT = RECORD_ROSBAG.read_text(encoding="utf-8")
MAPPING_SESSION = WORKSPACE_ROOT / "scripts" / "global_mapping_session.sh"
MAPPING_SESSION_TEXT = MAPPING_SESSION.read_text(encoding="utf-8")
BUILD_GLOBAL_MAP = WORKSPACE_ROOT / "scripts" / "build_global_map.sh"
VIEW_GLOBAL_MAP = WORKSPACE_ROOT / "scripts" / "view_global_map.sh"


class OperatorGuiContractTest(unittest.TestCase):
    def test_record_button_runs_synchronized_bag_and_static_mapping(self):
        self.assertTrue(RECORD_ROSBAG.is_file())
        self.assertTrue(os.access(str(RECORD_ROSBAG), os.X_OK))
        self.assertTrue(MAPPING_SESSION.is_file())
        self.assertTrue(os.access(str(MAPPING_SESSION), os.X_OK))
        self.assertIn('scripts/global_mapping_session.sh', GUI_SOURCE)
        self.assertIn('record_rosbag.sh" mode1', MAPPING_SESSION_TEXT)
        self.assertIn('fused_scan_mapper.py', MAPPING_SESSION_TEXT)
        self.assertIn('GLOBAL_MAPPING_LATEST=', MAPPING_SESSION_TEXT)
        self.assertIn('MAPPING_REQUIRE_DUAL_LIDAR', MAPPING_SESSION_TEXT)
        for topic in (
            "/tf",
            "/tf_static",
            "/livox/lidar",
            "/livox/imu",
            "/cloud_registered_body",
            "/Odometry",
            "/mid360/scan",
            "/dual_lidar/scan",
            "/scan",
            "/avoidance/dual_lidar_active",
        ):
            self.assertIn(topic, RECORD_ROSBAG_TEXT)
        self.assertIn('$ROBOT_WS/rosbags', RECORD_ROSBAG_TEXT)

    def test_offline_mapping_tools_have_fixed_workspace_storage(self):
        self.assertTrue(BUILD_GLOBAL_MAP.is_file())
        self.assertTrue(VIEW_GLOBAL_MAP.is_file())
        self.assertTrue(os.access(str(BUILD_GLOBAL_MAP), os.X_OK))
        self.assertTrue(os.access(str(VIEW_GLOBAL_MAP), os.X_OK))
        mapping = BUILD_GLOBAL_MAP.read_text(encoding="utf-8")
        self.assertIn('BAG_ROOT="$ROBOT_WS/rosbags"', mapping)
        self.assertIn('MAP_ROOT="$ROBOT_WS/global_maps"', mapping)
        self.assertIn('ROS_MASTER_URI="http://127.0.0.1:$ROS_PORT"', mapping)
        self.assertIn('/livox/lidar /livox/imu', mapping)
        self.assertIn('global_map_raw.pcd', mapping)
        self.assertIn('global_map.pcd', mapping)

    def test_embedded_rviz_has_local_navigation_goal_tool(self):
        tools = RVIZ_CONFIG[RVIZ_CONFIG.index("  Tools:") :]
        self.assertIn("- Class: rviz/SetGoal", tools)
        set_goal = tools[tools.index("- Class: rviz/SetGoal") :]
        self.assertIn("Topic: /move_base_simple/goal", set_goal.split("Value:", 1)[0])
        self.assertIn("- Class: rviz/SetInitialPose", tools)
        set_initial_pose = tools[tools.index("- Class: rviz/SetInitialPose") :]
        self.assertIn("Topic: /initialpose", set_initial_pose.split("Value:", 1)[0])
        self.assertIn("Fixed Frame: map", RVIZ_CONFIG)
        self.assertIn("Name: Static global map", RVIZ_CONFIG)
        self.assertIn("Topic: /map", RVIZ_CONFIG)

    def test_launch_defaults_to_fast_lio_streams(self):
        launch_root = ElementTree.parse(
            str(PACKAGE_ROOT / "launch" / "operator_gui.launch")
        ).getroot()
        launch_args = {
            element.attrib["name"]: element.attrib.get("default")
            for element in launch_root.findall("arg")
        }
        self.assertEqual("FAST_LIO", launch_args["navigation_mode_label"])
        self.assertEqual("/Odometry", launch_args["odom_topic"])
        self.assertEqual("/cloud_registered_body", launch_args["cloud_topic"])
        self.assertEqual("/livox/imu", launch_args["imu_topic"])
        self.assertNotIn("start_gps_error_monitor", launch_args)
        self.assertNotIn("geofence_config_file", launch_args)
        for parameter in ("odom_topic", "cloud_topic", "imu_topic"):
            self.assertIn(f'node_->param<std::string>("{parameter}"', GUI_SOURCE)

    def test_rviz_initializes_directly_in_navigation_frame(self):
        launch_root = ElementTree.parse(
            str(PACKAGE_ROOT / "launch" / "operator_gui.launch")
        ).getroot()
        launch_args = {
            element.attrib["name"]: element.attrib.get("default")
            for element in launch_root.findall("arg")
        }
        self.assertEqual("map", launch_args["rviz_startup_fixed_frame"])
        self.assertEqual("map", launch_args["rviz_navigation_fixed_frame"])
        self.assertIn(
            '"rviz_startup_fixed_frame", rviz_startup_fixed_frame_, "map"',
            GUI_SOURCE,
        )
        self.assertIn("rviz_fixed_frame=map", NVIDIA_UI_TEXT)

    def test_fast_lio_health_checks_live_chain_and_stability(self):
        for callback in ("odomCallback", "cloudCallback", "imuCallback"):
            self.assertIn(callback, GUI_HEADER)
        health = GUI_SOURCE[
            GUI_SOURCE.index("MainWindow::FastLioHealthResult") :
            GUI_SOURCE.index("void MainWindow::refreshUi()")
        ]
        for evidence in (
            "kFastLioFreshOdomSeconds",
            "kFastLioFreshCloudSeconds",
            "kFastLioFreshImuSeconds",
            "odom_rate_hz",
            "cloud_rate_hz",
            "imu_rate_hz",
            "recent_pose_step_m",
            "stationary_drift_m",
            "pose.covariance",
            'canTransform(fixed_frame, "base_link"',
        ):
            self.assertIn(evidence, health)
        for label in (
            "综合健康结论",
            "FAST-LIO 数据链",
            "连续性与静止漂移",
            "内部位置 σxy",
            "判定依据",
        ):
            self.assertIn(label, GUI_SOURCE)

    def test_relative_goal_replaces_wgs84_goal(self):
        self.assertIn(
            'advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal"',
            GUI_SOURCE,
        )
        self.assertIn("发送相对目标", GUI_SOURCE)
        self.assertIn("relative_forward_input_", GUI_HEADER)
        self.assertIn("std::cos(current_yaw) * forward_m", GUI_SOURCE)
        self.assertIn("std::sin(current_yaw) * left_m", GUI_SOURCE)
        self.assertIn("relativeGoalReady", GUI_SOURCE)
        self.assertNotIn("sensor_msgs::NavSatFix", GUI_SOURCE + GUI_HEADER)
        self.assertNotIn("/gps/goal_fix", GUI_SOURCE + GUI_HEADER)
        self.assertNotIn("发送 GPS 目标", GUI_SOURCE)

    def test_gps_and_rabbit_pages_are_removed(self):
        self.assertIn('buildFastLioPage(), QStringLiteral("FAST-LIO")', GUI_SOURCE)
        self.assertNotIn("buildGpsPage", GUI_SOURCE + GUI_HEADER)
        self.assertNotIn("buildRabbitPage", GUI_SOURCE + GUI_HEADER)
        self.assertNotIn("RabbitMQ", GUI_SOURCE + GUI_HEADER)
        self.assertNotIn("rabbitmq_bridge", GUI_SOURCE + GUI_HEADER)
        self.assertNotIn("autolabor_operator_msgs", (
            PACKAGE_ROOT / "CMakeLists.txt"
        ).read_text(encoding="utf-8"))

    def test_rabbitmq_runtime_module_is_deleted(self):
        self.assertFalse((WORKSPACE_ROOT / "scripts" / "rabbitmq_gps_goal_bridge.py").exists())
        self.assertFalse(
            (WORKSPACE_ROOT / "src" / "scripts" / "robot_bringup" / "scripts" /
             "fod_cloud_pose_reporter.py").exists()
        )
        self.assertFalse(
            (WORKSPACE_ROOT / "src" / "application" / "autolabor_operator_msgs" /
             "package.xml").exists()
        )
        for text in (ALL_IN_ONE_TEXT, NVIDIA_UI_TEXT):
            self.assertNotIn("RABBITMQ", text.upper())
            self.assertNotIn("fod_cloud_pose_reporter", text)

    def test_covariance_is_not_presented_as_ground_truth(self):
        self.assertIn("不等同于相对测量真值的绝对误差", GUI_SOURCE)
        self.assertIn("内部协方差", GUI_SOURCE)

    def test_rviz_shows_enhanced_mid360_cloud(self):
        self.assertIn("Class: rviz/PointCloud2", RVIZ_CONFIG)
        self.assertIn("Topic: /cloud_registered_body_enhanced", RVIZ_CONFIG)
        cloud_display = RVIZ_CONFIG[
            RVIZ_CONFIG.index("Class: rviz/PointCloud2") :
            RVIZ_CONFIG.index("Class: rviz/Path")
        ]
        self.assertIn("Enabled: true", cloud_display)
        self.assertIn("Value: true", cloud_display)

    def test_global_theme_does_not_overpaint_rviz_render_panel(self):
        self.assertIn("#include <rviz/render_panel.h>", GUI_SOURCE)
        self.assertNotIn("QMainWindow, QWidget { background:", GUI_SOURCE)
        self.assertIn("QMainWindow { background: #101721; }", GUI_SOURCE)
        self.assertIn('QWidget { color: #e7edf5; font-family:', GUI_SOURCE)
        setup = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::setupEmbeddedRviz()") :
            GUI_SOURCE.index("void MainWindow::toggleRvizPanels()")
        ]
        self.assertIn("render_panel->setAutoFillBackground(false);", setup)
        self.assertIn(
            "render_panel->setAttribute(Qt::WA_StyledBackground, false);", setup
        )

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
        for topic in ("/fod_camera/image_raw", "/fod/debug/image", "/fod/detections"):
            self.assertIn(topic, GUI_SOURCE)
        self.assertIn("dynamic_reconfigure::Reconfigure", GUI_SOURCE)
        self.assertIn('"/zed2/zed_node/set_parameters"', GUI_SOURCE)
        self.assertIn('"/zed2/zed_node/parameter_updates"', GUI_SOURCE)
        self.assertIn("立即单独启动", GUI_SOURCE)

    def test_visual_motion_uses_safe_mode_arbiter_only(self):
        self.assertIn('"/fod_navigation_mode/set_fod_enabled"', GUI_SOURCE)
        self.assertNotIn("advertise<geometry_msgs::Twist>", GUI_SOURCE)
        self.assertNotIn('"/fod_visual_servo/set_enabled"', GUI_SOURCE)
        self.assertIn("小于 5 m", GUI_SOURCE)
        self.assertIn("连续 1 秒", GUI_SOURCE)
        self.assertIn("直行 0.5 m", GUI_SOURCE)

    def test_manifest_declares_health_interfaces_without_remote_messages(self):
        root = ElementTree.parse(str(PACKAGE_ROOT / "package.xml")).getroot()
        dependencies = {element.text for element in root.findall("depend")}
        self.assertTrue(
            {"autolabor_fod_msgs", "diagnostic_msgs", "dynamic_reconfigure",
             "geometry_msgs", "tf2_ros"} <= dependencies
        )
        self.assertNotIn("autolabor_operator_msgs", dependencies)

    def test_all_in_one_opens_gui_before_navigation_readiness(self):
        self.assertTrue(os.access(str(ALL_IN_ONE), os.X_OK))
        bringup = ALL_IN_ONE_TEXT.index('"$SCRIPT_DIR/bringup.sh" gps')
        vision = ALL_IN_ONE_TEXT.index(
            "roslaunch autolabor_fod_vision zed_fod_detection.launch"
        )
        gui = ALL_IN_ONE_TEXT.index('"$SCRIPT_DIR/operator_gui.sh"')
        readiness_wait = ALL_IN_ONE_TEXT.index(
            "waiting for the complete navigation readiness gate in the background"
        )
        self.assertLess(bringup, vision)
        self.assertLess(bringup, gui)
        self.assertLess(vision, readiness_wait)
        self.assertLess(gui, readiness_wait)
        self.assertIn("ensure_ros_master", ALL_IN_ONE_TEXT[:bringup])
        self.assertIn(
            'process_is_running "$GUI_PID"', ALL_IN_ONE_TEXT[readiness_wait:]
        )

    def test_fast_lio_all_in_one_selects_odometry_and_safe_speed(self):
        self.assertTrue(os.access(str(FAST_LIO_ALL_IN_ONE), os.X_OK))
        self.assertIn("OPERATOR_NAV_MODE=fast_lio", FAST_LIO_ALL_IN_ONE_TEXT)
        self.assertIn('ODOM_TOPIC="/Odometry"', ALL_IN_ONE_TEXT)
        self.assertIn('FAST_LIO_NAV_MAX_VEL_X="$NAV_MAX_SPEED"', ALL_IN_ONE_TEXT)
        self.assertIn("cloud_topic:=/cloud_registered_body", ALL_IN_ONE_TEXT)
        self.assertIn("imu_topic:=/livox/imu", ALL_IN_ONE_TEXT)


if __name__ == "__main__":
    unittest.main()
