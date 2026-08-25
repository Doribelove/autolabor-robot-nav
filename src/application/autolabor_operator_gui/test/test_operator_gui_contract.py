#!/usr/bin/env python3

from pathlib import Path
import os
import re
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
GUI_SOURCE = (PACKAGE_ROOT / "src" / "main_window.cpp").read_text(encoding="utf-8")
GUI_HEADER = (
    PACKAGE_ROOT / "include" / "autolabor_operator_gui" / "main_window.h"
).read_text(encoding="utf-8")
RVIZ_CONFIG = (PACKAGE_ROOT / "config" / "operator_navigation.rviz").read_text(
    encoding="utf-8"
)
COVERAGE_RVIZ_CONFIG = (
    PACKAGE_ROOT / "config" / "coverage_navigation.rviz"
).read_text(encoding="utf-8")
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
    def test_coverage_page_requires_static_map_and_supports_polygon_editing(self):
        self.assertIn("buildCoveragePage()", GUI_SOURCE)
        self.assertIn("框定覆盖清扫范围", GUI_SOURCE)
        self.assertIn("确认区域并生成轨迹", GUI_SOURCE)
        self.assertIn("取消框定", GUI_SOURCE)
        self.assertIn("static_map_mode_ && data.map_received", GUI_SOURCE)
        self.assertIn('"/coverage/clicked_point"', GUI_SOURCE)
        self.assertIn('"/coverage/plan"', GUI_SOURCE)
        self.assertIn('"/coverage/start"', GUI_SOURCE)
        self.assertIn('"/coverage/set_paused"', GUI_SOURCE)
        self.assertIn('"/coverage/cancel"', GUI_SOURCE)
        self.assertIn("最高前进速度", GUI_SOURCE)
        self.assertIn("coverage_width_input_->setValue(1.00)", GUI_SOURCE)
        self.assertIn("coverage_speed_input_->setRange(0.10, 1.60)", GUI_SOURCE)
        self.assertIn("coverage_speed_input_->setValue(0.80)", GUI_SOURCE)
        self.assertIn("class ScrollSafeDoubleSpinBox", GUI_SOURCE)
        self.assertIn("event->ignore()", GUI_SOURCE)
        self.assertIn("call.request.max_speed_mps = max_speed_mps", GUI_SOURCE)
        self.assertIn("当前车辆位姿", GUI_SOURCE)
        self.assertIn("最近位置里程计", GUI_SOURCE)
        self.assertIn("recent_odom_distance_m", GUI_SOURCE)
        self.assertIn('lookupTransform("map", "base_link"', GUI_SOURCE)
        self.assertIn(
            "QWidget#coverageControls QLabel, QWidget#coverageControls QCheckBox { color: #111827; }",
            GUI_SOURCE,
        )
        self.assertIn("coverage_editable && !coverage_selecting", GUI_SOURCE)

    def test_coverage_cancel_is_available_through_the_whole_lifecycle(self):
        refresh = GUI_SOURCE[
            GUI_SOURCE.index("const bool coverage_has_ready_plan") :
            GUI_SOURCE.index("bool MainWindow::relativeGoalReady")
        ]
        for evidence in (
            "coverage_plan_pending_",
            'coverage.state == "PLANNING"',
            "coverage_has_ready_plan",
            "coverage_start_preparing",
            "coverage_active",
            "coverage_cancel_available",
        ):
            self.assertIn(evidence, refresh)

        cancel = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::cancelCoverageTask()") :
            GUI_SOURCE.index("void MainWindow::callSetBoolService")
        ]
        self.assertNotIn(
            "if (coverage_cancel_pending_ || coverage_plan_id_.empty())", cancel
        )
        self.assertIn("coverage_plan_pending_", cancel)
        self.assertIn("active_at_request", cancel)
        self.assertIn("preparing_at_request", cancel)

    def test_coverage_cancel_has_an_independent_pending_guard(self):
        self.assertIn("bool coverage_command_pending_ = false;", GUI_HEADER)
        self.assertIn("bool coverage_cancel_pending_ = false;", GUI_HEADER)
        cancel = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::cancelCoverageTask()") :
            GUI_SOURCE.index("void MainWindow::callSetBoolService")
        ]
        self.assertIn("if (coverage_cancel_pending_", cancel)
        self.assertIn("coverage_cancel_pending_ = true;", cancel)
        self.assertIn("coverage_cancel_pending_ = false;", cancel)
        self.assertNotIn("if (coverage_command_pending_)\n    return;", cancel)

    def test_coverage_terminal_state_resets_ui_and_allows_second_selection(self):
        terminal = GUI_SOURCE[
            GUI_SOURCE.index("const bool coverage_terminal") :
            GUI_SOURCE.index('values_["coverage_map"]->setText')
        ]
        for state in ("COMPLETED", "COMPLETED_PARTIAL", "CANCELED", "FAILED"):
            self.assertIn('coverage.state == "{}"'.format(state), terminal)
        self.assertIn("resetCoverageUiState(true);", terminal)

        reset = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::resetCoverageUiState") :
            GUI_SOURCE.index("void MainWindow::beginCoverageSelection")
        ]
        for evidence in (
            "coverage_selecting_ = false;",
            "coverage_draft_points_.clear();",
            "coverage_plan_id_.clear();",
            "publishCoverageDraft();",
            "selectCoveragePointTool(false);",
        ):
            self.assertIn(evidence, reset)

        begin = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::beginCoverageSelection") :
            GUI_SOURCE.index("void MainWindow::undoCoveragePoint")
        ]
        for evidence in (
            "coverage_selecting_ = true;",
            "coverage_draft_points_.clear();",
            "coverage_cancel_requested_ = false;",
            "selectCoveragePointTool(true);",
        ):
            self.assertIn(evidence, begin)
        self.assertIn(
            "coverage_editable && !coverage_selecting &&\n"
            "                                      !coverage_has_ready_plan",
            GUI_SOURCE,
        )

    def test_returning_to_coverage_tab_restores_publish_point_selection(self):
        attach = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::attachRvizToTab") :
            GUI_SOURCE.index("void MainWindow::publishMapDisplayStatus")
        ]
        for evidence in (
            "resume_coverage_selection = coverage_selecting_",
            "resume_coverage_selection ? QStringLiteral(\"rviz/PublishPoint\")",
            ': QStringLiteral("rviz/MoveCamera")',
        ):
            self.assertIn(evidence, attach)

    def test_canceled_plan_callback_cannot_revive_draft_or_plan_id(self):
        generation_match = re.search(
            r"std::uint64_t (coverage_plan_[a-z_]*"
            r"(?:generation|epoch|revision)[a-z_]*)\s*=\s*0;",
            GUI_HEADER,
        )
        self.assertIsNotNone(
            generation_match,
            "coverage planning futures need a monotonically invalidated generation",
        )
        generation = generation_match.group(1)

        reset = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::resetCoverageUiState") :
            GUI_SOURCE.index("void MainWindow::beginCoverageSelection")
        ]
        self.assertIn("++{};".format(generation), reset)

        confirm = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::confirmCoverageSelection") :
            GUI_SOURCE.index("void MainWindow::startCoverage")
        ]
        request_match = re.search(
            r"const std::uint64_t (\w+)\s*=\s*(?:\+\+)?{};".format(
                re.escape(generation)
            ),
            confirm,
        )
        self.assertIsNotNone(
            request_match,
            "each planning future must capture the generation it belongs to",
        )
        request_generation = request_match.group(1)
        callback = confirm[confirm.index("connect(watcher") :]
        self.assertRegex(
            callback,
            r"\[this,\s*watcher,\s*{}\]".format(
                re.escape(request_generation)
            ),
        )
        stale_guard = "{} != {}".format(request_generation, generation)
        self.assertIn(stale_guard, callback)
        guard_at = callback.index(stale_guard)
        self.assertLess(guard_at, callback.index("coverage_plan_pending_ = false;"))
        self.assertLess(guard_at, callback.index("coverage_plan_id_ = result.plan_id;"))
        self.assertLess(guard_at, callback.index("coverage_selecting_ = true;"))

    def test_coverage_status_distinguishes_navigation_from_cleaning_hardware(self):
        for evidence in (
            "覆盖任务状态",
            "覆盖导航状态",
            "覆盖路线执行中",
            "路线约束",
            "运动学核对",
            "底盘执行门",
            "障碍感知",
            "已覆盖估算",
            "未接入 · 仅执行覆盖导航",
            "coverage.kinematics_verified",
            "coverage.lane_spacing_m",
            "coverage.required_steering_angle_rad",
            "coverage.chassis_ready",
            "coverage.chassis_detail",
            "coverage.avoidance_ready",
            "coverage.avoidance_detail",
        ):
            self.assertIn(evidence, GUI_SOURCE)
        self.assertNotIn(
            '{ QStringLiteral("SWEEPING"), QStringLiteral("覆盖清扫中") }',
            GUI_SOURCE,
        )

    def test_rviz_shows_live_navigation_footprint_and_coverage_paths(self):
        for required in (
            "Topic: /map",
            "Topic: /coverage/planned_path",
            "Topic: /coverage/executed_path",
            "Marker Topic: /coverage/ui_markers",
            "Marker Topic: /coverage/markers",
            "Class: rviz/Polygon",
            "Name: Vehicle navigation footprint (base_link)",
            "Topic: /move_base/local_costmap/footprint",
            "Class: rviz/Odometry",
            "Name: Live vehicle pose and recent trail",
            "Keep: 120",
            "Topic: /Odometry",
            "Topic: /coverage/clicked_point",
            "Single click: false",
            "Fixed Frame: map",
        ):
            self.assertIn(required, COVERAGE_RVIZ_CONFIG)

        for required in (
            "Topic: /coverage/planned_path",
            "Topic: /coverage/executed_path",
            "Marker Topic: /coverage/ui_markers",
            "Marker Topic: /coverage/markers",
            "Topic: /coverage/clicked_point",
            "Single click: false",
        ):
            self.assertIn(required, RVIZ_CONFIG)

        for config in (RVIZ_CONFIG, COVERAGE_RVIZ_CONFIG):
            displays = yaml.safe_load(config)["Visualization Manager"]["Displays"]
            by_name = {display.get("Name"): display for display in displays}
            odometry = by_name["Live vehicle pose and recent trail"]
            self.assertEqual("rviz/Odometry", odometry["Class"])
            self.assertEqual("/Odometry", odometry["Topic"])
            self.assertEqual(120, odometry["Keep"])
            self.assertIs(True, odometry["Enabled"])
            self.assertIs(True, odometry["Value"])
            connector = by_name["Coverage connector preview (disabled)"]
            self.assertEqual("/coverage/planned_path", connector["Topic"])
            self.assertIs(False, connector["Enabled"])
            self.assertIs(False, connector["Value"])
            for path_name in ("TEB global plan", "TEB local plan"):
                self.assertIs(False, by_name[path_name]["Enabled"])
                self.assertIs(False, by_name[path_name]["Value"])
            aligned_scan = by_name["Localization aligned scan (dynamic)"]
            self.assertEqual(
                "/fast_lio_localization/aligned_scan", aligned_scan["Topic"]
            )
            self.assertIs(True, aligned_scan["Enabled"])
            vehicle_displays = [
                display for display in displays
                if display.get("Name") == "Vehicle navigation footprint (base_link)"
            ]
            self.assertEqual(1, len(vehicle_displays))
            vehicle_display = vehicle_displays[0]
            self.assertEqual("rviz/Polygon", vehicle_display["Class"])
            self.assertEqual(
                "/move_base/local_costmap/footprint", vehicle_display["Topic"]
            )
            self.assertIs(True, vehicle_display["Enabled"])
            self.assertIs(True, vehicle_display["Value"])
            self.assertNotIn(
                "rviz/RobotModel", [display.get("Class") for display in displays]
            )

        static_costmap = yaml.safe_load(
            (WORKSPACE_ROOT / "src/scripts/robot_bringup/config/costmap_common_static.yaml")
            .read_text(encoding="utf-8")
        )
        no_map_costmap = yaml.safe_load(
            (WORKSPACE_ROOT / "src/navigation_arena/arena-rosnav-3D/arena_navigation/"
             "arena_local_planer/model_based/conventional/config/dingo/"
             "costmap_common_params_nomap.yaml").read_text(encoding="utf-8")
        )
        teb = yaml.safe_load(
            (WORKSPACE_ROOT / "src/navigation_arena/arena-rosnav-3D/arena_navigation/"
             "arena_local_planer/model_based/conventional/config/dingo/"
             "teb_local_planner_params_nomap.yaml").read_text(encoding="utf-8")
        )["TebLocalPlannerROS"]
        coverage = yaml.safe_load(
            (WORKSPACE_ROOT / "src/application/autolabor_coverage/config/coverage.yaml")
            .read_text(encoding="utf-8")
        )
        expected_body = [
            [0.52, 0.35],
            [0.52, -0.35],
            [-0.52, -0.35],
            [-0.52, 0.35],
        ]
        self.assertEqual(expected_body, static_costmap["footprint"])
        self.assertEqual(expected_body, no_map_costmap["footprint"])
        self.assertEqual(expected_body, teb["footprint_model"]["vertices"])
        self.assertAlmostEqual(0.10, static_costmap["footprint_padding"])
        self.assertAlmostEqual(0.10, no_map_costmap["footprint_padding"])
        self.assertAlmostEqual(0.62, coverage["footprint_front_m"])
        self.assertAlmostEqual(0.62, coverage["footprint_rear_m"])
        self.assertAlmostEqual(0.45, coverage["footprint_half_width_m"])

    def test_local_costmap_is_a_prominent_foreground_overlay(self):
        for config in (RVIZ_CONFIG, COVERAGE_RVIZ_CONFIG):
            displays = yaml.safe_load(config)["Visualization Manager"]["Displays"]
            local_costmaps = [
                display for display in displays
                if display.get("Topic") == "/move_base/local_costmap/costmap"
            ]
            self.assertEqual(1, len(local_costmaps))
            local_costmap = local_costmaps[0]
            self.assertEqual("rviz/Map", local_costmap["Class"])
            self.assertEqual("costmap", local_costmap["Color Scheme"])
            self.assertIs(True, local_costmap["Enabled"])
            self.assertIs(True, local_costmap["Value"])
            self.assertGreaterEqual(local_costmap["Alpha"], 0.90)
            self.assertIs(False, local_costmap["Draw Behind"])

        operator_displays = yaml.safe_load(RVIZ_CONFIG)["Visualization Manager"]["Displays"]
        global_costmaps = [
            display for display in operator_displays
            if display.get("Topic") == "/move_base/global_costmap/costmap"
        ]
        self.assertEqual(1, len(global_costmaps))
        self.assertIs(False, global_costmaps[0]["Enabled"])
        self.assertIs(False, global_costmaps[0]["Value"])

    def test_recording_and_three_map_mapping_are_independent(self):
        self.assertTrue(RECORD_ROSBAG.is_file())
        self.assertTrue(os.access(str(RECORD_ROSBAG), os.X_OK))
        self.assertTrue(MAPPING_SESSION.is_file())
        self.assertTrue(os.access(str(MAPPING_SESSION), os.X_OK))
        self.assertIn('scripts/global_mapping_session.sh', GUI_SOURCE)
        self.assertIn('scripts/record_rosbag.sh', GUI_SOURCE)
        self.assertNotIn('record_rosbag.sh', MAPPING_SESSION_TEXT)
        self.assertIn('/cloud_registered', MAPPING_SESSION_TEXT)
        self.assertIn('/dual_lidar/scan', MAPPING_SESSION_TEXT)
        self.assertIn('fused_scan_mapper.py', MAPPING_SESSION_TEXT)
        self.assertIn('map_set_fuser.py', MAPPING_SESSION_TEXT)
        self.assertIn('STATIC_MAPPING_LATEST=', MAPPING_SESSION_TEXT)
        self.assertIn('录入静态地图', GUI_SOURCE)
        self.assertIn('结束静态地图录入', GUI_SOURCE)
        self.assertIn('static_map_mode_', GUI_SOURCE)
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

    def test_coverage_owns_move_base_and_blocks_overview_goal_preemption(self):
        for evidence in (
            'tool->getClassId() == QStringLiteral("rviz/SetGoal")',
            "coverage_owns_navigation",
            "覆盖任务正在独占 move_base，已阻止新的地图导航目标",
            "覆盖清扫正在独占 move_base，不能发送普通目标",
            "请在清扫页使用“取消覆盖清扫”",
        ):
            self.assertIn(evidence, GUI_SOURCE)

    def test_static_map_bootstrap_view_is_visible_before_initial_pose(self):
        for config in (RVIZ_CONFIG, COVERAGE_RVIZ_CONFIG):
            view = config[config.index("  Views:") : config.index("Window Geometry:")]
            self.assertIn("Target Frame: map", view)
            self.assertNotIn("Target Frame: base_link", view)
        for evidence in (
            "① 显示整张地图",
            "② 设置初始位姿",
            'QStringLiteral("rviz/SetInitialPose")',
            "map_origin_x",
            "map_origin_y",
            "map_origin_yaw",
            "width_pixels / axis_width_m",
            "height_pixels / axis_height_m",
            'view->subProp(QStringLiteral("Target Frame"))->setValue(QStringLiteral("map"))',
            "二维静态地图已加载，综合页 RViz 已自动显示完整地图",
        ):
            self.assertIn(evidence, GUI_SOURCE + GUI_HEADER)

    def test_local_view_follows_vehicle_only_after_localization(self):
        for evidence in (
            "③ 跟随车辆",
            "followOverviewVehicle",
            "setRvizFollowVehicleView",
            "rviz_follow_after_initial_pose_",
            "data.coverage_status.localized",
            'tf_buffer_.canTransform("map", "base_link"',
            'manager->setFixedFrame(QStringLiteral("map"))',
            '->setValue(QStringLiteral("base_link"))',
            'view->subProp(QStringLiteral("X"))->setValue(0.0)',
            'view->subProp(QStringLiteral("Y"))->setValue(0.0)',
            "局部跟车视角",
        ):
            self.assertIn(evidence, GUI_SOURCE + GUI_HEADER)
        follow_view = GUI_SOURCE[
            GUI_SOURCE.index("bool MainWindow::setRvizFollowVehicleView") :
            GUI_SOURCE.index("void MainWindow::updateNavigationPathDisplays")
        ]
        self.assertNotIn("ros::Duration", follow_view)

    def test_navigation_path_legend_and_stale_path_clearing_are_explicit(self):
        for evidence in (
            "路线图例：青色＝覆盖条带预览",
            "蓝色＝全局参考路线",
            "红色＝当前局部轨迹",
            "绿色＝覆盖执行记录",
            'kTebGlobalPlanDisplayName = "TEB global plan"',
            'kTebLocalPlanDisplayName = "TEB local plan"',
            "updateNavigationPathDisplays(data)",
            "actionlib_msgs::GoalStatus::PENDING",
            "actionlib_msgs::GoalStatus::ACTIVE",
            "display->setEnabled(goal_active)",
        ):
            self.assertIn(evidence, GUI_SOURCE + GUI_HEADER)

    def test_known_3d_map_is_an_explicit_reversible_option(self):
        name = "Name: Known 3D global map (optional)"
        name_at = RVIZ_CONFIG.index(name)
        block_start = RVIZ_CONFIG.rfind("\n    - ", 0, name_at)
        block_end = RVIZ_CONFIG.find("\n    - ", name_at)
        prior_map = RVIZ_CONFIG[block_start:block_end]
        for evidence in (
            "Class: rviz/PointCloud2",
            "Topic: /fast_lio_localization/prior_map",
            "Decay Time: 0",
            "Enabled: false",
            "Value: false",
            "Color Transformer: AxisColor",
            "Axis: Z",
        ):
            self.assertIn(evidence, prior_map)

        default_view = RVIZ_CONFIG[
            RVIZ_CONFIG.index("  Views:") : RVIZ_CONFIG.index("Window Geometry:")
        ]
        self.assertIn("Class: rviz/TopDownOrtho", default_view)
        self.assertIn("Target Frame: map", default_view)
        for evidence in (
            "④ 显示静态三维先验",
            "rviz_3d_map_button_->setCheckable(true)",
            "rviz_3d_map_button_->setChecked(false)",
            "setOverview3dMapView",
            'setCurrentViewControllerType(QStringLiteral("rviz/Orbit"))',
            'setCurrentViewControllerType(QStringLiteral("rviz/TopDownOrtho"))',
            "findDisplayByName",
            "prior_map->setEnabled(true)",
            "prior_map->setEnabled(false)",
            'selectRvizTool(rviz_frame_, QStringLiteral("rviz/MoveCamera"))',
            "if (!setOverview3dMapView(false, data))",
            "Always enforce the real RViz state",
            "Validate all geometry before changing the display or view controller",
            "!overview_3d_map_enabled_",
        ):
            self.assertIn(evidence, GUI_SOURCE + GUI_HEADER)
        self.assertGreaterEqual(
            GUI_SOURCE.count("if (!setOverview3dMapView(false, data))"), 2
        )

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

    def test_static_map_launch_exposes_map_frame_without_faking_robot_pose(self):
        launch_root = ElementTree.parse(
            str(PACKAGE_ROOT / "launch" / "operator_gui.launch")
        ).getroot()
        static_groups = [
            group
            for group in launch_root.findall("group")
            if group.attrib.get("if") == "$(arg static_map_mode)"
        ]
        self.assertEqual(1, len(static_groups))
        anchor = static_groups[0].find("node")
        self.assertIsNotNone(anchor)
        self.assertEqual("tf2_ros", anchor.attrib["pkg"])
        self.assertEqual("static_transform_publisher", anchor.attrib["type"])
        self.assertEqual("operator_map_display_anchor", anchor.attrib["name"])
        self.assertEqual("true", anchor.attrib["required"])
        self.assertEqual(
            "0 0 0 0 0 0 1 map autolabor_map_display_anchor",
            anchor.attrib["args"],
        )
        self.assertNotIn("camera_init", anchor.attrib["args"])

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
        cloud_start = RVIZ_CONFIG.index("Class: rviz/PointCloud2")
        cloud_display = RVIZ_CONFIG[
            cloud_start : RVIZ_CONFIG.index("Class: rviz/Path", cloud_start)
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

    def test_overview_and_coverage_share_one_rviz_manager(self):
        self.assertIn("overview_tab_index_", GUI_HEADER)
        self.assertIn("coverage_tab_index_", GUI_HEADER)
        self.assertIn("rviz_attached_tab_index_", GUI_HEADER)
        self.assertNotIn("coverage_rviz_frame_", GUI_HEADER + GUI_SOURCE)
        self.assertNotIn("coverage_rviz_initialized_", GUI_HEADER + GUI_SOURCE)
        self.assertEqual(1, GUI_SOURCE.count("new rviz::VisualizationFrame"))
        self.assertIn("QTabWidget::currentChanged", GUI_SOURCE)
        self.assertIn("QTimer::singleShot(0, this", GUI_SOURCE)
        self.assertIn("void MainWindow::attachRvizToTab", GUI_SOURCE)
        self.assertIn("rviz_frame_->setParent(target_host, Qt::Widget);", GUI_SOURCE)
        setup_ros = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::setupRosInterfaces()") :
            GUI_SOURCE.index("void MainWindow::setupEmbeddedRviz()")
        ]
        self.assertIn("if (active_tab == coverage_tab_index_)", setup_ros)
        self.assertIn("else if (active_tab == overview_tab_index_)", setup_ros)
        self.assertNotIn("setupEmbeddedRviz();\n  setupCoverageRviz();", setup_ros)
        coverage_setup = GUI_SOURCE[
            GUI_SOURCE.index("void MainWindow::setupCoverageRviz()") :
            GUI_SOURCE.index("void MainWindow::shutdownRosInterfaces()")
        ]
        self.assertIn("setupEmbeddedRviz();", coverage_setup)
        self.assertNotIn("new rviz::VisualizationFrame", coverage_setup)

    def test_map_fit_waits_for_real_render_panel_size(self):
        fit = GUI_SOURCE[
            GUI_SOURCE.index("bool MainWindow::fitRvizMapView") :
            GUI_SOURCE.index("bool MainWindow::selectRvizTool")
        ]
        self.assertIn("render_panel->width() < 200", fit)
        self.assertIn("render_panel->height() < 120", fit)

    def test_embedded_map_display_resubscribes_and_reports_actual_readiness(self):
        readiness = GUI_SOURCE[
            GUI_SOURCE.index("bool MainWindow::ensureStaticMapDisplayReady") :
            GUI_SOURCE.index("bool MainWindow::fitRvizMapView")
        ]
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("dynamic_cast<rviz::MapDisplay*>", readiness)
        self.assertIn('kStaticMapDisplayName = "Static global map"', GUI_SOURCE)
        self.assertLess(
            readiness.index("map_display->setEnabled(false);"),
            readiness.index("map_display->setEnabled(true);"),
        )
        self.assertIn("map_display->getWidth()", readiness)
        self.assertIn("map_display->getHeight()", readiness)
        self.assertIn("map_display->getResolution()", readiness)
        self.assertIn('publishMapDisplayStatus("ERROR;', readiness)
        self.assertIn('"READY;width="', readiness)
        self.assertIn(
            '"/autolabor_operator_gui/map_display_status", 1, true', GUI_SOURCE
        )
        self.assertIn("find_library(RVIZ_DEFAULT_PLUGIN_LIBRARY", cmake)
        self.assertIn("${RVIZ_DEFAULT_PLUGIN_LIBRARY}", cmake)
        self.assertIn("embedded_map_ready", GUI_SOURCE)

    def test_light_rviz_and_coverage_sidebar_surfaces_use_dark_text(self):
        self.assertIn("QToolBar { background: #f5f5f5; color: #111827;", GUI_SOURCE)
        self.assertIn("QMenu, QAbstractItemView { background: #ffffff; color: #111827;",
                      GUI_SOURCE)
        self.assertIn("QWidget#coverageControls QGroupBox::title { color: #111827; }",
                      GUI_SOURCE)

    def test_light_message_boxes_use_dark_readable_text_and_buttons(self):
        self.assertIn(
            "QMessageBox { background: #f4f6f8; color: #111827; }",
            GUI_SOURCE,
        )
        self.assertIn(
            "QMessageBox QLabel { background: transparent; color: #111827; }",
            GUI_SOURCE,
        )
        self.assertIn(
            "QMessageBox QPushButton { background: #e5e7eb; color: #111827;",
            GUI_SOURCE,
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
        self.assertIn("sensor_msgs::image_encodings::BGRA8", GUI_SOURCE)
        self.assertIn("sensor_msgs::image_encodings::RGBA8", GUI_SOURCE)
        self.assertIn("message.width) * 4U", GUI_SOURCE)
        self.assertIn("QImage::Format_RGBA8888", GUI_SOURCE)

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
