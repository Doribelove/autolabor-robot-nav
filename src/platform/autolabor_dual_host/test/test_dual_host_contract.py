#!/usr/bin/env python3

import importlib.util
import json
import os
import shlex
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest import mock


PACKAGE_DIR = os.path.dirname(os.path.dirname(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, "..", "..", ".."))


class DualHostContractTest(unittest.TestCase):
    def _launch_text(self, name):
        path = os.path.join(PACKAGE_DIR, "launch", name)
        ET.parse(path)
        with open(path, "r", encoding="utf-8") as stream:
            return stream.read()

    @staticmethod
    def _write_text(path, value):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(value)

    def _add_fake_usb_network_device(
        self, temporary, sys_class_net, interface, mac, usb_id, serial
    ):
        usb_device = os.path.join(temporary, "devices", interface)
        usb_function = os.path.join(usb_device, "net-function")
        os.makedirs(usb_function)
        vendor, product = usb_id.split(":")
        self._write_text(os.path.join(usb_device, "idVendor"), vendor + "\n")
        self._write_text(os.path.join(usb_device, "idProduct"), product + "\n")
        self._write_text(os.path.join(usb_device, "serial"), serial + "\n")
        interface_dir = os.path.join(sys_class_net, interface)
        os.makedirs(interface_dir)
        self._write_text(os.path.join(interface_dir, "address"), mac + "\n")
        self._write_text(os.path.join(interface_dir, "carrier"), "1\n")
        os.symlink(usb_function, os.path.join(interface_dir, "device"))

    def _write_network_test_config(self, path):
        self._write_text(
            path,
            """J6M_IP=192.168.10.100
NVIDIA_J6M_IP=192.168.10.50
NVIDIA_J6M_INTERFACE=eth0
NVIDIA_J6M_MAC=6C:1F:F7:C4:82:83
NVIDIA_J6M_USB_ID=0B95:1790
NVIDIA_J6M_USB_SERIAL=000000000011CA
NVIDIA_J6M_CONNECTION=matrix-eth2
NVIDIA_LIVOX_INTERFACE=eth0
NVIDIA_LIVOX_MAC=50:54:7B:E3:C9:10
NVIDIA_LIVOX_USB_ID=1A86:E397
NVIDIA_LIVOX_USB_SERIAL=50547BE3C910
NVIDIA_LIVOX_CONNECTION=mid360-eth0
NVIDIA_LIVOX_IP=192.168.1.50
MID360_IP=192.168.1.112
DUAL_HOST_DEVICE_WAIT_SEC=1
DUAL_HOST_NETWORK_WAIT_SEC=1
DUAL_HOST_NETWORK_POLL_SEC=0.01
""",
        )

    def test_network_identity_uses_exact_usb_fallback_not_stale_eth_name(self):
        load_config = os.path.join(WORKSPACE_DIR, "scripts", "load_config.sh")
        with tempfile.TemporaryDirectory() as temporary:
            sys_class_net = os.path.join(temporary, "net")
            os.makedirs(sys_class_net)
            self._add_fake_usb_network_device(
                temporary,
                sys_class_net,
                "eth0",
                "50:54:7b:e3:c9:10",
                "1a86:e397",
                "50547BE3C910",
            )
            self._add_fake_usb_network_device(
                temporary,
                sys_class_net,
                "eth7",
                "6c:1f:f7:c4:96:b8",
                "0b95:1790",
                "000000000011CA",
            )
            config = os.path.join(temporary, "dual_host.env")
            self._write_network_test_config(config)
            script = r'''set -euo pipefail
DUAL_HOST_CONFIG="$1"
DUAL_HOST_SYS_CLASS_NET_ROOT="$2"
source "$3"
printf 'J6M=%s %s %s\n' "$NVIDIA_J6M_INTERFACE" "$NVIDIA_J6M_MAC" "$NVIDIA_J6M_IDENTITY_SOURCE"
printf 'MID360=%s %s %s\n' "$NVIDIA_LIVOX_INTERFACE" "$NVIDIA_LIVOX_MAC" "$NVIDIA_LIVOX_IDENTITY_SOURCE"
dual_host_network_roles_are_distinct
'''
            result = subprocess.run(
                ["bash", "-c", script, "bash", config, sys_class_net, load_config],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(
            "J6M=eth7 6c:1f:f7:c4:96:b8 usb-id+serial\n"
            "MID360=eth0 50:54:7b:e3:c9:10 configured-mac\n",
            result.stdout,
        )

    def test_j6m_time_sync_is_subsecond_and_latency_checked(self):
        path = os.path.join(WORKSPACE_DIR, "scripts", "sync_j6m_time.sh")
        with open(path, "r", encoding="utf-8") as stream:
            sync = stream.read()
        for evidence in (
            "coproc J6M_TIME_SSH",
            "__J6M_TIME_READY__",
            "date +%s.%N",
            "date +%s%N",
            "local_midpoint_ns",
            "max_skew_ns=100000000",
            "midpoint skew",
        ):
            self.assertIn(evidence, sync)
        self.assertNotIn('host_epoch="$(date +%s)"', sync)

    def test_duplicate_usb_identity_is_not_accepted(self):
        load_config = os.path.join(WORKSPACE_DIR, "scripts", "load_config.sh")
        with tempfile.TemporaryDirectory() as temporary:
            sys_class_net = os.path.join(temporary, "net")
            os.makedirs(sys_class_net)
            self._add_fake_usb_network_device(
                temporary,
                sys_class_net,
                "eth0",
                "50:54:7b:e3:c9:10",
                "1a86:e397",
                "50547BE3C910",
            )
            for interface, mac in (
                ("eth7", "6c:1f:f7:c4:96:b8"),
                ("eth8", "6c:1f:f7:c4:96:b9"),
            ):
                self._add_fake_usb_network_device(
                    temporary,
                    sys_class_net,
                    interface,
                    mac,
                    "0b95:1790",
                    "000000000011CA",
                )
            config = os.path.join(temporary, "dual_host.env")
            self._write_network_test_config(config)
            script = r'''set -euo pipefail
DUAL_HOST_CONFIG="$1"
DUAL_HOST_SYS_CLASS_NET_ROOT="$2"
source "$3"
[[ -z "$NVIDIA_J6M_INTERFACE" ]]
[[ "$NVIDIA_LIVOX_INTERFACE" == eth0 ]]
! dual_host_network_roles_are_distinct
'''
            result = subprocess.run(
                ["bash", "-c", script, "bash", config, sys_class_net, load_config],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

        self.assertEqual(0, result.returncode, result.stderr)

    def test_profile_repair_sets_managed_before_activation(self):
        load_config = os.path.join(WORKSPACE_DIR, "scripts", "load_config.sh")
        prepare = os.path.join(WORKSPACE_DIR, "scripts", "network_prepare.sh")
        with tempfile.TemporaryDirectory() as temporary:
            sys_class_net = os.path.join(temporary, "net")
            os.makedirs(sys_class_net)
            self._add_fake_usb_network_device(
                temporary,
                sys_class_net,
                "eth0",
                "50:54:7b:e3:c9:10",
                "1a86:e397",
                "50547BE3C910",
            )
            self._add_fake_usb_network_device(
                temporary,
                sys_class_net,
                "eth7",
                "6c:1f:f7:c4:96:b8",
                "0b95:1790",
                "000000000011CA",
            )
            config = os.path.join(temporary, "dual_host.env")
            log = os.path.join(temporary, "nmcli.log")
            state = os.path.join(temporary, "state")
            os.makedirs(state)
            self._write_network_test_config(config)
            script = r'''set -euo pipefail
DUAL_HOST_CONFIG="$1"
DUAL_HOST_SYS_CLASS_NET_ROOT="$2"
log="$3"
state="$4"
source "$5"
source "$6"
nmcli() {
  printf '%q ' "$@" >>"$log"
  printf '\n' >>"$log"
  case "$*" in
    "device show eth7") return 0 ;;
    "device set eth7 managed yes") touch "$state/managed"; return 0 ;;
    "-e no -g GENERAL.NM-MANAGED device show eth7")
      [[ -f "$state/managed" ]] && echo yes || echo no
      return 0 ;;
    "connection show matrix-eth2") return 0 ;;
    "-e no -g connection.interface-name connection show matrix-eth2") echo eth0; return 0 ;;
    "-e no -g 802-3-ethernet.mac-address connection show matrix-eth2") echo '6C:1F:F7:C4:82:83'; return 0 ;;
    "-e no -g connection.autoconnect connection show matrix-eth2") echo no; return 0 ;;
    "-e no -g ipv4.method connection show matrix-eth2") echo manual; return 0 ;;
    "-e no -g ipv4.addresses connection show matrix-eth2") echo 192.168.10.50/24; return 0 ;;
    "-e no -g ipv4.never-default connection show matrix-eth2") echo yes; return 0 ;;
    "-e no -g ipv4.gateway connection show matrix-eth2") return 0 ;;
    "-e no -g ipv6.method connection show matrix-eth2") echo disabled; return 0 ;;
    "connection modify matrix-eth2 "*) touch "$state/modified"; return 0 ;;
    "-e no -g GENERAL.CONNECTION device show eth7")
      [[ -f "$state/active" ]] && echo matrix-eth2 || echo --
      return 0 ;;
    "--wait 15 connection up matrix-eth2 ifname eth7")
      [[ -f "$state/managed" && -f "$state/modified" ]] || return 8
      touch "$state/active" "$state/address"
      return 0 ;;
  esac
  echo "unexpected nmcli call: $*" >&2
  return 9
}
ip() {
  if [[ "$*" == "-o -4 address show dev eth7" && -f "$state/address" ]]; then
    echo '7: eth7 inet 192.168.10.50/24 scope global eth7'
  fi
}
dual_host_prepare_profile NVIDIA_J6M J6M matrix-eth2 192.168.10.50
'''
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    script,
                    "bash",
                    config,
                    sys_class_net,
                    log,
                    state,
                    load_config,
                    prepare,
                ],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            with open(log, "r", encoding="utf-8") as stream:
                commands = stream.read()

        self.assertEqual(0, result.returncode, result.stderr)
        managed = commands.index("device set eth7 managed yes")
        modified = commands.index("connection modify matrix-eth2")
        activated = commands.index("--wait 15 connection up matrix-eth2 ifname eth7")
        self.assertLess(managed, modified)
        self.assertLess(modified, activated)
        modify_command = commands[modified : commands.index("\n", modified)]
        self.assertIn("connection.interface-name ''", modify_command)
        self.assertIn("connection.autoconnect yes", modify_command)
        self.assertIn(
            "802-3-ethernet.mac-address 6c:1f:f7:c4:96:b8", modify_command
        )
        self.assertNotIn("connection.interface-name eth7", modify_command)

    def test_network_preflight_precedes_remote_stop(self):
        start_path = os.path.join(WORKSPACE_DIR, "scripts", "start_dual_host.sh")
        with open(start_path, "r", encoding="utf-8") as stream:
            start = stream.read()
        supervisor = start[start.index('echo "[1/6] Self-checking'):]
        first_prepare = supervisor.index("dual_host_prepare_network")
        first_stop = supervisor.index('"$SCRIPT_DIR/stop_dual_host.sh"')
        second_prepare = supervisor.index("dual_host_prepare_network", first_stop)
        self.assertLess(first_prepare, first_stop)
        self.assertGreater(second_prepare, first_stop)
        start_branch = start[start.index('elif [[ "$mode" == --start ]]'):]
        self.assertLess(
            start_branch.index("dual_host_prepare_network"),
            start_branch.index("start_managed_service"),
        )

    def test_nvidia_owns_only_final_driver_topic(self):
        text = self._launch_text("nvidia_gateway.launch")
        self.assertIn('<param name="input_topic" value="/cmd_vel_safe"/>', text)
        self.assertIn('<param name="output_topic" value="/cmd_vel"/>', text)
        self.assertIn('<arg name="motion_enabled" default="false"/>', text)
        self.assertIn('/gateway/livox/lidar', text)
        self.assertIn('/gateway/livox/imu', text)
        self.assertIn('<arg name="lidar_center_distance_m" default="0.92"/>', text)
        self.assertIn(
            '<arg name="lidar_center_distance_m" value="$(arg lidar_center_distance_m)"/>',
            text,
        )

    def test_navigation_default_and_watchdog_hard_speed_limits_are_separate(self):
        nvidia_launch = self._launch_text("nvidia_gateway.launch")
        j6m_launch = self._launch_text("j6m_fastlio_navigation.launch")
        self.assertIn('<arg name="max_linear_speed" default="1.70"/>', nvidia_launch)
        self.assertIn('<arg name="max_linear_speed" default="0.80"/>', j6m_launch)
        self.assertIn('<arg name="max_angular_speed" default="0.60"/>', nvidia_launch)
        self.assertIn('<arg name="max_angular_speed" default="0.60"/>', j6m_launch)
        gateway = os.path.join(WORKSPACE_DIR, "scripts", "nvidia_gateway.sh")
        with open(gateway, "r", encoding="utf-8") as stream:
            gateway_text = stream.read()
        self.assertIn('max_linear_speed:="$CMD_VEL_MAX_LINEAR_SPEED"', gateway_text)
        self.assertIn('max_angular_speed:="$CMD_VEL_MAX_ANGULAR_SPEED"', gateway_text)
        example = os.path.join(WORKSPACE_DIR, "config", "dual_host.env.example")
        with open(example, "r", encoding="utf-8") as stream:
            example_text = stream.read()
        self.assertIn("NAV_MAX_LINEAR_SPEED=0.80", example_text)
        self.assertIn("CMD_VEL_MAX_LINEAR_SPEED=1.70", example_text)
        self.assertIn("CMD_VEL_MAX_ANGULAR_SPEED=0.60", example_text)

    def test_ai_navigation_uses_explicit_goal_id_through_j6m_safety_bridge(self):
        launch = self._launch_text("j6m_fastlio_navigation.launch")
        bridge_path = os.path.join(
            WORKSPACE_DIR,
            "src/platform/autolabor_dual_host/scripts/move_base_pause_bridge.py",
        )
        backend_path = os.path.join(
            WORKSPACE_DIR,
            "src/sweeper_mcp/src/sweeper_mcp/ros_backend.py",
        )
        fod_manager_path = os.path.join(
            WORKSPACE_DIR,
            "src/application/autolabor_fod_control/scripts/"
            "fod_navigation_mode_manager.py",
        )
        coverage_owner_service_path = os.path.join(
            WORKSPACE_DIR,
            "src/application/autolabor_coverage/srv/SetCoverageOwner.srv",
        )
        dual_host_cmake_path = os.path.join(
            WORKSPACE_DIR,
            "src/platform/autolabor_dual_host/CMakeLists.txt",
        )
        dual_host_package_path = os.path.join(
            WORKSPACE_DIR,
            "src/platform/autolabor_dual_host/package.xml",
        )
        with open(bridge_path, "r", encoding="utf-8") as stream:
            bridge = stream.read()
        with open(backend_path, "r", encoding="utf-8") as stream:
            backend = stream.read()
        with open(fod_manager_path, "r", encoding="utf-8") as stream:
            fod_manager = stream.read()
        with open(coverage_owner_service_path, "r", encoding="utf-8") as stream:
            coverage_owner_service = stream.read()
        with open(dual_host_cmake_path, "r", encoding="utf-8") as stream:
            dual_host_cmake = stream.read()
        with open(dual_host_package_path, "r", encoding="utf-8") as stream:
            dual_host_package = stream.read()

        for evidence in (
            '<param name="action_goal_request_topic" value="/navigation_goal/action_request"/>',
            '<param name="action_goal_topic" value="/move_base/goal"/>',
            '<param name="action_status_topic" value="/move_base/status"/>',
            '<param name="required_action_server_node" value="/move_base"/>',
            '<param name="require_coverage_state" type="bool" value="$(arg use_static_map)"/>',
            '<param name="coverage_owner_service" value="/navigation_pause/set_coverage_owner"/>',
            '<param name="action_cancel_ack_topic" value="/navigation_goal/cancel_ack"/>',
            '<param name="coverage_claim_cancel_timeout_sec" type="double" value="2.0"/>',
            '/navigation_goal/legacy_simple_input_disabled',
        ):
            self.assertIn(evidence, launch)
        self.assertIn("MoveBaseActionGoal", bridge)
        self.assertIn("_action_goal_request_callback", bridge)
        self.assertIn("AI_GOAL_ID_RE.fullmatch", bridge)
        self.assertIn("_publish_cancel_goal_id", bridge)
        self.assertIn("_action_output_ready", bridge)
        self.assertIn("_set_coverage_owner", bridge)
        self.assertIn("_submit_simple_action_locked", bridge)
        self.assertIn("SIMPLE_GOAL_ID_RE", bridge)
        self.assertIn('acknowledgement.text = "not_forwarded"', bridge)
        self.assertIn(
            "self.coverage_owner_token or self.coverage_topic_active", bridge)
        self.assertEqual(
            [
                "bool claim",
                "string owner_token",
                "---",
                "bool success",
                "bool claimed",
                "string current_owner_token",
                "string message",
            ],
            coverage_owner_service.splitlines(),
        )
        self.assertIn("autolabor_coverage", dual_host_cmake)
        self.assertIn("<depend>autolabor_coverage</depend>", dual_host_package)
        self.assertIn('"action_request": "/navigation_goal/action_request"', backend)
        self.assertIn('goal_id = "sweeper-ai-%s" % uuid.uuid4().hex', backend)
        self.assertIn(
            'self._topics["ai_heartbeat"], GoalID, queue_size=1', backend)
        self.assertIn("_wait_cancel_confirmation", backend)
        self.assertIn('anonymous=False', backend)
        # Actionizing Qt goals does not bypass FOD pause or velocity ownership:
        # the manager observes every /move_base/goal and remains the sole
        # /cmd_vel_safe publisher in this launch.
        self.assertIn('"~move_base_goal_topic", "/move_base/goal"', fod_manager)
        self.assertIn("self._move_base_goal_cb", fod_manager)
        self.assertIn(
            '<param name="output_cmd_topic" value="/cmd_vel_safe"/>', launch)

    def test_runtime_health_checks_data_host_placement_and_live_limits(self):
        health_path = os.path.join(WORKSPACE_DIR, "scripts", "health_check.sh")
        with open(health_path, "r", encoding="utf-8") as stream:
            health = stream.read()
        self.assertIn("topic_has_recent_message", health)
        self.assertIn("node_on_host", health)
        self.assertIn("'$2 == wanted {print $1; exit}'", health)
        for topic in (
            "/gateway/livox/lidar",
            "/gateway/livox/imu",
            "/Odometry",
            "/cloud_registered_body",
            "/scan",
            "/cmd_vel_safe",
            "/cmd_vel",
        ):
            self.assertIn(topic, health)
        self.assertIn("/nvidia_cmd_vel_watchdog/max_linear_speed", health)
        self.assertIn("/nvidia_cmd_vel_watchdog/max_angular_speed", health)
        self.assertIn("/fod_visual_servo/expected_model_sha256", health)
        self.assertIn("/fod_visual_servo/allowed_class_names", health)
        self.assertIn("/fod_detector/expected_model_sha256", health)
        self.assertIn("/fod_detector/required_class_names", health)
        self.assertIn("/move_base/TebLocalPlannerROS/max_vel_theta", health)
        self.assertIn("--allow-missing-data", health)
        self.assertIn("stack remains running", health)
        self.assertIn("critical_runtime_topics", health)
        self.assertIn("/fod_camera/image_raw", health)
        self.assertIn("/fod_camera/depth_registered", health)
        self.assertIn("required camera topic", health)
        self.assertIn("zed_camera_check.sh", health)
        self.assertIn("/operator_map_display_anchor", health)
        self.assertIn("transform_is_available map autolabor_map_display_anchor", health)
        self.assertIn("^[[:space:]-]*Translation:", health)
        self.assertIn("without connecting robot TF", health)
        self.assertIn("/autolabor_operator_gui/map_display_status", health)
        self.assertIn("'READY;'", health)
        self.assertIn("confirmed the 2-D map texture is loaded", health)

    def test_static_health_enforces_local_large_v3_without_opening_audio(self):
        health_path = os.path.join(WORKSPACE_DIR, "scripts", "health_check.sh")
        with open(health_path, "r", encoding="utf-8") as stream:
            health = stream.read()

        start = health.index('ASR_LARGE_V3_SHA256="')
        end = health.index("if dual_host_validate_fod_model_contract", start)
        asr_check = health[start:end]
        for evidence in (
            "ASR_LARGE_V3_SHA256",
            "e5b1a55b89c1367dacf97e3e19bfd829a01529dbfdeefa8caeb59b3f1b81dadb",
            "SWEEPER_AI_CONFIG",
            "asr.enabled must be a boolean",
            "asr.model must be large-v3",
            "asr.device must be cuda",
            "model_sha256",
            "checkpoint_sha256",
            "runtime/asr/venv/bin/python3",
            "runtime/asr/models/large-v3.pt",
            "import whisper",
            'torch.version.cuda == "11.4"',
            "torch.cuda.is_available()",
            "sha256sum",
            "no physical microphone input_device is configured",
            "device was not opened",
        ):
            self.assertIn(evidence, asr_check)
        self.assertNotIn("whisper.load_model", asr_check)
        self.assertNotIn("arecord ", asr_check)
        self.assertNotIn("pyaudio", asr_check.lower())
        self.assertIn(
            'if [[ "$mode" == --static ]]; then\n  check_nvidia_asr_static_contract',
            health,
        )

    def test_fod_model_contract_is_shared_and_versioned_on_j6m(self):
        def script_text(relative_path):
            with open(
                os.path.join(WORKSPACE_DIR, relative_path),
                "r",
                encoding="utf-8",
            ) as stream:
                return stream.read()

        launch = self._launch_text("j6m_fastlio_navigation.launch")
        stack = script_text(
            "src/platform/autolabor_dual_host/scripts/j6m_stack.sh"
        )
        remote_start = script_text("deploy/j6m/start.sh")
        managed_start = script_text("scripts/start_dual_host.sh")
        deploy = script_text("scripts/deploy_j6m.sh")
        config = script_text("config/dual_host.env.example")

        self.assertIn('arg name="fod_model_sha256"', launch)
        self.assertIn('arg name="fod_required_class_names"', launch)
        self.assertIn(
            'value="$(arg fod_model_sha256)"', launch
        )
        self.assertIn(
            'value="$(arg fod_required_class_names)"', launch
        )
        for variable in (
            "NVIDIA_FOD_MODEL_SHA256",
            "NVIDIA_FOD_REQUIRED_CLASS_NAMES",
        ):
            self.assertIn(variable, config)
            self.assertIn(variable, stack)
            self.assertIn(variable, remote_start)
        self.assertIn("dual_host_validate_fod_model_contract", managed_start)
        self.assertIn("dual_host_validate_fod_weights", managed_start)
        self.assertIn("sync_j6m_runtime_config", managed_start)
        self.assertIn("verify_j6m_visual_model_contract_release", managed_start)
        self.assertIn("requested_fod_motion_enabled", managed_start)
        self.assertIn("./src/application/autolabor_fod_control", deploy)
        self.assertIn(
            "robot_bringup\\\\;autolabor_fod_control\\\\;autolabor_dual_lidar",
            deploy,
        )
        self.assertIn("rospack find autolabor_fod_control", deploy)

    def test_one_run_fod_motion_authorization_is_explicit_and_documented(self):
        def script_text(relative_path):
            with open(
                os.path.join(WORKSPACE_DIR, relative_path),
                "r",
                encoding="utf-8",
            ) as stream:
                return stream.read()

        managed_start = script_text("scripts/start_dual_host.sh")
        load_config = script_text("scripts/load_config.sh")
        remote_start = script_text("deploy/j6m/start.sh")
        readme = script_text("README.md")

        self.assertIn("--authorize-fod-motion", managed_start)
        self.assertIn(
            '--setenv="DUAL_HOST_FOD_MOTION_OVERRIDE=true"', managed_start
        )
        self.assertIn("DUAL_HOST_FOD_MOTION_OVERRIDE", load_config)
        self.assertIn("requested_fod_motion_enabled", remote_start)
        self.assertIn("requested_fod_motion_enabled", script_text("scripts/deploy_j6m.sh"))
        self.assertIn(
            'FOD_MOTION_ENABLED="$FOD_MOTION_ENABLED"', managed_start
        )
        self.assertIn(
            "printf 'FOD_MOTION_ENABLED=%q\\n'", managed_start
        )
        self.assertIn(
            "/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --start \\",
            readme,
        )
        self.assertIn("--authorize-fod-motion </dev/null", readme)
        self.assertIn(
            "/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --stop </dev/null",
            readme,
        )

        with tempfile.TemporaryDirectory() as temporary:
            config = os.path.join(temporary, "dual_host.env")
            sys_class_net = os.path.join(temporary, "net")
            os.makedirs(sys_class_net)
            self._write_text(
                config,
                """J6M_IP=192.168.10.100
NVIDIA_J6M_IP=192.168.10.50
MOTION_ENABLED=true
FOD_MOTION_ENABLED=false
""",
            )
            command = r'''set -e
DUAL_HOST_CONFIG="$1"
DUAL_HOST_SYS_CLASS_NET_ROOT="$2"
DUAL_HOST_FOD_MOTION_OVERRIDE=true
export DUAL_HOST_CONFIG DUAL_HOST_SYS_CLASS_NET_ROOT
export DUAL_HOST_FOD_MOTION_OVERRIDE
source "$3"
printf '%s\n' "$FOD_MOTION_ENABLED"
'''
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    command,
                    "bash",
                    config,
                    sys_class_net,
                    os.path.join(WORKSPACE_DIR, "scripts", "load_config.sh"),
                ],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual("true\n", result.stdout)

    def test_zed_usb_preflight_requires_superspeed(self):
        check = os.path.join(WORKSPACE_DIR, "scripts", "zed_camera_check.sh")
        with tempfile.TemporaryDirectory() as temporary:
            sys_root = os.path.join(temporary, "sys")
            hidraw_sys_root = os.path.join(temporary, "hidraw")
            device_root = os.path.join(temporary, "dev")
            dev_root = os.path.join(device_root, "bus", "usb")
            video = os.path.join(sys_root, "2-1.1")
            hid = os.path.join(sys_root, "2-1.2")
            os.makedirs(video)
            os.makedirs(hid)
            for device, product, devnum in (
                (video, "f780", "10"),
                (hid, "f781", "11"),
            ):
                self._write_text(os.path.join(device, "idVendor"), "2b03\n")
                self._write_text(os.path.join(device, "idProduct"), product + "\n")
                self._write_text(os.path.join(device, "busnum"), "2\n")
                self._write_text(os.path.join(device, "devnum"), devnum + "\n")
            self._write_text(os.path.join(video, "speed"), "480\n")
            self._write_text(os.path.join(hid, "serial"), "23748636\n")
            os.makedirs(os.path.join(dev_root, "002"))
            self._write_text(os.path.join(dev_root, "002", "010"), "")
            self._write_text(os.path.join(dev_root, "002", "011"), "")
            # ZED SDK 4 can use the accessible f781 usbfs endpoint even when
            # this Jetson kernel does not bind the optional hidraw interface.
            os.makedirs(hidraw_sys_root)
            config = os.path.join(temporary, "dual_host.env")
            self._write_network_test_config(config)
            environment = dict(os.environ)
            environment.update(
                {
                    "DUAL_HOST_CONFIG": config,
                    "DUAL_HOST_USB_SYS_ROOT": sys_root,
                    "DUAL_HOST_USB_DEV_ROOT": dev_root,
                    "DUAL_HOST_HIDRAW_SYS_ROOT": hidraw_sys_root,
                    "DUAL_HOST_DEVICE_ROOT": device_root,
                }
            )
            usb2 = subprocess.run(
                [check, "--wait", "0"],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=environment,
            )
            self._write_text(os.path.join(video, "speed"), "5000\n")
            usb3 = subprocess.run(
                [check, "--wait", "0"],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=environment,
            )

        self.assertNotEqual(0, usb2.returncode)
        self.assertIn("negotiated only 480M", usb2.stderr)
        self.assertEqual(0, usb3.returncode, usb3.stderr)
        self.assertIn("SuperSpeed USB (5000M", usb3.stdout)

    def test_zed_udev_install_repairs_post_boot_coldplug(self):
        rule = os.path.join(WORKSPACE_DIR, "deploy", "99-autolabor-zed.rules")
        service = os.path.join(
            WORKSPACE_DIR, "deploy", "autolabor-zed-coldplug.service"
        )
        helper = os.path.join(
            WORKSPACE_DIR, "deploy", "autolabor-usb-coldplug.sh"
        )
        installer = os.path.join(WORKSPACE_DIR, "scripts", "install_zed_udev.sh")
        with open(rule, "r", encoding="utf-8") as stream:
            rule_text = stream.read()
        with open(service, "r", encoding="utf-8") as stream:
            service_text = stream.read()
        with open(helper, "r", encoding="utf-8") as stream:
            helper_text = stream.read()
        with open(installer, "r", encoding="utf-8") as stream:
            installer_text = stream.read()

        self.assertIn('ATTR{idVendor}=="2b03"', rule_text)
        self.assertIn('ATTR{idProduct}=="f780"', rule_text)
        self.assertIn('ATTR{idProduct}=="f781"', rule_text)
        self.assertIn('GROUP="video"', rule_text)
        self.assertIn("After=systemd-udevd.service systemd-udev-trigger.service", service_text)
        self.assertIn("Before=NetworkManager.service", service_text)
        self.assertIn("EnvironmentFile=/etc/default/autolabor-usb-coldplug", service_text)
        self.assertIn("ExecStart=/usr/local/sbin/autolabor-usb-coldplug", service_text)
        for module in ("ax88179_178a", "cdc_ether", "ftdi_sio", "ch341", "uvcvideo"):
            self.assertIn(module, helper_text)
        self.assertIn("matches_configured_identity", helper_text)
        self.assertIn("reset_stuck_mid360_adapter", helper_text)
        self.assertIn("usb_device_has_carrier", helper_text)
        self.assertIn("AUTOLABOR_MID360_USB_SERIAL", helper_text)
        self.assertIn("install -m 0755", installer_text)
        self.assertIn("AUTOLABOR_J6M_USB_SERIAL", installer_text)
        self.assertIn("AUTOLABOR_MID360_USB_SERIAL", installer_text)
        self.assertIn("systemctl enable autolabor-zed-coldplug.service", installer_text)
        self.assertIn("systemctl restart autolabor-zed-coldplug.service", installer_text)

    def test_j6m_uses_mid360_for_fastlio_and_optional_ld19_for_scan(self):
        text = self._launch_text("j6m_fastlio_navigation.launch")
        self.assertIn('/gateway/livox/lidar /livox/lidar', text)
        self.assertIn('/gateway/livox/imu /livox/imu', text)
        self.assertIn('output_cmd_vel_topic" value="/cmd_vel_navigation', text)
        self.assertIn('output_cmd_topic" value="/cmd_vel_safe', text)
        self.assertIn('dual_lidar_scan" value="/dual_lidar/scan', text)
        self.assertIn('/cloud_registered_body_enhanced', text)
        self.assertIn('<arg name="mid360_sensor_x" default="0.20"/>', text)
        self.assertIn('<arg name="mid360_sensor_z" default="1.0"/>', text)
        self.assertIn("fast_lio_lidar_in_body_z", text)
        self.assertIn('<arg name="body_tf_z" value="$(eval -(', text)
        self.assertIn('<arg name="sensor_x" value="$(arg mid360_sensor_x)"/>', text)
        self.assertIn('<arg name="mid360_crop_enabled" default="true"/>', text)
        self.assertIn('<arg name="mid360_crop_min_x" default="-0.75"/>', text)
        self.assertIn('<arg name="crop_min_x" value="$(arg mid360_crop_min_x)"/>', text)

    def test_livox_config_keeps_sensor_and_host_addresses_separate(self):
        path = os.path.join(PACKAGE_DIR, "config", "livox_mid360_nvidia.json")
        with open(path, "r", encoding="utf-8") as stream:
            config = json.load(stream)
        host = config["MID360"]["host_net_info"]
        self.assertEqual("192.168.1.50", host["point_data_ip"])
        self.assertEqual("192.168.1.112", config["lidar_configs"][0]["ip"])

    def test_j6m_release_bundles_static_localization_runtime(self):
        deploy_path = os.path.join(WORKSPACE_DIR, "scripts", "deploy_j6m.sh")
        with open(deploy_path, "r", encoding="utf-8") as stream:
            deploy = stream.read()
        for runtime_path in (
            "./lib/map_server",
            "./lib/libmap_server_image_loader.so",
            "./share/map_server",
        ):
            self.assertIn(runtime_path, deploy)
        self.assertIn("./src/localization_fastlio/FAST_LIO", deploy)
        self.assertIn("./src/localization_fastlio/fast_lio_localization", deploy)
        self.assertIn(
            "fast_lio\\\\;fast_lio_localization\\\\;autolabor_coverage\\\\;robot_bringup",
            deploy,
        )
        self.assertIn("./src/application/autolabor_coverage", deploy)
        self.assertIn("rospack find autolabor_coverage", deploy)
        self.assertNotIn("./lib/amcl", deploy)
        self.assertIn("rospack find map_server", deploy)
        self.assertIn("ldd /opt/autolabor/dual_host/releases/", deploy)
        for soname in (
            "libyaml-cpp.so.0.6",
            "libSDL-1.2.so.0",
            "libSDL_image-1.2.so.0",
            "libpulsecommon-13.99.so",
            "libsndfile.so.1",
            "libFLAC.so.8",
        ):
            self.assertIn(soname, deploy)
        self.assertIn('rsync -aL "${navigation_system_libraries[@]}"', deploy)
        self.assertIn("if ldd /opt/autolabor/dual_host/releases/", deploy)
        self.assertIn(
            "-DFAST_LIO_RUNTIME_DIR=/var/lib/autolabor/fast_lio/", deploy
        )
        self.assertIn("grep -aFq /var/lib/autolabor/fast_lio/", deploy)
        self.assertNotIn("strings /opt/autolabor/dual_host/releases/", deploy)

        remote_health_path = os.path.join(
            WORKSPACE_DIR, "deploy", "j6m", "health_check.sh"
        )
        with open(remote_health_path, "r", encoding="utf-8") as stream:
            remote_health = stream.read()
        self.assertIn(
            "/opt/autolabor/dual_host/current/lib/fast_lio/fastlio_mapping",
            remote_health,
        )
        self.assertNotIn(
            "/opt/autolabor/ros/install/lib/fast_lio/fastlio_mapping",
            remote_health,
        )
        self.assertIn("/var/lib/autolabor/fast_lio/Log", remote_health)
        self.assertIn("grep -aFq /var/lib/autolabor/fast_lio/", remote_health)
        for interface in (
            "rosmsg md5 autolabor_coverage/CoverageRegion",
            "rosmsg md5 autolabor_coverage/CoverageStatus",
            "rossrv md5 autolabor_coverage/PlanCoverage",
            "rossrv md5 autolabor_coverage/CancelCoverageBatch",
            "rossrv md5 autolabor_coverage/StartCoverageBatch",
        ):
            self.assertIn(interface, deploy)
            self.assertIn(interface, remote_health)

    def test_persistent_can_fault_logging_is_throttled(self):
        driver_path = os.path.join(
            WORKSPACE_DIR,
            "src",
            "autolabor_core",
            "autolabor_canbus_driver",
            "autolabor_canbus_driver",
            "src",
            "m2_driver.cpp",
        )
        with open(driver_path, "r", encoding="utf-8") as stream:
            driver = stream.read()
        monitor_case = driver[
            driver.index("case Autocan::Vcu::ControllerMonitor:"):
            driver.index("case Autocan::Vcu::ControlTimeout:")
        ]
        self.assertIn("ROS_WARN_THROTTLE(5.0", monitor_case)
        self.assertNotIn('ROS_WARN("VCU ControllerMonitor', monitor_case)

    def test_dual_lidar_role_aliases_follow_verified_physical_ports(self):
        rules_path = os.path.join(
            WORKSPACE_DIR, "deploy", "99-autolabor-dual-lidar.rules"
        )
        installer_path = os.path.join(
            WORKSPACE_DIR, "scripts", "install_dual_lidar_udev.sh"
        )
        example_path = os.path.join(
            WORKSPACE_DIR, "config", "dual_host.env.example"
        )
        with open(rules_path, "r", encoding="utf-8") as stream:
            rules = stream.read()
        with open(installer_path, "r", encoding="utf-8") as stream:
            installer = stream.read()
        with open(example_path, "r", encoding="utf-8") as stream:
            example = stream.read()

        self.assertIn(
            'ID_PATH}=="platform-3610000.xhci-usb-0:4.4:1.0"', rules
        )
        self.assertIn('SYMLINK+="autolabor/lidar_front"', rules)
        self.assertIn(
            'ID_PATH}=="platform-3610000.xhci-usb-0:4.3:1.0"', rules
        )
        self.assertIn('SYMLINK+="autolabor/lidar_rear"', rules)
        self.assertIn("udevadm control --reload-rules", installer)
        self.assertIn("udevadm trigger --subsystem-match=tty", installer)
        self.assertIn("FRONT_LIDAR_PORT=/dev/autolabor/lidar_front", example)
        self.assertIn("REAR_LIDAR_PORT=/dev/autolabor/lidar_rear", example)

    def test_nvidia_ui_binds_saved_regions_to_the_selected_static_map(self):
        with open(
            os.path.join(WORKSPACE_DIR, "scripts", "nvidia_ui.sh"),
            "r",
            encoding="utf-8",
        ) as stream:
            ui_text = stream.read()
        for launch_argument in (
            'static_map_set:="${STATIC_MAP_SET:-}"',
            'static_map_source_mode:="${STATIC_MAP_SOURCE_MODE:-fused}"',
            'coverage_region_root:="${STATIC_MAP_SET:-}"',
            'coverage_region_legacy_root:="$DUAL_HOST_WS/global_maps/coverage_regions"',
        ):
            self.assertIn(launch_argument, ui_text)

        launch_path = os.path.join(
            WORKSPACE_DIR,
            "src/application/autolabor_operator_gui/launch/operator_gui.launch",
        )
        launch_root = ET.parse(launch_path).getroot()
        argument_names = {
            item.attrib.get("name") for item in launch_root.findall("arg")
        }
        self.assertTrue(
            {
                "static_map_set",
                "static_map_source_mode",
                "coverage_region_root",
                "coverage_region_legacy_root",
            }
            <= argument_names
        )
        gui_node = launch_root.find("node[@pkg='autolabor_operator_gui']")
        self.assertIsNotNone(gui_node)
        parameters = {
            item.attrib.get("name"): item.attrib.get("value")
            for item in gui_node.findall("param")
        }
        self.assertEqual("$(arg static_map_set)", parameters.get("static_map_set"))
        self.assertEqual(
            "$(arg static_map_source_mode)",
            parameters.get("static_map_source_mode"),
        )
        self.assertEqual(
            "$(arg coverage_region_root)",
            parameters.get("coverage_region_root"),
        )
        self.assertEqual(
            "$(arg coverage_region_legacy_root)",
            parameters.get("coverage_region_legacy_root"),
        )

    def test_shutdown_is_synchronous_and_verifies_residuals(self):
        def script_text(relative_path):
            with open(
                os.path.join(WORKSPACE_DIR, relative_path), "r", encoding="utf-8"
            ) as stream:
                return stream.read()

        stop_text = script_text("scripts/stop_dual_host.sh")
        control_text = script_text("scripts/process_control.sh")
        gateway_text = script_text("scripts/nvidia_gateway.sh")
        ui_text = script_text("scripts/nvidia_ui.sh")
        remote_text = script_text("deploy/j6m/stop.sh")
        start_text = script_text("scripts/start_dual_host.sh")
        config_text = script_text("scripts/load_config.sh")

        self.assertIn("dual_host_stop_pid_file", control_text)
        self.assertIn("dual_host_wait_for_records", control_text)
        self.assertIn("dual_host_process_start_ticks", control_text)
        self.assertIn("dual_host_process_tree_records", control_text)
        self.assertIn("dual_host_child_pids", control_text)
        self.assertIn("dual_host_process_ignores_interrupt", control_text)
        self.assertIn("dual_host_collect_tagged_process_records", control_text)
        self.assertIn("dual_host_collect_workspace_process_records", control_text)
        self.assertIn("dual_host_stop_records", control_text)
        self.assertIn("sending TERM", control_text)
        self.assertIn("sending KILL", control_text)
        self.assertIn("recover_managed_nvidia_orphans", stop_text)
        self.assertIn("managed_nvidia_legacy_command", stop_text)
        self.assertIn("cleanup_stale_ros_nodes.py", stop_text)
        self.assertIn("--node", stop_text)
        self.assertIn('fuser "$CAN_PORT"', stop_text)
        self.assertLess(stop_text.index("start_nvidia.sh"), stop_text.index("dual_host_select_ssh"))
        self.assertIn("nvidia_gateway.child.pid", gateway_text)
        self.assertIn("nvidia_ui.children", ui_text)
        self.assertIn("wait_for_zed_image", ui_text)
        self.assertIn("/fod_camera/image_raw", ui_text)
        self.assertIn('serial_number:="$NVIDIA_ZED_SERIAL"', ui_text)
        self.assertIn("dual_host_stop_pid_file", remote_text)
        self.assertIn("j6m_launcher.pid", remote_text)
        self.assertIn("mountpoint -q", remote_text)
        self.assertIn("systemd-run --user", start_text)
        self.assertIn("KillMode=control-group", start_text)
        self.assertIn("DUAL_HOST_RUN_TOKEN", start_text)
        self.assertIn("wait_for_managed_service", start_text)
        self.assertIn("zed_camera_check.sh", start_text)
        self.assertIn("mandatory runtime check failed", start_text)
        self.assertIn("performing one complete cold restart", start_text)
        self.assertIn(
            'health_check.sh" --runtime --allow-missing-data', start_text
        )
        self.assertIn("verify_j6m_static_localization_release", start_text)
        self.assertIn("fast_lio_localization/fast_lio_map_localizer", start_text)
        self.assertLess(
            start_text.index("verify_j6m_static_localization_release \"$target\""),
            start_text.index('sync_static_map.sh\" \"$target\"'),
        )
        self.assertIn("dual_host_find_interface_by_mac", config_text)
        self.assertIn("NVIDIA_LIVOX_MAC", config_text)

        combined = "\n".join((control_text, gateway_text, ui_text, stop_text))
        executable_lines = "\n".join(
            line for line in combined.splitlines() if not line.lstrip().startswith("#")
        )
        for forbidden in ("pkill ", "killall ", "dual_host_stop_matching"):
            self.assertNotIn(forbidden, executable_lines)
        self.assertNotIn("ps -eo", control_text)

    def test_orphan_discovery_requires_exact_token_or_workspace_provenance(self):
        control_path = os.path.join(WORKSPACE_DIR, "scripts", "process_control.sh")

        def proc_stat(pid, parent, start_ticks):
            fields = ["S", str(parent)] + ["0"] * 17 + [str(start_ticks)]
            return "{} (managed node) {}\n".format(pid, " ".join(fields))

        with tempfile.TemporaryDirectory() as temporary:
            proc_root = os.path.join(temporary, "proc")
            os.makedirs(proc_root)
            token_file = os.path.join(temporary, "run.token")
            with open(token_file, "w", encoding="utf-8") as stream:
                stream.write("01234567-89ab-cdef-0123-456789abcdef\n")

            processes = {
                5000: {
                    "uid": 1000,
                    "token": "01234567-89ab-cdef-0123-456789abcdef",
                    "workspace": "/workspace",
                    "master": "http://192.168.10.100:11311",
                },
                5001: {
                    "uid": 1000,
                    "token": "wrong-token-value",
                    "workspace": "/workspace",
                    "master": "http://192.168.10.100:11311",
                },
                5002: {
                    "uid": 1000,
                    "token": "01234567-89ab-cdef-0123-456789abcdef",
                    "workspace": "/unrelated",
                    "master": "http://192.168.10.100:11311",
                },
                5003: {
                    "uid": 2000,
                    "token": "01234567-89ab-cdef-0123-456789abcdef",
                    "workspace": "/workspace",
                    "master": "http://192.168.10.100:11311",
                },
            }
            for pid, values in processes.items():
                process_dir = os.path.join(proc_root, str(pid))
                os.makedirs(process_dir)
                with open(os.path.join(process_dir, "stat"), "w", encoding="utf-8") as stream:
                    stream.write(proc_stat(pid, 1, pid + 100))
                with open(os.path.join(process_dir, "status"), "w", encoding="utf-8") as stream:
                    stream.write("Uid:\t{0}\t{0}\t{0}\t{0}\nSigIgn:\t0\n".format(values["uid"]))
                with open(os.path.join(process_dir, "cmdline"), "wb") as stream:
                    stream.write(
                        b"/workspace/devel/lib/example/managed_node\0__name:=managed\0"
                    )
                environment = {
                    "DUAL_HOST_RUN_TOKEN": values["token"],
                    "DUAL_HOST_WS": values["workspace"],
                    "ROS_MASTER_URI": values["master"],
                }
                with open(os.path.join(process_dir, "environ"), "wb") as stream:
                    stream.write(
                        b"\0".join(
                            "{}={}".format(key, value).encode("utf-8")
                            for key, value in environment.items()
                        )
                        + b"\0"
                    )

            script = """
set -e
DUAL_HOST_PROC_ROOT={proc_root}
DUAL_HOST_MANAGED_UID=1000
source {control}
legacy_matcher() {{ [[ "$2" == /workspace/devel/lib/example/managed_node\\ * ]]; }}
echo tagged
dual_host_collect_tagged_process_records {token_file} /workspace
echo legacy
dual_host_collect_workspace_process_records \\
  /workspace http://192.168.10.100:11311 legacy_matcher
""".format(
                proc_root=shlex.quote(proc_root),
                control=shlex.quote(control_path),
                token_file=shlex.quote(token_file),
            )
            result = subprocess.run(
                ["bash", "-c", script],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(
            "tagged\n5000:5100\nlegacy\n5000:5100\n5001:5101\n",
            result.stdout,
        )

    def test_orphan_discovery_excludes_its_process_substitution_scanner(self):
        control_path = os.path.join(WORKSPACE_DIR, "scripts", "process_control.sh")
        token = "scanner-self-test-0123456789abcdef"
        with tempfile.TemporaryDirectory() as temporary:
            token_file = os.path.join(temporary, "run.token")
            with open(token_file, "w", encoding="utf-8") as stream:
                stream.write(token + "\n")
            script = """
set -euo pipefail
source {control}
mapfile -t records < <(
  dual_host_collect_tagged_process_records {token_file} {workspace}
)
printf '%s\n' "${{#records[@]}}"
""".format(
                control=shlex.quote(control_path),
                token_file=shlex.quote(token_file),
                workspace=shlex.quote(WORKSPACE_DIR),
            )
            environment = os.environ.copy()
            environment["DUAL_HOST_RUN_TOKEN"] = token
            environment["DUAL_HOST_WS"] = WORKSPACE_DIR
            result = subprocess.run(
                ["bash", "-c", script],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=environment,
            )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual("0\n", result.stdout)

    def _run_mocked_process_stop(self, wait_body, command="/opt/test/managed.sh"):
        control_path = os.path.join(WORKSPACE_DIR, "scripts", "process_control.sh")
        with tempfile.TemporaryDirectory() as temporary:
            pid_file = os.path.join(temporary, "managed.pid")
            signal_log = os.path.join(temporary, "signals.log")
            with open(pid_file, "w", encoding="utf-8") as stream:
                stream.write("4242 77\n")
            script = """
set -e
source {control}
dual_host_process_is_running() {{ [[ "$1" == 4242 || "$1" == 4243 ]]; }}
dual_host_process_start_ticks() {{
  case "$1" in 4242) echo 77 ;; 4243) echo 78 ;; *) return 1 ;; esac
}}
dual_host_pid_command() {{
  case "$1" in 4242) printf '%s\\n' {command} ;; 4243) echo /opt/test/child ;; esac
}}
dual_host_pid_is_self_or_ancestor() {{ return 1; }}
dual_host_process_tree_records() {{ printf '%s\\n' 4242:77 4243:78; }}
dual_host_send_signal() {{ printf '%s %s\\n' "$1" "$2" >>{signal_log}; }}
{wait_body}
dual_host_stop_pid_file {pid_file} mocked '(^|/)managed\\.sh([[:space:]]|$)'
""".format(
                control=shlex.quote(control_path),
                command=shlex.quote(command),
                signal_log=shlex.quote(signal_log),
                wait_body=wait_body,
                pid_file=shlex.quote(pid_file),
            )
            result = subprocess.run(
                ["bash", "-c", script],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            signals = ""
            if os.path.exists(signal_log):
                with open(signal_log, "r", encoding="utf-8") as stream:
                    signals = stream.read()
            return result, signals, os.path.exists(pid_file)

    def test_shutdown_mock_signals_only_recorded_tree(self):
        wait_body = """
wait_calls=0
dual_host_record_is_running() { (( wait_calls < 3 )); }
dual_host_wait_for_records() {
  wait_calls=$((wait_calls + 1))
  (( wait_calls >= 3 ))
}
"""
        result, signals, pid_file_exists = self._run_mocked_process_stop(wait_body)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(
            "INT 4242\nTERM 4242\nTERM 4243\nKILL 4242\nKILL 4243\n",
            signals,
        )
        self.assertFalse(pid_file_exists)

    def test_shutdown_mock_never_signals_unrelated_pid_file_owner(self):
        result, signals, pid_file_exists = self._run_mocked_process_stop(
            "dual_host_wait_for_records() { return 0; }",
            command="/usr/lib/gnome-session-binary",
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual("", signals)
        self.assertFalse(pid_file_exists)

    def test_shutdown_skips_int_when_recorded_root_inherited_ignored_signal(self):
        wait_body = """
dual_host_process_ignores_interrupt() { return 0; }
dual_host_record_is_running() { return 0; }
dual_host_wait_for_records() { return 0; }
"""
        result, signals, pid_file_exists = self._run_mocked_process_stop(wait_body)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual("TERM 4242\nTERM 4243\n", signals)
        self.assertFalse(pid_file_exists)

    def test_process_tree_falls_back_to_proc_ppids_without_children_file(self):
        control_path = os.path.join(WORKSPACE_DIR, "scripts", "process_control.sh")

        def proc_stat(pid, parent, start_ticks):
            fields = ["S", str(parent)] + ["0"] * 17 + [str(start_ticks)]
            return "{} (test process) {}\n".format(pid, " ".join(fields))

        with tempfile.TemporaryDirectory() as proc_root:
            processes = {
                5000: (1, 11),
                5001: (5000, 12),
                5002: (5001, 13),
                6000: (1, 14),
            }
            for pid, (parent, start_ticks) in processes.items():
                process_dir = os.path.join(proc_root, str(pid))
                os.makedirs(process_dir)
                with open(
                    os.path.join(process_dir, "stat"), "w", encoding="utf-8"
                ) as stream:
                    stream.write(proc_stat(pid, parent, start_ticks))
                with open(
                    os.path.join(process_dir, "status"), "w", encoding="utf-8"
                ) as stream:
                    ignored = "0000000000000002" if pid == 5000 else "0"
                    stream.write("SigIgn:\t{}\n".format(ignored))

            script = """
set -e
DUAL_HOST_PROC_ROOT={proc_root}
source {control}
dual_host_process_tree_records 5000
if dual_host_process_ignores_interrupt 5000; then
  echo ignores_int=yes
else
  echo ignores_int=no
fi
""".format(
                proc_root=shlex.quote(proc_root),
                control=shlex.quote(control_path),
            )
            result = subprocess.run(
                ["bash", "-c", script],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(
            "5000:11\n5001:12\n5002:13\nignores_int=yes\n", result.stdout
        )

    def _load_stale_cleanup_module(self):
        path = os.path.join(WORKSPACE_DIR, "scripts", "cleanup_stale_ros_nodes.py")
        spec = importlib.util.spec_from_file_location("cleanup_stale_ros_nodes_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_stale_cleanup_only_touches_explicit_node_whitelist(self):
        module = self._load_stale_cleanup_module()
        master = mock.Mock()
        master.lookupNode.side_effect = lambda name: "http://192.168.10.50:1234/"
        module.rosgraph.Master = mock.Mock(return_value=master)
        module.rosnode.get_node_names = mock.Mock(
            return_value=["/managed_stale", "/unrelated_stale"]
        )
        module.rosnode.rosnode_ping = mock.Mock(return_value=False)
        module.rosnode.cleanup_master_blacklist = mock.Mock()
        module.resolved_addresses = lambda host: {host}
        argv = [
            "cleanup_stale_ros_nodes.py",
            "--host",
            "192.168.10.50",
            "--node",
            "/managed_stale",
            "--fail-if-live",
        ]
        with mock.patch.object(sys, "argv", argv):
            self.assertEqual(0, module.main())
        module.rosnode.cleanup_master_blacklist.assert_called_once_with(
            master, ["/managed_stale"]
        )
        master.lookupNode.assert_called_once_with("/managed_stale")

    def test_stale_cleanup_refuses_to_remove_live_managed_node(self):
        module = self._load_stale_cleanup_module()
        master = mock.Mock()
        master.lookupNode.return_value = "http://192.168.10.50:1234/"
        module.rosgraph.Master = mock.Mock(return_value=master)
        module.rosnode.get_node_names = mock.Mock(return_value=["/managed_live"])
        module.rosnode.rosnode_ping = mock.Mock(return_value=True)
        module.rosnode.cleanup_master_blacklist = mock.Mock()
        module.resolved_addresses = lambda host: {host}
        argv = [
            "cleanup_stale_ros_nodes.py",
            "--host",
            "192.168.10.50",
            "--node",
            "/managed_live",
            "--fail-if-live",
        ]
        with mock.patch.object(sys, "argv", argv):
            self.assertEqual(1, module.main())
        module.rosnode.cleanup_master_blacklist.assert_not_called()


if __name__ == "__main__":
    unittest.main()
