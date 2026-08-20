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

    def test_nvidia_owns_only_final_driver_topic(self):
        text = self._launch_text("nvidia_gateway.launch")
        self.assertIn('<param name="input_topic" value="/cmd_vel_safe"/>', text)
        self.assertIn('<param name="output_topic" value="/cmd_vel"/>', text)
        self.assertIn('<arg name="motion_enabled" default="false"/>', text)
        self.assertIn('/gateway/livox/lidar', text)
        self.assertIn('/gateway/livox/imu', text)

    def test_j6m_uses_mid360_for_fastlio_and_optional_ld19_for_scan(self):
        text = self._launch_text("j6m_fastlio_navigation.launch")
        self.assertIn('/gateway/livox/lidar /livox/lidar', text)
        self.assertIn('/gateway/livox/imu /livox/imu', text)
        self.assertIn('output_cmd_vel_topic" value="/cmd_vel_navigation', text)
        self.assertIn('output_cmd_topic" value="/cmd_vel_safe', text)
        self.assertIn('dual_lidar_scan" value="/dual_lidar/scan', text)
        self.assertIn('/cloud_registered_body_enhanced', text)
        self.assertIn('<arg name="mid360_sensor_x" default="0.20"/>', text)
        self.assertIn('<arg name="body_tf_x" value="$(eval -float(arg(\'mid360_sensor_x\')))"/>', text)
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
        self.assertIn("fast_lio\\\\;fast_lio_localization\\\\;robot_bringup", deploy)
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
        self.assertIn("dual_host_stop_pid_file", remote_text)
        self.assertIn("j6m_launcher.pid", remote_text)
        self.assertIn("mountpoint -q", remote_text)
        self.assertIn("systemd-run --user", start_text)
        self.assertIn("KillMode=control-group", start_text)
        self.assertIn("DUAL_HOST_RUN_TOKEN", start_text)
        self.assertIn("wait_for_managed_service", start_text)
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
