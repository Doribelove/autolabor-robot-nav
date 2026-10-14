#!/usr/bin/env python3
"""Loopback TCP tests: no connection to the real cleaning device."""

import contextlib
import io
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from sweep_test import DeviceError, SweepTester, main, menu


class DeviceServer:
    QUERY = bytes.fromhex("CCDDC30100000DCE9C")
    SWITCH = bytes.fromhex("CCDDD30100000000000100000000000103E8DDCC")
    HYDRAULIC = {
        bytes.fromhex("CCDDA20102030200C80400000076EC"): ("retract", 2),
        bytes.fromhex("CCDDA201020302012C04000000DBB6"): ("retract", 3),
        bytes.fromhex("CCDDA2010203000000040200C876EC"): ("extend", 2),
        bytes.fromhex("CCDDA20102030000000402012CDBB6"): ("extend", 3),
    }

    def __init__(self, behavior="normal", fragmented=False, port=0):
        self.state = 0
        self.switches = 0
        self.hydraulic = []
        self.commands = []
        self.behavior = behavior
        self.fragmented = fragmented
        self.stop = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", port))
        self.port = self.listener.getsockname()[1]
        self.listener.listen(2)
        self.listener.settimeout(0.1)
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.stop.set()
        self.listener.close()
        self.thread.join(timeout=2)

    def send(self, connection, data):
        if self.fragmented:
            for value in data:
                connection.sendall(bytes([value]))
                time.sleep(0.002)
        else:
            connection.sendall(data)

    def serve(self):
        while not self.stop.is_set():
            try:
                connection, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with connection:
                connection.settimeout(0.1)
                try:
                    connection.sendall(b"v1.0\r\n")
                    pending = bytearray()
                    while not self.stop.is_set():
                        try:
                            chunk = connection.recv(1024)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break
                        pending.extend(chunk)
                        if bytes(pending) == self.QUERY:
                            pending.clear()
                            self.commands.append(("query", time.monotonic()))
                            if self.behavior == "unknown_status":
                                self.send(connection, b"unexpected reply")
                            else:
                                reply = "EEFFC3010000000000020D" if self.state else "EEFFC3010000000000000D"
                                self.send(connection, bytes.fromhex(reply))
                        elif bytes(pending) == self.SWITCH:
                            pending.clear()
                            self.commands.append(("switch", time.monotonic()))
                            self.switches += 1
                            if self.behavior != "unchanged":
                                self.state = 1 - self.state
                            if self.behavior == "disconnect_after_switch":
                                break
                            if self.behavior != "lost_ack":
                                self.send(connection, b"OK!")
                        elif bytes(pending) in self.HYDRAULIC:
                            action, duration = self.HYDRAULIC[bytes(pending)]
                            self.hydraulic.append((action, duration, time.monotonic()))
                            self.commands.append((action, time.monotonic()))
                            pending.clear()
                            if self.behavior == "hydraulic_disconnect":
                                break
                            if self.behavior in ("hydraulic_status_only", "hydraulic_status_then_ack"):
                                reply = "EEFFC3010000000000020D" if self.state else "EEFFC3010000000000000D"
                                self.send(connection, bytes.fromhex(reply))
                            if self.behavior not in ("hydraulic_lost_ack", "hydraulic_status_only"):
                                self.send(connection, b"OK!")
                except OSError:
                    pass


class SweepProtocolTest(unittest.TestCase):
    def setUp(self):
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()

    def tearDown(self):
        self.output.__exit__(None, None, None)

    def client(self, server):
        client = SweepTester("127.0.0.1", server.port, timeout=0.15, settle=0.2)
        self.addCleanup(client.close)
        return client

    def test_repeated_on_off_are_idempotent(self):
        with DeviceServer() as server:
            device = self.client(server)
            for wanted in (0, 1, 1, 0, 0):
                device.set_state(wanted)
                self.assertEqual(server.state, wanted)
            self.assertEqual(server.switches, 2)

    def test_fragmented_status_and_ack_in_menu(self):
        with DeviceServer(fragmented=True) as server:
            device = self.client(server)
            with patch("builtins.input", side_effect=["1", "s", "q"]):
                menu(device)
            self.assertEqual(server.state, 0)
            self.assertEqual(server.switches, 2)

    def test_missing_ack_does_not_repeat_toggle(self):
        with DeviceServer(behavior="lost_ack") as server:
            self.client(server).set_state(1)
            self.assertEqual(server.state, 1)
            self.assertEqual(server.switches, 1)

    def test_disconnect_after_switch_queries_new_connection(self):
        with DeviceServer(behavior="disconnect_after_switch") as server:
            self.client(server).set_state(1)
            self.assertEqual(server.state, 1)
            self.assertEqual(server.switches, 1)

    def test_ack_alone_is_not_reported_as_success(self):
        with DeviceServer(behavior="unchanged") as server:
            with self.assertRaises(DeviceError):
                self.client(server).set_state(1)
            self.assertEqual(server.switches, 1)

    def test_unknown_initial_state_never_toggles(self):
        with DeviceServer(behavior="unknown_status") as server:
            with self.assertRaises(DeviceError):
                self.client(server).set_state(1)
            self.assertEqual(server.switches, 0)

    def test_ctrl_c_in_menu_stops(self):
        with DeviceServer() as server:
            device = self.client(server)
            with patch("builtins.input", side_effect=["1", KeyboardInterrupt()]):
                menu(device)
            self.assertEqual(server.state, 0)
            self.assertEqual(server.switches, 2)

    def test_cli_status_only_reads(self):
        with DeviceServer() as server:
            self.assertEqual(main(["status", "--ip", "127.0.0.1", "--port", str(server.port)]), 0)
            self.assertEqual(server.switches, 0)
            self.assertEqual(server.hydraulic, [])

    def test_hydraulic_exact_frames_and_sweep_state_unchanged(self):
        for action in ("extend", "retract"):
            for duration in (2, 3):
                with self.subTest(action=action, duration=duration), DeviceServer(fragmented=True) as server:
                    server.state = 1
                    device = self.client(server)
                    message = device.move_hydraulic(action, duration)
                    self.assertTrue(device.hydraulic_acknowledged)
                    self.assertIn("指令已接收", message)
                    self.assertEqual([(a, d) for a, d, _ in server.hydraulic], [(action, duration)])
                    self.assertEqual(server.state, 1)
                    self.assertEqual(server.switches, 0)
                    self.assertGreaterEqual(time.monotonic() - server.hydraulic[0][2], duration)

    def test_hydraulic_lost_ack_status_only_and_disconnect_never_resend(self):
        for behavior in ("hydraulic_lost_ack", "hydraulic_status_only", "hydraulic_disconnect"):
            with self.subTest(behavior=behavior), DeviceServer(behavior=behavior) as server:
                device = self.client(server)
                with self.assertRaisesRegex(DeviceError, "未自动重发"):
                    device.move_hydraulic("extend")
                self.assertFalse(device.hydraulic_acknowledged)
                self.assertEqual(len(server.hydraulic), 1)
                self.assertEqual(server.switches, 0)
                self.assertGreaterEqual(time.monotonic() - server.hydraulic[0][2], 1.95)

    def test_hydraulic_fragmented_ack_after_unrelated_status(self):
        with DeviceServer(behavior="hydraulic_status_then_ack", fragmented=True) as server:
            device = self.client(server)
            device.move_hydraulic("retract")
            self.assertTrue(device.hydraulic_acknowledged)
            self.assertEqual(len(server.hydraulic), 1)

    def test_hydraulic_invalid_input_never_connects(self):
        with DeviceServer() as server:
            device = self.client(server)
            for action, duration in (("extend", 1), ("retract", 4), ("unknown", 2)):
                with self.assertRaises(DeviceError):
                    device.move_hydraulic(action, duration)
                self.assertIsNone(device.sock)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(["extend", "--duration", "1", "--ip", "127.0.0.1", "--port", str(server.port)])
            self.assertEqual(error.exception.code, 2)
            self.assertEqual(server.hydraulic, [])

    def test_hydraulic_cli_defaults_to_two_seconds_and_reports_ack_only(self):
        with DeviceServer() as server:
            output = io.StringIO()
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(io.StringIO()):
                code = main(["extend", "--json", "--ip", "127.0.0.1", "--port", str(server.port)])
            self.assertEqual(code, 0)
            reply = json.loads(output.getvalue())
            self.assertTrue(reply["success"])
            self.assertTrue(reply["acknowledged"])
            self.assertFalse(reply["position_confirmed"])
            self.assertEqual(reply["duration_s"], 2)
            self.assertEqual(reply["state"], -1)
            self.assertEqual(server.hydraulic[0][:2], ("extend", 2))
            self.assertEqual(server.switches, 0)

    def test_concurrent_hydraulic_cli_waits_for_previous_pulse(self):
        script = Path(__file__).resolve().parents[2] / "scripts" / "sweep_test.py"
        with DeviceServer() as server:
            base = [sys.executable, str(script)]
            options = ["--json", "--ip", "127.0.0.1", "--port", str(server.port)]
            processes = [subprocess.Popen(base + [action] + options,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                         for action in ("extend", "retract")]
            try:
                for process in processes:
                    out, err = process.communicate(timeout=12)
                    self.assertEqual(process.returncode, 0, err + out)
                    self.assertTrue(json.loads(out)["acknowledged"])
                self.assertEqual(len(server.hydraulic), 2)
                self.assertEqual({a for a, _, _ in server.hydraulic}, {"extend", "retract"})
                self.assertGreaterEqual(server.hydraulic[1][2] - server.hydraulic[0][2], 2.0)
                self.assertEqual(server.switches, 0)
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.kill()
                        process.wait()

    def test_concurrent_cli_on_requests_toggle_once_and_report_ownership(self):
        script = Path(__file__).resolve().parents[2] / "scripts" / "sweep_test.py"
        with DeviceServer(fragmented=True) as server:
            command = [sys.executable, str(script), "on", "--json",
                       "--ip", "127.0.0.1", "--port", str(server.port)]
            processes = [subprocess.Popen(command, stdout=subprocess.PIPE,
                                          stderr=subprocess.PIPE, text=True)
                         for _ in range(2)]
            try:
                replies = []
                for process in processes:
                    out, err = process.communicate(timeout=12)
                    self.assertEqual(process.returncode, 0, err + out)
                    replies.append(json.loads(out))
                self.assertTrue(all(reply["success"] for reply in replies))
                self.assertEqual(sorted(reply["previous_state"] for reply in replies), [0, 1])
                self.assertEqual(server.state, 1)
                self.assertEqual(server.switches, 1)
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.kill()
                        process.wait()

    def test_recovery_start_and_stop_overlap_pulses_with_motor_changes(self):
        with DeviceServer(fragmented=True) as server:
            device = self.client(server)
            for enabled, action in ((True, "extend"), (False, "retract")):
                with self.subTest(enabled=enabled):
                    server.commands.clear()
                    requested = time.monotonic()
                    device.set_recovery(enabled)
                    self.assertEqual(server.state, int(enabled))
                    self.assertTrue(device.hydraulic_acknowledged)
                    self.assertEqual(server.hydraulic[-1][:2], (action, 3))
                    self.assertGreaterEqual(time.monotonic() - server.hydraulic[-1][2], 3.0)
                    actions = [(name, stamp) for name, stamp in server.commands if name != "query"]
                    self.assertEqual([a for a, _ in actions],
                                     ["extend", "switch"] if enabled else ["switch", "retract"])
                    self.assertLess(actions[1][1] - requested, 0.30,
                                    "fresh TCP connection must not impose a fixed half-second sleep")
                    self.assertLess(actions[1][1] - actions[0][1], 0.5,
                                    "motor and rod must overlap, not wait out the pulse")
            self.assertEqual(server.switches, 2)

    def test_recovery_preexisting_motor_on_still_extends_then_stops_and_retracts(self):
        with DeviceServer() as server:
            server.state = 1
            device = self.client(server)
            device.set_recovery(True)
            self.assertEqual(device.previous_state, 1)
            self.assertEqual(server.switches, 0)
            device.set_recovery(False)
            self.assertEqual(server.state, 0)
            self.assertEqual(server.switches, 1)
            self.assertEqual([(a, d) for a, d, _ in server.hydraulic],
                             [("extend", 3), ("retract", 3)])

    def test_recovery_lost_extension_ack_does_not_start_motor_or_repeat_pulse(self):
        with DeviceServer(behavior="hydraulic_lost_ack") as server:
            device = self.client(server)
            with self.assertRaisesRegex(DeviceError, "未自动重发"):
                device.set_recovery(True)
            self.assertTrue(device.hydraulic_sent)
            self.assertFalse(device.hydraulic_acknowledged)
            self.assertEqual(server.state, 0)
            self.assertEqual(server.switches, 0)
            self.assertEqual([(a, d) for a, d, _ in server.hydraulic], [("extend", 3)])
            self.assertGreaterEqual(time.monotonic() - server.hydraulic[0][2], 2.95)

    def test_recovery_failed_retraction_still_confirms_motor_off(self):
        with DeviceServer(behavior="hydraulic_lost_ack") as server:
            server.state = 1
            device = self.client(server)
            with self.assertRaisesRegex(DeviceError, "未自动重发"):
                device.set_recovery(False)
            self.assertEqual(device.last_state, 0)
            self.assertEqual(server.switches, 1)
            self.assertEqual([(a, d) for a, d, _ in server.hydraulic], [("retract", 3)])

    def test_recovery_failed_motor_stop_still_sends_one_retraction(self):
        with DeviceServer(behavior="unchanged") as server:
            server.state = 1
            device = self.client(server)
            with self.assertRaisesRegex(DeviceError, "未确认装置已关闭"):
                device.set_recovery(False)
            self.assertEqual(server.state, 1)
            self.assertEqual(server.switches, 1)
            self.assertTrue(device.hydraulic_acknowledged)
            self.assertEqual([(a, d) for a, d, _ in server.hydraulic], [("retract", 3)])

    def test_recovery_json_reports_partial_start_for_cleanup(self):
        with DeviceServer(behavior="unchanged") as server:
            output = io.StringIO()
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(io.StringIO()):
                code = main(["recovery-start", "--json", "--settle", "0.2",
                             "--ip", "127.0.0.1", "--port", str(server.port)])
            reply = json.loads(output.getvalue())
            self.assertEqual(code, 2)
            self.assertFalse(reply["success"])
            self.assertTrue(reply["hydraulic_sent"])
            self.assertTrue(reply["acknowledged"])
            self.assertEqual(reply["duration_s"], 3)
            self.assertFalse(reply["position_confirmed"])
            self.assertEqual(reply["state"], 0)

    def test_recovery_cli_holds_shared_lock_until_pulse_finishes(self):
        script = Path(__file__).resolve().parents[2] / "scripts" / "sweep_test.py"
        with DeviceServer() as server:
            options = ["--json", "--ip", "127.0.0.1", "--port", str(server.port)]
            first = subprocess.Popen([sys.executable, str(script), "recovery-start"] + options,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            second = None
            try:
                deadline = time.monotonic() + 3
                while not server.hydraulic and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(server.hydraulic)
                second = subprocess.Popen([sys.executable, str(script), "retract"] + options,
                                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                for process in (first, second):
                    out, err = process.communicate(timeout=12)
                    self.assertEqual(process.returncode, 0, err + out)
                    self.assertTrue(json.loads(out)["success"])
                self.assertEqual([(a, d) for a, d, _ in server.hydraulic],
                                 [("extend", 3), ("retract", 2)])
                self.assertGreaterEqual(server.hydraulic[1][2] - server.hydraulic[0][2], 3.0)
            finally:
                for process in (first, second):
                    if process is not None and process.poll() is None:
                        process.kill()
                        process.wait()


if __name__ == "__main__":
    unittest.main()
