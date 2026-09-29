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

    def __init__(self, behavior="normal", fragmented=False, port=0):
        self.state = 0
        self.switches = 0
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
                            if self.behavior == "unknown_status":
                                self.send(connection, b"unexpected reply")
                            else:
                                reply = "EEFFC3010000000000020D" if self.state else "EEFFC3010000000000000D"
                                self.send(connection, bytes.fromhex(reply))
                        elif bytes(pending) == self.SWITCH:
                            pending.clear()
                            self.switches += 1
                            if self.behavior != "unchanged":
                                self.state = 1 - self.state
                            if self.behavior == "disconnect_after_switch":
                                break
                            if self.behavior != "lost_ack":
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


if __name__ == "__main__":
    unittest.main()
