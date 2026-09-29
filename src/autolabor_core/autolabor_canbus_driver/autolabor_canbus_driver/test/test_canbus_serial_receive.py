#!/usr/bin/env python3
"""Exercise the real serial driver using a PTY; never open robot hardware."""

import os
import pathlib
import pty
import subprocess
import tempfile
import threading
import time
import unittest

import roslib.packages
import rospy
import rostest
from autolabor_canbus_driver.msg import CanBusMessage


def serial_frame(msg_type, sequence):
    payload = sequence.to_bytes(4, "little") + bytes(4)
    # VCU 0, data frame, single frame ID; Dallas/Maxim CRC-8.
    body = bytes((0x31, 0x00, msg_type, 0x00)) + payload
    crc = 0
    for value in body:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8C if crc & 1 else 0)
    return bytes((0xFE,)) + body + bytes((crc,)), payload


class CanbusSerialReceiveTest(unittest.TestCase):
    def test_serial_batches_preserve_status_and_telemetry(self):
        master_fd, slave_fd = pty.openpty()
        port = os.ttyname(slave_fd)
        received = []
        lock = threading.Lock()

        def callback(msg):
            with lock:
                received.append((msg.node_type, msg.node_seq, msg.msg_type,
                                 bytes(msg.payload)))

        subscriber = rospy.Subscriber(
            "/canbus_serial_test/messages", CanBusMessage, callback,
            queue_size=4096,
        )
        driver = rospy.get_param("~driver_executable", "")
        if not driver:
            driver = roslib.packages.find_node(
                "autolabor_canbus_driver", "canbus_driver")[0]
        process = None
        with tempfile.TemporaryFile() as driver_log:
            try:
                process = subprocess.Popen([
                    str(pathlib.Path(driver).resolve()),
                    "__name:=canbus_serial_test_driver",
                    "_port_name:=" + port,
                    "canbus_msg:=/canbus_serial_test/messages",
                    "canbus_server:=/canbus_serial_test/query",
                ], stdout=driver_log, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + 5.0
                while subscriber.get_num_connections() == 0:
                    self.assertIsNone(process.poll(), "test driver exited")
                    self.assertLess(time.monotonic(), deadline,
                                    "test driver did not connect")
                    time.sleep(0.02)

                expected = []
                safety_types = (0x17, 0x18, 0x19, 0x80)
                # A real USB read contains up to 18 complete 14-byte frames.
                # Interleave sparse status replies with the high-rate data;
                # losing either must fail, even when telemetry still appears.
                for batch in range(60):
                    wire = bytearray()
                    for index in range(18):
                        kind = (safety_types[batch % 4] if index == 0 else
                                (0x20, 0x21, 0x22)[index % 3])
                        frame, payload = serial_frame(kind, batch * 18 + index)
                        wire.extend(frame)
                        expected.append((0x10, 0, kind, payload))
                    self.assertEqual(os.write(master_fd, wire), len(wire))
                    time.sleep(0.02)

                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    with lock:
                        if len(received) >= len(expected):
                            break
                    time.sleep(0.02)
                with lock:
                    actual = list(received)
                self.assertEqual(len(actual), len(expected),
                                 "valid serial frames were lost before ROS delivery")
                self.assertEqual(actual, expected)
            finally:
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                subscriber.unregister()
                os.close(master_fd)
                os.close(slave_fd)


if __name__ == "__main__":
    rospy.init_node("canbus_serial_receive_test")
    rostest.rosrun("autolabor_canbus_driver", "canbus_serial_receive",
                  CanbusSerialReceiveTest)
