#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import io
import time
from unittest import mock
from fod_platform_protocol import decode_incoming

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from autolabor_canbus_driver.msg import ChassisStatusInfo
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String


SCRIPT = Path(__file__).resolve().parent / "fod_platform_link.py"
SPEC = importlib.util.spec_from_file_location("fod_cloud_pose_reporter", str(SCRIPT))
REPORTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORTER)


class FodCloudPoseReporterTest(unittest.TestCase):
    def test_heartbeat_matches_cmd_1008_field_contract(self):
        heartbeat = REPORTER.make_heartbeat(
            {
                "timestamp_ms": 1000,
                "local_timestamp_ms": 1010,
                "device": "100",
                "longitude": 104.1,
                "latitude": 30.2,
                "course_angle": 6.5,
                "roll": 1.0,
                "pitch": -0.1,
                "status": REPORTER.STATUS_RUNNING,
                "error_code": 0,
                "speed": 2.5,
                "mode": 1,
                "soc": 43.0,
            }
        )
        self.assertEqual(heartbeat["CMD"], "1008")
        self.assertEqual(heartbeat["CHECK"], "0")
        self.assertEqual(heartbeat["RESERVED"], "0")
        self.assertEqual(len(heartbeat["DATA"]), 1)
        data = heartbeat["DATA"][0]
        self.assertEqual(data["DEVICE"], "100")
        self.assertEqual(data["STATUS"], 1)
        self.assertEqual(data["LON"], 104.1)
        self.assertEqual(data["LAT"], 30.2)

    def test_route_status_uses_documented_task_codes(self):
        self.assertEqual(REPORTER.route_status_code({"active": True}), 1)
        self.assertEqual(REPORTER.route_status_code({"state": "COMPLETE"}), 6)
        self.assertEqual(REPORTER.route_status_code({"state": "ABORTED"}), 7)
        self.assertEqual(REPORTER.route_status_code({"state": "IDLE"}), 9)

    def test_active_action_has_priority_over_old_terminal_entries(self):
        message = GoalStatusArray()
        failed = GoalStatus()
        failed.status = GoalStatus.ABORTED
        active = GoalStatus()
        active.status = GoalStatus.ACTIVE
        message.status_list = [failed, active]
        self.assertEqual(REPORTER.action_status_code(message), 1)

    def test_fod_mode_keeps_whole_recovery_running_and_reports_fault(self):
        for state in ("ENTERING_FOD", "FOD_ACTIVE", "FOD_VERIFY", "RETURNING_GPS"):
            self.assertEqual(REPORTER.fod_mode_status_code(state), 1)
        self.assertEqual(REPORTER.fod_mode_status_code("FOD_ABORTED"), 7)
        self.assertEqual(REPORTER.fod_mode_status_code("FAULT_STOP"), 7)
        self.assertEqual(REPORTER.fod_mode_status_code("FOD_COMPLETE_STOP"), 6)
        self.assertIsNone(REPORTER.fod_mode_status_code("GPS_ACTIVE"))

    def test_heading_and_level_orientation_are_normalized(self):
        self.assertAlmostEqual(REPORTER.normalize_heading_degrees(-10.0), 350.0)
        quaternion = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        roll, pitch = REPORTER.quaternion_roll_pitch_degrees(quaternion)
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)

    def test_vehicle_snapshot_combines_live_ros_inputs(self):
        state = REPORTER.VehicleState("100", 1, 2.0, 2.0, 2.0, 5.0)
        fix = NavSatFix()
        fix.header.stamp = REPORTER.rospy.Time.from_sec(time.time())
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude = 30.2
        fix.longitude = 104.1
        state.fix_callback(fix)
        state.heading_callback(Float64(data=370.0))
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 2.5
        state.odom_callback(odom)
        state.battery_callback(ChassisStatusInfo(battery_percent=43.0))
        state.route_status_callback(String(data='{"active": true, "state": "ACTIVE_FINAL"}'))
        state.fod_mode_state_callback(String(data="FOD_ACTIVE"))

        snapshot = state.snapshot()
        self.assertIsNotNone(snapshot)
        self.assertEqual(snapshot["error_code"], 0)
        self.assertEqual(snapshot["status"], REPORTER.STATUS_RUNNING)
        self.assertAlmostEqual(snapshot["course_angle"], 10.0)
        self.assertAlmostEqual(snapshot["speed"], 2.5)
        self.assertAlmostEqual(snapshot["soc"], 43.0)

        state.fod_mode_state_callback(String(data="FAULT_STOP"))
        self.assertEqual(state.snapshot()["status"], REPORTER.STATUS_FAILED)

    def test_publish_uses_required_exchange_and_json_body(self):
        class FakeChannel:
            def __init__(self):
                self.call = None

            def basic_publish(self, **kwargs):
                self.call = kwargs

        class FakeConnection:
            def process_data_events(self):
                pass

        reporter = REPORTER.CloudPoseReporter.__new__(REPORTER.CloudPoseReporter)
        reporter.exchange = "FOD_CAR_HEARTBEAT"
        reporter.routing_key = ""
        reporter.command = "1008"
        reporter.device = 'FOD-102'
        reporter.outbox_log = io.StringIO()
        reporter.channel = FakeChannel()
        reporter.connection = FakeConnection()
        reporter.published_count = 0
        reporter.publish_status = lambda *args, **kwargs: None
        snapshot = {
            "timestamp_ms": 1000,
            "local_timestamp_ms": 1010,
            "device": "100",
            "longitude": 104.1,
            "latitude": 30.2,
            "course_angle": 6.5,
            "roll": 0.0,
            "pitch": 0.0,
            "status": 1,
            "error_code": 0,
            "speed": 2.5,
            "mode": 1,
            "soc": 43.0,
        }
        reporter.publish_once(snapshot)
        self.assertEqual(reporter.channel.call["exchange"], "FOD_CAR_HEARTBEAT")
        self.assertEqual(reporter.channel.call["routing_key"], "")
        body = json.loads(reporter.channel.call["body"].decode("utf-8"))
        self.assertEqual(body["CMD"], "1008")
        self.assertEqual(reporter.published_count, 1)
        self.assertTrue(reporter.channel.call['mandatory'])
        self.assertTrue(json.loads(reporter.outbox_log.getvalue())['broker_confirmed'])

    def test_invalid_or_stale_fix_is_not_reported_as_live_position(self):
        state = REPORTER.VehicleState('FOD-102', 1, 2., 2., 2., 5.)
        fix = NavSatFix(latitude=30., longitude=104.)
        fix.header.stamp = REPORTER.rospy.Time.from_sec(time.time())
        fix.status.status = -1
        state.fix_callback(fix)
        self.assertIsNone(state.snapshot())
        fix.status.status = 0
        state.fix_received -= 3.
        self.assertIsNone(state.snapshot())
        state.fix_callback(fix)
        fix.header.stamp = REPORTER.rospy.Time.from_sec(time.time()-5.)
        self.assertIsNone(state.snapshot())  # Fresh delivery of an old fix is still old.
        fix.latitude = 104.
        state.fix_callback(fix)
        self.assertIsNone(state.snapshot())

    def test_document_mission_and_other_vehicle_are_received_but_never_executed(self):
        body = dict(vehicleId='FOD-102', missionId='test', type='normal',
                    waypoints=[dict(lat=30.3, lon=104.4, action='go')])
        r = decode_incoming(json.dumps(body).encode(), 'FOD-102')
        self.assertEqual(r['kind'], 'mission')
        self.assertTrue(r['addressed_to_this_vehicle'])
        self.assertFalse(r['executed'])
        body['vehicleId'] = 'OTHER'
        self.assertFalse(decode_incoming(json.dumps(body).encode(), 'FOD-102')['addressed_to_this_vehicle'])

    def test_existing_target_and_valid_group_formats_keep_coordinate_order(self):
        for body in [dict(TARGETS=[dict(LAT='30.3', LON='104.4')]), [[dict(lat=30.3,lon=104.4)]]]:
            r = decode_incoming(json.dumps(body).encode(), 'FOD-102')
            self.assertEqual(r['points'][0]['longitude'],104.4)
            self.assertEqual(r['points'][0]['latitude'],30.3)

    def test_malformed_groups_swapped_coordinates_and_nan_are_rejected(self):
        for body in [b'{[[{}]]}', b'{"x":NaN}',
                     b'{"TARGETS":[{"LAT":104.4,"LON":30.3}]}',
                     b'{"waypoints":[{"lat":true,"lon":104.4}]}']:
            with self.assertRaises((ValueError, TypeError)):
                decode_incoming(body, 'FOD-102')

    def test_inbox_records_invalid_packet_before_acknowledging_own_copy(self):
        reporter = REPORTER.CloudPoseReporter.__new__(REPORTER.CloudPoseReporter)
        reporter.vehicle_id = 'FOD-102'
        reporter.received_count = reporter.valid_count = reporter.rejected_count = 0
        reporter.inbox_log = io.StringIO()
        reporter.inbox_pub = mock.Mock()
        reporter.publish_status = mock.Mock()
        channel = mock.Mock()
        method = SimpleNamespace(exchange='autopilot.mission.direct', routing_key='', delivery_tag=7, redelivered=False)
        with mock.patch.object(REPORTER.rospy.Time, 'now', return_value=REPORTER.rospy.Time(123)):
            reporter.receive(channel, method, SimpleNamespace(message_id='sample'), b'bad JSON')
        self.assertFalse(json.loads(reporter.inbox_log.getvalue())['valid'])
        self.assertEqual(reporter.rejected_count,1)
        channel.basic_ack.assert_called_once_with(delivery_tag=7)


if __name__ == "__main__":
    unittest.main()
