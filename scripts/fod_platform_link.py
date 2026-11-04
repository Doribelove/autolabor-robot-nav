#!/usr/bin/env python3
"""FOD-102 position heartbeat and addressed tasks via existing GPS/FOD APIs.

Based on the existing robot_ws fod_cloud_pose_reporter. No cmd_vel publisher.
A broker ACK is not a platform UI acknowledgement or physical pickup proof.
"""

import json
import math
import os
import threading
import time
import base64
from pathlib import Path
import uuid

import pika
import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from autolabor_canbus_driver.msg import ChassisStatusInfo
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String
from platform_credentials import rabbitmq_credentials
from fod_platform_protocol import decode_incoming
from fod_platform_tasks import MissionRunner, RosNavigationBackend


STATUS_RUNNING = 1
STATUS_COMPLETE = 6
STATUS_FAILED = 7
STATUS_IDLE = 9

ERROR_GPS = 1
ERROR_HEADING = 2
ERROR_ODOMETRY = 4
ERROR_BATTERY = 8

RUNNING_ACTION_STATES = frozenset(
    (GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING)
)
FAILED_ACTION_STATES = frozenset(
    (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST)
)


def environment(primary, fallback, default):
    return os.environ.get(primary, os.environ.get(fallback, default))


def environment_bool(primary, default=False):
    value = os.environ.get(primary)
    if value is None:
        return bool(default)
    return value.strip().lower() in ("1", "true", "yes", "on")


def normalize_heading_degrees(value):
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return value % 360.0


def quaternion_roll_pitch_degrees(quaternion):
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)
    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)
    return math.degrees(roll), math.degrees(pitch)


def route_status_code(payload):
    if not isinstance(payload, dict):
        return None
    if bool(payload.get("active", False)):
        return STATUS_RUNNING
    state = str(payload.get("state", "")).strip().upper()
    if state == "COMPLETE":
        return STATUS_COMPLETE
    if state in ("ABORTED", "CANCELED", "REJECTED_NEW_GOAL", "FAULT_STOP"):
        return STATUS_FAILED
    if state.startswith("ACTIVE") or state in ("ADVANCING", "PAUSED"):
        return STATUS_RUNNING
    if state.startswith("IDLE") or not state:
        return STATUS_IDLE
    return STATUS_IDLE


def action_status_code(message):
    if message is None or not message.status_list:
        return None
    states = [int(item.status) for item in message.status_list]
    if any(state in RUNNING_ACTION_STATES for state in states):
        return STATUS_RUNNING
    latest = states[-1]
    if latest == GoalStatus.SUCCEEDED:
        return STATUS_COMPLETE
    if latest in FAILED_ACTION_STATES:
        return STATUS_FAILED
    return STATUS_IDLE


def fod_mode_status_code(state):
    state = str(state or "").strip().upper()
    if state in ("ENTERING_FOD", "FOD_ACTIVE", "FOD_VERIFY", "RETURNING_GPS"):
        return STATUS_RUNNING
    if state == "FOD_COMPLETE_STOP":
        return STATUS_COMPLETE
    if state in ("FOD_ABORTED", "FAULT_STOP"):
        return STATUS_FAILED
    return None


def make_heartbeat(data, command="1008"):
    """Build the compact JSON-compatible document shown in the field spec."""
    return {
        "CMD": str(command),
        "DATA": [
            {
                "timestamp": int(data["timestamp_ms"]),
                "localtimestamp": int(data["local_timestamp_ms"]),
                "DEVICE": str(data["device"]),
                "LON": float(data["longitude"]),
                "LAT": float(data["latitude"]),
                "COURSE_ANGLE": float(data["course_angle"]),
                "ROLL": float(data["roll"]),
                "PITCH": float(data["pitch"]),
                "STATUS": int(data["status"]),
                "ERRORCODE": int(data["error_code"]),
                "SPEED": float(data["speed"]),
                "MODE": int(data["mode"]),
                "SOC": float(data["soc"]),
            }
        ],
        "CHECK": "0",
        "RESERVED": "0",
    }


class VehicleState:
    def __init__(self, device, mode, gps_timeout, heading_timeout, odom_timeout,
                 battery_timeout):
        self.device = str(device)
        self.mode = int(mode)
        self.gps_timeout = float(gps_timeout)
        self.heading_timeout = float(heading_timeout)
        self.odom_timeout = float(odom_timeout)
        self.battery_timeout = float(battery_timeout)
        self.lock = threading.RLock()
        self.fix = None
        self.fix_received = None
        self.heading = None
        self.heading_received = None
        self.odom = None
        self.odom_received = None
        self.battery = None
        self.battery_received = None
        self.route_status = None
        self.action_status = None
        self.fod_mode_state = None

    def fix_callback(self, message):
        with self.lock:
            self.fix = message
            self.fix_received = time.monotonic()

    def heading_callback(self, message):
        with self.lock:
            self.heading = float(message.data)
            self.heading_received = time.monotonic()

    def odom_callback(self, message):
        with self.lock:
            self.odom = message
            self.odom_received = time.monotonic()

    def battery_callback(self, message):
        with self.lock:
            self.battery = float(message.battery_percent)
            self.battery_received = time.monotonic()

    def route_status_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            rospy.logwarn_throttle(10.0, "cloud pose reporter received invalid route status JSON")
            return
        with self.lock:
            self.route_status = payload

    def action_status_callback(self, message):
        with self.lock:
            self.action_status = message

    def fod_mode_state_callback(self, message):
        with self.lock:
            self.fod_mode_state = message.data

    @staticmethod
    def _fresh(received, now, timeout):
        return received is not None and 0.0 <= now - received <= timeout

    def snapshot(self):
        now_monotonic = time.monotonic()
        local_timestamp_ms = int(time.time() * 1000.0)
        with self.lock:
            fix = self.fix
            if (fix is None or not math.isfinite(fix.latitude) or not math.isfinite(fix.longitude)
                    or not (-90 <= fix.latitude <= 90 and -180 <= fix.longitude <= 180)
                    or fix.status.status < NavSatStatus.STATUS_FIX
                    or not self._fresh(self.fix_received, now_monotonic, self.gps_timeout)
                    or fix.header.stamp.to_sec() <= 0
                    or abs(local_timestamp_ms/1000.0-fix.header.stamp.to_sec()) > self.gps_timeout):
                return None

            error_code = 0
            if (not self._fresh(self.fix_received, now_monotonic, self.gps_timeout) or
                    fix.status.status < NavSatStatus.STATUS_FIX):
                error_code |= ERROR_GPS

            course_angle = 0.0
            if (self.heading is not None and math.isfinite(self.heading) and
                    self._fresh(self.heading_received, now_monotonic, self.heading_timeout)):
                course_angle = normalize_heading_degrees(self.heading)
            else:
                error_code |= ERROR_HEADING

            roll = 0.0
            pitch = 0.0
            speed = 0.0
            if self.odom is not None and self._fresh(
                    self.odom_received, now_monotonic, self.odom_timeout):
                roll, pitch = quaternion_roll_pitch_degrees(self.odom.pose.pose.orientation)
                linear = self.odom.twist.twist.linear
                speed = math.hypot(float(linear.x), float(linear.y))
                if not all(math.isfinite(value) for value in (roll, pitch, speed)):
                    roll = pitch = speed = 0.0
                    error_code |= ERROR_ODOMETRY
            else:
                error_code |= ERROR_ODOMETRY

            soc = 0.0
            if (self.battery is not None and math.isfinite(self.battery) and
                    self._fresh(self.battery_received, now_monotonic, self.battery_timeout)):
                soc = max(0.0, min(100.0, self.battery))
            else:
                error_code |= ERROR_BATTERY

            action_code = action_status_code(self.action_status)
            route_code = route_status_code(self.route_status)
            fod_mode_code = fod_mode_status_code(self.fod_mode_state)
            if fod_mode_code == STATUS_FAILED:
                status = STATUS_FAILED
            elif fod_mode_code == STATUS_RUNNING:
                status = STATUS_RUNNING
            elif fod_mode_code == STATUS_COMPLETE:
                status = STATUS_COMPLETE
            elif action_code == STATUS_RUNNING or route_code == STATUS_RUNNING:
                status = STATUS_RUNNING
            elif route_code in (STATUS_COMPLETE, STATUS_FAILED):
                status = route_code
            elif action_code in (STATUS_COMPLETE, STATUS_FAILED):
                status = action_code
            else:
                status = STATUS_IDLE

            stamp = fix.header.stamp.to_sec()
            timestamp_ms = int(stamp * 1000.0) if stamp > 0.0 else local_timestamp_ms
            return {
                "timestamp_ms": timestamp_ms,
                "local_timestamp_ms": local_timestamp_ms,
                "device": self.device,
                "longitude": fix.longitude,
                "latitude": fix.latitude,
                "course_angle": course_angle,
                "roll": roll,
                "pitch": pitch,
                "status": status,
                "error_code": error_code,
                "speed": speed,
                "mode": self.mode,
                "soc": soc,
            }


class CloudPoseReporter:
    def __init__(self):
        config_path = rospy.get_param('~config', str(Path(__file__).resolve().parents[1]/'config/fod_platform_link.json'))
        config = json.loads(Path(config_path).read_text())
        self.host = rospy.get_param(
            "~rabbitmq_host", environment("FOD_CLOUD_RABBITMQ_HOST", "RABBITMQ_HOST", "39.98.47.163"))
        self.port = int(rospy.get_param(
            "~rabbitmq_port", environment("FOD_CLOUD_RABBITMQ_PORT", "RABBITMQ_PORT", "5672")))
        local_user, local_password = rabbitmq_credentials()
        self.user = rospy.get_param(
            "~rabbitmq_user", environment("FOD_CLOUD_RABBITMQ_USER", "RABBITMQ_USER", local_user))
        self.password = rospy.get_param(
            "~rabbitmq_password", environment("FOD_CLOUD_RABBITMQ_PASSWORD", "RABBITMQ_PWD", local_password))
        self.virtual_host = rospy.get_param(
            "~rabbitmq_virtual_host", environment("FOD_CLOUD_RABBITMQ_VHOST", "RABBITMQ_VHOST", "/"))
        self.exchange = rospy.get_param(
            "~exchange", os.environ.get("FOD_HEARTBEAT_EXCHANGE", config['heartbeat_exchange']))
        self.exchange_type = rospy.get_param(
            "~exchange_type", os.environ.get("FOD_HEARTBEAT_EXCHANGE_TYPE", "fanout"))
        self.routing_key = rospy.get_param(
            "~routing_key", os.environ.get("FOD_HEARTBEAT_ROUTING_KEY", ""))
        self.declare_exchange = bool(rospy.get_param(
            "~declare_exchange", environment_bool("FOD_HEARTBEAT_DECLARE_EXCHANGE", False)))
        self.publish_rate = float(rospy.get_param("~publish_rate", config['publish_rate_hz']))
        self.retry_delay = float(rospy.get_param("~retry_delay", 5.0))
        self.command = str(rospy.get_param("~command", "1008"))
        self.device = str(rospy.get_param(
            "~device", os.environ.get("FOD_CLOUD_DEVICE", config['device'])))
        self.vehicle_id = str(rospy.get_param('~vehicle_id', config['vehicle_id']))
        self.receive_exchanges = list(config['receive_exchanges'])
        self.inbox_queue = str(config['inbox_queue'])
        self.log_directory = Path(rospy.get_param('~log_directory', config['log_directory']))
        self.log_directory.mkdir(parents=True, exist_ok=True)
        self.inbox_log = (self.log_directory/'received.jsonl').open('a', buffering=1)
        self.outbox_log = (self.log_directory/'published.jsonl').open('a', buffering=1)
        self.received_count, self.valid_count, self.rejected_count = 0, 0, 0
        self.last_received = None
        self.remote_execution = bool(rospy.get_param('~remote_execution', config.get('remote_execution', False)))
        self.task_status = {}
        self.task_pub = rospy.Publisher('/fod_platform/task_status', String, queue_size=10, latch=True)
        self.task_runner = None
        self.task_thread = None
        self.inbox_pub = rospy.Publisher('/fod_platform/inbox', String, queue_size=20, latch=True)
        self.mode = int(rospy.get_param("~mode", 1))
        if not self.exchange or not math.isfinite(self.publish_rate) or self.publish_rate <= 0.0:
            raise ValueError("exchange must be non-empty and publish_rate must be positive")

        self.state = VehicleState(
            self.device,
            self.mode,
            rospy.get_param("~gps_timeout", 2.0),
            rospy.get_param("~heading_timeout", 2.0),
            rospy.get_param("~odom_timeout", 2.0),
            rospy.get_param("~battery_timeout", 5.0),
        )
        self.status_publisher = rospy.Publisher(
            "/fod_platform/status", String, queue_size=1, latch=True)
        self.published_count = 0
        self.connection = None
        self.channel = None

        if self.remote_execution:
            from std_srvs.srv import Trigger, TriggerResponse
            backend = RosNavigationBackend()
            def task_update(value):
                self.task_status = value
                self.task_pub.publish(String(data=json.dumps(value, ensure_ascii=False)))
                with (self.log_directory/'tasks.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(system_ns=time.time_ns(), task=value), ensure_ascii=False)+'\n')
            self.task_runner = MissionRunner(backend, self.vehicle_id,
                config.get('mission_state_file', str(Path.home()/'.config/autolabor/platform_tasks.json')),
                task_update)
            backend.foreign_callback = lambda:self.task_runner.cancel('本地发送了新的 GPS 目标', takeover=True)
            backend.cancel_callback = lambda:self.task_runner.cancel('操作员取消导航')
            def cancel_task(_request):
                self.task_runner.cancel()
                return TriggerResponse(success=True, message='已取消排队任务，并请求当前远端任务停车收尾')
            self.cancel_task_service = rospy.Service('/fod_platform/cancel_task', Trigger, cancel_task)
            self.task_thread = threading.Thread(target=self.task_runner.run, daemon=True, name='remote-task-worker')
            self.task_thread.start()
            rospy.on_shutdown(self.stop_task_worker)
            task_update(self.task_runner.snapshot())

        rospy.Subscriber(rospy.get_param("~fix_topic", "/gps/fix"), NavSatFix,
                         self.state.fix_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param("~heading_topic", "/gps/heading"), Float64,
                         self.state.heading_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/odom"), Odometry,
                         self.state.odom_callback, queue_size=20)
        rospy.Subscriber(rospy.get_param("~battery_topic", "/m2_driver/chassis_info"),
                         ChassisStatusInfo, self.state.battery_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param("~route_status_topic", "/gps/long_range/status"),
                         String, self.state.route_status_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param("~move_base_status_topic", "/move_base/status"),
                         GoalStatusArray, self.state.action_status_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param(
            "~fod_mode_state_topic", "/fod_navigation_mode/state"),
            String, self.state.fod_mode_state_callback, queue_size=10)
        self.publish_status(False, "waiting for RabbitMQ connection")

    def publish_status(self, connected, message, last_payload=None):
        gps_available = self.state.snapshot() is not None
        last_confirmed_ms = getattr(self, 'last_confirmed_ms', 0)
        status = {
            "connected": bool(connected),
            "message": str(message),
            "exchange": self.exchange,
            "routing_key": self.routing_key,
            "publish_rate_hz": self.publish_rate,
            "published_count": self.published_count,
            "device": self.device,
            "vehicle_id": self.vehicle_id,
            "inbox_queue": self.inbox_queue,
            "receive_exchanges": self.receive_exchanges,
            "received_count": self.received_count,
            "valid_count": self.valid_count,
            "rejected_count": self.rejected_count,
            "last_received": self.last_received,
            "remote_execution": getattr(self, 'remote_execution', False),
            "task": getattr(self, 'task_status', {}),
            "gps_available": gps_available,
            "last_confirmed_ms": last_confirmed_ms,
            "position_publishing": bool(connected and gps_available and last_confirmed_ms
                                         and 0 <= time.time()*1000-last_confirmed_ms < 3000),
            "delivery_meaning": "broker confirmation; platform application receipt unconfirmed",
        }
        if last_payload is not None:
            status["last_publish_ms"] = last_payload["local_timestamp_ms"]
            status["last_error_code"] = last_payload["error_code"]
            status["last_task_status"] = last_payload["status"]
        self.status_publisher.publish(String(data=json.dumps(status, sort_keys=True)))

    def connect(self):
        credentials = pika.PlainCredentials(self.user, self.password)
        parameters = pika.ConnectionParameters(
            host=self.host,
            port=self.port,
            virtual_host=self.virtual_host,
            credentials=credentials,
            heartbeat=60,
            blocked_connection_timeout=10,
            connection_attempts=2,
            retry_delay=max(0.1, self.retry_delay),
            socket_timeout=5,
        )
        self.connection = pika.BlockingConnection(parameters)
        self.channel = self.connection.channel()
        if self.declare_exchange:
            self.channel.exchange_declare(
                exchange=self.exchange,
                exchange_type=self.exchange_type,
                durable=True,
            )
        else:
            self.channel.exchange_declare(exchange=self.exchange, passive=True)
        self.channel.confirm_delivery()
        # Dedicated queue: do not compete with collection_vehicle or consume its
        # backlog. Existing platform exchanges are fanout, despite 'direct' in
        # the mission exchange name. New exchanges use durable storage.
        self.channel.queue_declare(queue=self.inbox_queue, durable=True)
        for exchange in self.receive_exchanges:
            self.channel.exchange_declare(exchange=exchange, exchange_type='fanout', durable=True)
            self.channel.queue_bind(queue=self.inbox_queue, exchange=exchange)
        self.channel.basic_qos(prefetch_count=20)
        self.channel.basic_consume(queue=self.inbox_queue, on_message_callback=self.receive, auto_ack=False)
        rospy.loginfo(
            "FOD cloud pose reporter connected to %s:%d%s, exchange=%s, rate=%.1f Hz",
            self.host, self.port, self.virtual_host, self.exchange, self.publish_rate)
        self.publish_status(True, "connected")

    def disconnect(self):
        try:
            if self.connection is not None and self.connection.is_open:
                self.connection.close()
        except Exception:
            pass
        self.connection = None
        self.channel = None

    def publish_once(self, snapshot):
        snapshot = dict(snapshot)
        task = getattr(self, 'task_status', {})
        if task.get('active'):
            snapshot['status'] = STATUS_RUNNING
        elif (task.get('owns_task_status') and snapshot['status'] != STATUS_RUNNING
                and task.get('state') in ('COMPLETED', 'FAILED', 'CANCELED')):
            snapshot['status'] = STATUS_COMPLETE if task['state'] == 'COMPLETED' else STATUS_FAILED
        document = make_heartbeat(snapshot, self.command)
        body = json.dumps(document, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        message_id = str(uuid.uuid4())
        properties = pika.BasicProperties(
            content_type="application/json",
            content_encoding="utf-8",
            delivery_mode=2,
            timestamp=int(time.time()),
            message_id=message_id,
            app_id='autolabor-'+self.device,
        )
        self.channel.basic_publish(
            exchange=self.exchange,
            routing_key=self.routing_key,
            body=body,
            properties=properties,
            mandatory=True,
        )
        self.outbox_log.write(json.dumps(dict(system_ns=time.time_ns(), message_id=message_id,
            broker_confirmed=True, payload=document), ensure_ascii=False)+'\n')
        self.last_confirmed_ms = int(time.time()*1000)
        self.connection.process_data_events()
        self.published_count += 1
        self.publish_status(True, "publishing", snapshot)

    def receive(self, channel, method, properties, body):
        record = dict(system_ns=time.time_ns(), ros_ns=rospy.Time.now().to_nsec(),
                      exchange=method.exchange, routing_key=method.routing_key,
                      message_id=properties.message_id, redelivered=method.redelivered,
                      raw_base64=base64.b64encode(body).decode('ascii'))
        self.received_count += 1
        try:
            record.update(decode_incoming(body, self.vehicle_id), valid=True)
            self.valid_count += 1
        except (ValueError, TypeError, KeyError, UnicodeError) as exc:
            record.update(valid=False, error=str(exc), executed=False)
            self.rejected_count += 1
        if record.get('valid') and getattr(self, 'task_runner', None) is not None:
            # Preserve the full request before the worker can issue a goal.
            with (self.log_directory/'received_raw.jsonl').open('a') as raw_stream:
                raw_stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
                raw_stream.flush()
                os.fsync(raw_stream.fileno())
            try:
                record['task_handling'] = self.task_runner.submit(record)
            except ValueError as error:
                record['task_handling'] = str(error)
        # Acknowledge only this private copy after preserving the original body.
        text = json.dumps(record, ensure_ascii=False, allow_nan=False)
        self.inbox_log.write(text+'\n')
        self.inbox_log.flush()
        self.inbox_pub.publish(String(data=text))
        channel.basic_ack(delivery_tag=method.delivery_tag)
        self.last_received = {k:record.get(k) for k in ('system_ns','exchange','kind','valid','message_id','mission_id','addressed_to_this_vehicle','task_handling')}
        self.publish_status(True, record.get('task_handling', 'remote packet saved'))

    def stop_task_worker(self):
        if self.task_runner is not None:
            self.task_runner.cancel('平台通信节点结束')
            self.task_runner.shutdown.set()
            if self.task_thread is not threading.current_thread():
                self.task_thread.join(timeout=12)

    def run(self):
        next_publish = 0.0
        while not rospy.is_shutdown():
            if self.connection is None or not self.connection.is_open:
                try:
                    self.connect()
                except Exception as error:
                    self.disconnect()
                    message = "RabbitMQ connection failed: {}".format(error)
                    rospy.logwarn_throttle(10.0, message)
                    self.publish_status(False, message)
                    rospy.sleep(max(0.1, self.retry_delay))
                    continue

            remaining = next_publish-time.monotonic()
            if remaining > 0:
                try:
                    self.connection.process_data_events(time_limit=min(.25, remaining))
                except Exception as error:
                    self.publish_status(False, 'RabbitMQ receive failed: '+str(error))
                    self.disconnect()
                continue

            snapshot = self.state.snapshot()
            if snapshot is None:
                rospy.logwarn_throttle(10.0, "cloud pose reporter is waiting for /gps/fix")
                self.publish_status(True, "connected; waiting for a finite GPS position")
            else:
                try:
                    self.publish_once(snapshot)
                except Exception as error:
                    message = "RabbitMQ publish failed: {}".format(error)
                    rospy.logwarn(message)
                    self.publish_status(False, message, snapshot)
                    self.disconnect()
            # Receiving a busy burst must not accelerate the heartbeat rate.
            next_publish = time.monotonic()+1.0/self.publish_rate
        self.disconnect()
        self.inbox_log.close()
        self.outbox_log.close()


def main():
    rospy.init_node("fod_platform_link")
    try:
        CloudPoseReporter().run()
    except (TypeError, ValueError) as error:
        rospy.logfatal("Invalid FOD cloud pose reporter configuration: %s", error)
        raise


if __name__ == "__main__":
    main()
