#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import pika
import sys
import threading
import time

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus

# ====================== RabbitMQ 配置 ======================
RABBITMQ_HOST = "39.98.47.163"
RABBITMQ_PORT = 5672
RABBITMQ_USER = "caacsriUser"
RABBITMQ_PWD = "caacsriUser"
VIRTUAL_HOST = "/"

# 队列名称，必须和发送端确认一致
QUEUE_NAME = "collection_vehicle"

# ====================== ROS 导航目标配置 ======================
# rabbitmq_gps_goal_bridge.py 接收 RabbitMQ JSON，保存最新 GPS 目标；
# 操作员在当前终端输入 1 后，才把保存的目标发布出去。
# gps_module/gps_goal_node.py 会订阅 /gps/goal_fix，把经纬度转换成局部坐标，
# 再发布 /move_base_simple/goal 给 move_base。
GPS_GOAL_FIX_TOPIC = "/gps/goal_fix"
GPS_FRAME_ID = "gps"

# ====================== Exchange 配置 ======================
# 如果发送端是直接发到队列：
# exchange = ""
# routing_key = "collection_vehicle"
#
# 那这里保持 EXCHANGE_NAME = "" 即可。
#
# 如果发送端是发到某个 exchange，请填写：
# EXCHANGE_NAME = "对方的exchange名称"
# EXCHANGE_TYPE = "direct" / "topic" / "fanout"
# ROUTING_KEY = "对方发送时用的routing_key"

EXCHANGE_NAME = ""          # 不知道就先留空
EXCHANGE_TYPE = "direct"    # direct / topic / fanout
ROUTING_KEY = QUEUE_NAME
# ===========================================================


def decode_message(body: bytes) -> str:
    """
    尝试解码消息，避免因为编码问题导致程序看起来没反应。
    """
    for encoding in ("utf-8", "gbk", "gb2312"):
        try:
            return body.decode(encoding)
        except UnicodeDecodeError:
            pass

    return repr(body)


def get_env(name, default):
    value = os.environ.get(name)
    return value if value not in (None, "") else default


def parse_allowed_types(value):
    """
    将 "0,2" 这类配置解析成 {0, 2}。为空表示不过滤 TYPE。
    """
    if value in (None, ""):
        return None

    result = set()
    for item in str(value).split(","):
        item = item.strip()
        if not item:
            continue
        result.add(int(item, 0))
    return result


def as_float(value, field_name):
    if value in (None, ""):
        raise ValueError(f"{field_name} is empty")
    result = float(value)
    if math.isnan(result):
        raise ValueError(f"{field_name} is NaN")
    return result


def load_json_message(message):
    """
    RabbitMQ 正常应发送纯 JSON。这里额外兼容前后带日志文本的情况：
    如果整体解析失败，就截取第一个 { 到最后一个 } 再解析。
    """
    text = message.strip()
    if text.startswith("\ufeff"):
        text = text[1:]

    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        start = text.find("{")
        end = text.rfind("}")
        if start < 0 or end <= start:
            raise
        data = json.loads(text[start:end + 1])

    if isinstance(data, str):
        data = json.loads(data)
    if not isinstance(data, dict):
        raise ValueError("message JSON must be an object")
    return data


def extract_targets(data, allowed_types=None):
    targets = data.get("TARGETS")
    if not isinstance(targets, list):
        raise ValueError("TARGETS must be a list")

    extracted = []
    for index, target in enumerate(targets):
        if not isinstance(target, dict):
            print(f"[!] 跳过 TARGETS[{index}]：不是对象")
            continue

        raw_type = target.get("TYPE")
        target_type = int(raw_type, 0) if isinstance(raw_type, str) else raw_type
        if allowed_types is not None and target_type not in allowed_types:
            print(f"[!] 跳过 TARGETS[{index}]：TYPE={target_type} 不在允许列表")
            continue

        lat = as_float(target.get("LAT"), f"TARGETS[{index}].LAT")
        lon = as_float(target.get("LON"), f"TARGETS[{index}].LON")
        if not -90.0 <= lat <= 90.0:
            raise ValueError(f"TARGETS[{index}].LAT out of range: {lat}")
        if not -180.0 <= lon <= 180.0:
            raise ValueError(f"TARGETS[{index}].LON out of range: {lon}")

        extracted.append({
            "lat": lat,
            "lon": lon,
            "time": target.get("TIME", ""),
            "type": target_type,
            "data": target.get("DATA", ""),
            "url": target.get("URL", ""),
        })

    return extracted


class GpsGoalBridge:
    def __init__(self):
        self.goal_fix_topic = rospy.get_param(
            "~goal_fix_topic",
            get_env("GPS_GOAL_FIX_TOPIC", GPS_GOAL_FIX_TOPIC),
        )
        self.frame_id = rospy.get_param(
            "~gps_frame_id",
            get_env("GPS_FRAME_ID", GPS_FRAME_ID),
        )
        self.allowed_types = parse_allowed_types(rospy.get_param(
            "~allowed_types",
            get_env("ALLOWED_TARGET_TYPES", ""),
        ))
        self.wait_for_subscriber = bool(rospy.get_param(
            "~wait_for_subscriber",
            True,
        ))
        self.latest_target = None
        self.latest_target_lock = threading.Lock()

        self.pub = rospy.Publisher(
            self.goal_fix_topic,
            NavSatFix,
            queue_size=10,
            latch=False,
        )

        print("[*] ROS GPS 目标接收桥接已启动")
        print(f"    goal_fix_topic: {self.goal_fix_topic}")
        print(f"    frame_id: {self.frame_id}")
        if self.allowed_types is None:
            print("    allowed_types: all")
        else:
            print(f"    allowed_types: {sorted(self.allowed_types)}")

    def save_latest_target(self, target):
        saved_target = dict(target)
        saved_target["received_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        with self.latest_target_lock:
            self.latest_target = saved_target

        print("\n[√] 已保存最新 GPS 点（尚未发送给无人车）")
        print(f"    LAT: {saved_target['lat']:.12f}")
        print(f"    LON: {saved_target['lon']:.12f}")
        print(f"    TYPE: {saved_target['type']}")
        print(f"    TIME: {saved_target['time']}")
        print(f"    DATA: {saved_target['data']}")
        if saved_target["url"]:
            print(f"    URL: {saved_target['url']}")
        print(f"    接收时间: {saved_target['received_at']}")
        print("    输入 1 发送该点，输入 2 清空该点")

    def get_latest_target(self):
        with self.latest_target_lock:
            if self.latest_target is None:
                return None
            return dict(self.latest_target)

    def clear_latest_target(self):
        with self.latest_target_lock:
            had_target = self.latest_target is not None
            self.latest_target = None

        if had_target:
            print("\n[√] 已清空保存的最新 GPS 点")
        else:
            print("\n[!] 当前没有已保存的 GPS 点")

    def wait_until_connected(self):
        if not self.wait_for_subscriber:
            return

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and self.pub.get_num_connections() == 0:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for subscriber on %s",
                self.goal_fix_topic,
            )
            rate.sleep()

    def publish_target(self, target):
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = target["lat"]
        msg.longitude = target["lon"]
        msg.altitude = 0.0

        self.pub.publish(msg)
        print(
            "[√] 已将保存的 GPS 目标发布到 %s: lat=%.12f lon=%.12f type=%s time=%s data=%s"
            % (
                self.goal_fix_topic,
                target["lat"],
                target["lon"],
                target["type"],
                target["time"],
                target["data"],
            )
        )

    def publish_latest_target(self):
        target = self.get_latest_target()
        if target is None:
            print("\n[!] 当前没有已保存的 GPS 点，无法发送")
            return False

        self.wait_until_connected()
        if rospy.is_shutdown():
            return False
        self.publish_target(target)
        return True

    def handle_json_message(self, message):
        data = load_json_message(message)
        cmd = data.get("CMD", "")
        device = data.get("DEVICE", "")
        targets = extract_targets(data, self.allowed_types)

        print(f"CMD: {cmd}, DEVICE: {device}, TARGETS 数量: {len(targets)}")
        if not targets:
            print("[!] 未提取到有效 GPS 点，消息将 ack，但不会更新已保存的点")
            return 0

        if len(targets) > 1:
            print(f"[!] 本条消息包含 {len(targets)} 个目标，只保存 TARGETS 中最后一个点")
        self.save_latest_target(targets[-1])

        return len(targets)


def start_operator_console(gps_bridge):
    def console_loop():
        print("\n========== GPS 目标操作 ==========")
        print("输入 1：发送当前保存的最新 GPS 点")
        print("输入 2：清空当前保存的最新 GPS 点")
        print("按 Ctrl+C：退出桥接程序")

        while not rospy.is_shutdown():
            try:
                command = input("GPS操作> ").strip()
            except EOFError:
                print("\n[!] 当前终端没有可用的标准输入，交互操作已停止")
                return

            if command == "1":
                gps_bridge.publish_latest_target()
            elif command == "2":
                gps_bridge.clear_latest_target()
            elif command:
                print("[!] 无效输入：请输入 1 或 2")

    console_thread = threading.Thread(
        target=console_loop,
        name="gps_goal_operator_console",
        daemon=True,
    )
    console_thread.start()
    return console_thread


def connect_rabbitmq():
    credentials = pika.PlainCredentials(
        username=RABBITMQ_USER,
        password=RABBITMQ_PWD
    )

    params = pika.ConnectionParameters(
        host=RABBITMQ_HOST,
        port=RABBITMQ_PORT,
        virtual_host=VIRTUAL_HOST,
        credentials=credentials,
        heartbeat=600,
        blocked_connection_timeout=300,
        connection_attempts=3,
        retry_delay=5,
        socket_timeout=10,
    )

    print("[*] 正在连接 RabbitMQ...")
    print(f"    host: {RABBITMQ_HOST}")
    print(f"    port: {RABBITMQ_PORT}")
    print(f"    vhost: {VIRTUAL_HOST}")
    print(f"    queue: {QUEUE_NAME}")

    connection = pika.BlockingConnection(params)
    channel = connection.channel()

    print("[√] RabbitMQ 连接成功")

    return connection, channel


def setup_queue(channel):
    """
    声明队列并按需绑定 exchange。
    """
    print("[*] 正在声明队列...")

    result = channel.queue_declare(
        queue=QUEUE_NAME,
        durable=True
    )

    print("[√] 队列声明成功")
    print(f"    Ready 消息数: {result.method.message_count}")
    print(f"    当前消费者数量: {result.method.consumer_count}")

    if EXCHANGE_NAME:
        print("[*] 正在声明并绑定 exchange...")
        print(f"    exchange: {EXCHANGE_NAME}")
        print(f"    exchange_type: {EXCHANGE_TYPE}")
        print(f"    routing_key: {ROUTING_KEY}")

        channel.exchange_declare(
            exchange=EXCHANGE_NAME,
            exchange_type=EXCHANGE_TYPE,
            durable=True
        )

        if EXCHANGE_TYPE == "fanout":
            channel.queue_bind(
                queue=QUEUE_NAME,
                exchange=EXCHANGE_NAME
            )
        else:
            channel.queue_bind(
                queue=QUEUE_NAME,
                exchange=EXCHANGE_NAME,
                routing_key=ROUTING_KEY
            )

        print("[√] exchange 绑定成功")
    else:
        print("[*] 当前未配置 exchange，默认监听队列本身")


def start_consume(channel, gps_bridge):
    """
    启动消费者。
    """

    # 一次只给当前消费者分发 1 条未确认消息，方便调试和控制处理压力
    channel.basic_qos(prefetch_count=1)

    def message_callback(ch, method, properties, body):
        print("\n========== 收到 RabbitMQ 消息 ==========")
        print(f"delivery_tag: {method.delivery_tag}")
        print(f"exchange: {method.exchange}")
        print(f"routing_key: {method.routing_key}")
        print(f"body bytes length: {len(body)}")

        try:
            message = decode_message(body)
            print(f"消息内容: {message}")
            extracted_count = gps_bridge.handle_json_message(message)

            ch.basic_ack(delivery_tag=method.delivery_tag)
            if extracted_count > 0:
                print(f"[√] 消息处理完成，提取 GPS 点 {extracted_count} 个，最新点已保存，未自动发送，已 ack")
            else:
                print("[√] 消息处理完成，没有更新 GPS 点，已 ack")

        except Exception as e:
            print(f"[×] 消息处理失败: {e}")

            # 调试阶段建议 requeue=False，避免同一条坏消息无限重复消费
            # 如果你确实希望失败后重新入队，可以改成 requeue=True
            ch.basic_nack(
                delivery_tag=method.delivery_tag,
                requeue=False
            )
            print("[!] 消息已 nack，未重新入队")

    channel.basic_consume(
        queue=QUEUE_NAME,
        on_message_callback=message_callback,
        auto_ack=False
    )

    print("\n[*] 已开始等待消息...")
    print("[*] 如果一直没有输出，通常说明发送端没有把消息投递到这个队列")
    print("[*] 请重点确认：vhost、queue、exchange、routing_key 是否一致")
    print("[*] 按 Ctrl+C 退出\n")

    start_operator_console(gps_bridge)
    channel.start_consuming()


def main():
    connection = None

    try:
        rospy.init_node("rabbitmq_gps_goal_bridge", anonymous=False)
        gps_bridge = GpsGoalBridge()

        connection, channel = connect_rabbitmq()
        setup_queue(channel)
        start_consume(channel, gps_bridge)

    except KeyboardInterrupt:
        print("\n[!] 手动中断，正在退出...")

    except pika.exceptions.ProbableAuthenticationError:
        print("[×] RabbitMQ 用户名或密码错误")
        sys.exit(1)

    except pika.exceptions.ProbableAccessDeniedError:
        sys.exit(1)

    except pika.exceptions.AMQPConnectionError as e:
        print(f"[×] RabbitMQ 连接失败: {e}")
        sys.exit(1)

    except pika.exceptions.ChannelClosedByBroker as e:
        print(f"[×] RabbitMQ 通道被服务端关闭: {e}")
        print("[!] 常见原因：队列已存在，但 durable 等属性和你声明的不一致")
        sys.exit(1)

    except Exception as e:
        print(f"[×] 程序异常: {e}")
        sys.exit(1)

    finally:
        if connection and connection.is_open:
            connection.close()
            print("[√] RabbitMQ 连接已关闭")


if __name__ == "__main__":
    main()
