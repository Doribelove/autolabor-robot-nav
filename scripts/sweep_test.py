#!/usr/bin/env python3
"""Manual TCP verification for SweepDeviceControl; standard library only."""

import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "SweepDeviceControl"))
from SweepDeviceControl import SweepDeviceTCPClient


class DeviceError(RuntimeError):
    pass


class SweepTester(SweepDeviceTCPClient):
    """Reuse the supplied protocol; send each toggle or hydraulic pulse once."""

    def __init__(self, ip, port=50003, timeout=2.0, settle=4.0):
        super().__init__(ip=ip, port=port, timeout=timeout)
        self.settle = settle
        self.last_state = -1
        self.previous_state = None
        self.hydraulic_acknowledged = False
        self.hydraulic_sent = False
        self.hydraulic_deadline = 0.0

    def _flush_pending(self, flush_timeout=0.5):
        # A fresh TCP session has no stale command replies. exchange() already
        # tolerates the unsolicited version banner and fragmented frames.
        # Do not wait half a second for silence before every motor command.
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def _parse_status(self, response):
        # TCP may combine multiple frames; use the last complete state frame.
        on = response.rfind(self.resp_on)
        off = response.rfind(self.resp_off)
        if max(on, off) < 0:
            return -1
        return int(on > off)

    def exchange(self, command, expect_status, ack_only=False):
        if not self.sock and not self.connect():
            raise DeviceError("无法连接 {}:{}".format(self.device_ip, self.device_port))
        response = bytearray()
        deadline = time.monotonic() + self.timeout
        try:
            self.sock.sendall(bytes.fromhex(command))
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise socket.timeout("等待完整应答超时")
                self.sock.settimeout(remaining)
                chunk = self.sock.recv(1024)
                if not chunk:
                    raise ConnectionError("设备断开 TCP 连接")
                response.extend(chunk)
                value = response.hex().upper()
                if not ack_only and self._parse_status(value) != -1:
                    return value
                if not expect_status and self.ack_success in value:
                    return value
        except OSError as error:
            self.close()
            detail = response.hex().upper() or "无"
            raise DeviceError("{}；收到的应答：{}".format(error, detail)) from error
        finally:
            if self.sock:
                self.sock.settimeout(self.timeout)

    def status(self):
        # Retrying a query is harmless; a lost toggle ACK must never be retried.
        for attempt in range(2):
            try:
                self.last_state = self._parse_status(self.exchange(self.cmd_query_status, True))
                return self.last_state
            except DeviceError:
                if attempt == 1:
                    self.last_state = -1
                    raise
                time.sleep(0.1)

    def set_state(self, wanted):
        before = self.status()
        self.previous_state = before
        if self._request_state(wanted, before):
            self._confirm_state(wanted)

    def _request_state(self, wanted, before):
        label = "开启" if wanted else "关闭"
        if before == wanted:
            print("[确认] 装置已经{}，无需发送切换指令。".format(label))
            return False

        print("[发送] {}清扫装置：只发送一次切换指令。".format("启动" if wanted else "停止"))
        try:
            reply = self.exchange(self.cmd_switch, False)
            print("[应答] {}".format(reply))
        except DeviceError as error:
            print("[提示] 切换应答未确认：{}；继续查询实际状态，不重发切换。".format(error))
        return True

    def _confirm_state(self, wanted):
        label = "开启" if wanted else "关闭"
        deadline = time.monotonic() + self.settle
        detail = "尚未收到目标状态"
        while True:
            try:
                actual = self.status()
                if actual == wanted:
                    print("[确认] 设备状态回读：已{}。".format(label))
                    return
                detail = "设备仍报告已{}".format("开启" if actual else "关闭")
            except DeviceError as error:
                detail = str(error)
            if time.monotonic() >= deadline:
                raise DeviceError("未确认装置已{}：{}。请查询状态；本次没有重复切换。".format(label, detail))
            time.sleep(max(0.0, min(0.2, deadline - time.monotonic())))

    def _start_hydraulic(self, action, duration_s):
        self.hydraulic_acknowledged = False
        self.hydraulic_sent = False
        self.hydraulic_deadline = 0.0
        command = self.cmd_hydraulic.get((action, duration_s))
        if command is None:
            raise DeviceError("液压杆只支持 extend / retract，动作时长只能为 2 或 3 秒。")
        label = "升出" if action == "extend" else "回收"
        if not self.sock and not self.connect():
            raise DeviceError("无法连接 {}:{}".format(self.device_ip, self.device_port))
        print("[发送] 液压杆{} {} 秒；只发送一次指令。".format(label, duration_s))
        self.hydraulic_deadline = time.monotonic() + duration_s
        self.hydraulic_sent = True
        try:
            # A sweep-state frame is not an acknowledgement of rod motion.
            self.exchange(command, False, ack_only=True)
            self.hydraulic_acknowledged = True
            self.hydraulic_deadline = time.monotonic() + duration_s
            print("[确认] 控制器已接收液压杆指令，等待动作计时结束。")
        except DeviceError as error:
            raise DeviceError("液压杆{}指令未确认：{}。未自动重发，请观察实际位置。"
                              .format(label, error)) from error

    def _wait_hydraulic(self):
        # Keep the shared device lock through the pulse, including lost ACKs.
        remaining = self.hydraulic_deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)

    def move_hydraulic(self, action, duration_s=2):
        try:
            self._start_hydraulic(action, duration_s)
        finally:
            self._wait_hydraulic()
        label = "升出" if action == "extend" else "回收"
        return "液压杆{} {} 秒指令已接收，动作计时结束；设备不回传位置，请观察实物。".format(label, duration_s)

    def set_recovery(self, enabled):
        """Overlap one 3-second rod pulse and one confirmed motor transition."""
        before = self.status()
        self.previous_state = before
        wanted = int(enabled)
        errors = []
        try:
            if enabled:
                # Confirm reception, then start the motor during extension.
                # Do not start it if the extension command was not acknowledged.
                self._start_hydraulic("extend", 3)
                self._request_state(wanted, before)
            else:
                # Send OFF first, then retract without waiting for motor
                # state polling. Both commands use this same TCP connection.
                self._request_state(wanted, before)
                try:
                    self._start_hydraulic("retract", 3)
                except DeviceError as error:
                    errors.append(str(error))
            try:
                self._confirm_state(wanted)
            except DeviceError as error:
                errors.append(str(error))
        finally:
            self._wait_hydraulic()
        if errors:
            raise DeviceError("；".join(errors))
        return "清扫已{}；液压杆{} 3 秒指令已接收，动作计时结束（设备不回传杆位置）。".format(
            "开启" if enabled else "关闭", "升出" if enabled else "回收")


def show_status(device):
    print("[状态] 清扫装置已{}。".format("开启" if device.status() else "关闭"))


def menu(device, duration_s=2):
    show_status(device)
    print("\n输入 1 启动；0 停止；s 查询；e 液压杆升出；r 液压杆回收；q 停止清扫并退出。")
    print("液压杆单次动作 {} 秒；均需回车。".format(duration_s))
    print("Ctrl+C / Ctrl+D 也会尝试停止并退出。")
    while True:
        try:
            choice = input("清扫测试> ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print()
            choice = "q"
        if choice in ("q", "quit", "exit"):
            device.set_state(0)
            return
        try:
            if choice in ("1", "on", "start"):
                device.set_state(1)
            elif choice in ("0", "off", "stop"):
                device.set_state(0)
            elif choice in ("s", "status"):
                show_status(device)
            elif choice in ("e", "extend", "r", "retract"):
                action = "extend" if choice in ("e", "extend") else "retract"
                print(device.move_hydraulic(action, duration_s))
            elif choice:
                print("请输入 1、0、s、e、r 或 q。")
        except DeviceError as error:
            print("[失败] {}".format(error))
        except KeyboardInterrupt:
            print("\n正在查询并停止清扫装置……")
            device.set_state(0)
            return


def positive_seconds(value):
    number = float(value)
    if not 0 < number < float("inf"):
        raise argparse.ArgumentTypeError("必须是大于 0 的有限秒数")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description="清扫装置控制：状态查询、启停、液压杆升出/回收。")
    parser.add_argument("action", nargs="?", choices=("menu", "status", "on", "off", "extend", "retract",
                                                    "recovery-start", "recovery-stop"), default="menu",
                        help="默认 menu；recovery-start/stop 联动清扫和液压杆，固定 3 秒")
    parser.add_argument("--duration", type=int, choices=(2, 3), default=2,
                        help="液压杆单次动作秒数，仅支持 2 或 3（默认 2）")
    parser.add_argument("--ip", default="192.168.0.197", help="设备 IP（说明书默认：192.168.0.197）")
    parser.add_argument("--port", type=int, default=50003, help="设备 TCP 端口（默认：50003）")
    parser.add_argument("--timeout", type=positive_seconds, default=2.0, help="单次通信超时秒数（默认：2）")
    parser.add_argument("--settle", type=positive_seconds, default=4.0, help="切换后等待目标状态秒数（默认：4）")
    parser.add_argument("--json", action="store_true", help="输出供 Qt 使用的 JSON 结果，过程日志写入 stderr")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("端口必须在 1..65535 之间")
    if args.json and args.action == "menu":
        parser.error("--json 不能用于交互菜单")
    # The device accepts one TCP client. Serialize whole operations from the
    # Qt panel and automatic recovery, including toggle + state confirmation.
    identity = hashlib.sha256("{}:{}".format(args.ip, args.port).encode()).hexdigest()[:16]
    lock_path = Path(tempfile.gettempdir()) / "autolabor-sweep-{}-{}.lock".format(os.getuid(), identity)
    with lock_path.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        return execute_action(args)


def execute_action(args):
    device = SweepTester(args.ip, args.port, args.timeout, args.settle)
    hydraulic = args.action in ("extend", "retract")
    recovery = args.action in ("recovery-start", "recovery-stop")
    if args.json:
        # Keep human-readable diagnostics separate from the machine result.
        result = {"action": args.action, "success": False, "state": -1, "message": ""}
        with contextlib.redirect_stdout(sys.stderr):
            try:
                if args.action == "status":
                    device.status()
                elif hydraulic:
                    result["message"] = device.move_hydraulic(args.action, args.duration)
                elif recovery:
                    result["message"] = device.set_recovery(args.action == "recovery-start")
                else:
                    device.set_state(int(args.action == "on"))
                result["success"] = True
                if not hydraulic and not recovery:
                    result["message"] = "设备回读：清扫装置已{}。".format("开启" if device.last_state else "关闭")
            except DeviceError as error:
                result["message"] = str(error)
            except KeyboardInterrupt:
                device.last_state = -1
                result["message"] = "通信被中断，请重新查询设备状态。"
            finally:
                result["state"] = device.last_state
                result["previous_state"] = device.previous_state
                if hydraulic or recovery:
                    result.update(duration_s=3 if recovery else args.duration,
                                  hydraulic_sent=device.hydraulic_sent,
                                  acknowledged=device.hydraulic_acknowledged,
                                  position_confirmed=False)
                device.close()
        print(json.dumps(result, ensure_ascii=False), flush=True)
        return 0 if result["success"] else 2
    print("清扫控制器 {}:{} / {}".format(args.ip, args.port, args.action))
    try:
        if args.action == "menu":
            menu(device, args.duration)
        elif args.action == "status":
            show_status(device)
        elif hydraulic:
            print(device.move_hydraulic(args.action, args.duration))
        elif recovery:
            print(device.set_recovery(args.action == "recovery-start"))
        else:
            device.set_state(int(args.action == "on"))
        return 0
    except DeviceError as error:
        print("[失败] {}".format(error), file=sys.stderr)
        print("检查设备电源、IP/网段、网线，以及是否有其他程序占用其单 TCP 连接。", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n操作已中断；请运行 off 命令确认关闭。", file=sys.stderr)
        return 130
    finally:
        device.close()


if __name__ == "__main__":
    sys.exit(main())
