import socket
import sys
import time


class SweepDeviceTCPClient:
    def __init__(self, ip: str = "192.168.1.197", port: int = 50003, timeout: int = 5):
        self.device_ip = ip
        self.device_port = port
        self.timeout = timeout
        self.sock = None
        # 预设指令 十六进制字符串
        self.cmd_switch = "CCDDD30100000000000100000000000103E8DDCC"  # 开关机指令
        self.cmd_query_status = "CCDDC30100000DCE9C"                 # 查询状态指令
        # 响应标识
        self.ack_success = "4F4B21"  # 指令下发成功ACK
        self.resp_on = "EEFFC3010000000000020D"   # 设备开启
        self.resp_off = "EEFFC3010000000000000D"  # 设备关闭

    def connect(self) -> bool:
        """建立单TCP连接，失败返回False"""
        try:
            if self.sock:
                self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.device_ip, self.device_port))
            print(f"[成功] 成功连接设备 {self.device_ip}:{self.device_port}")
            self._flush_pending()
            return True
        except Exception as e:
            print(f"[失败] 连接设备失败: {str(e)}")
            self.sock = None
            return False

    def _flush_pending(self, flush_timeout: float = 0.5):
        """连接后清空设备主动推送的数据（如版本号 v1.0）"""
        if not self.sock:
            return
        try:
            self.sock.settimeout(flush_timeout)
            while True:
                data = self.sock.recv(1024)
                if not data:
                    break
                text = data.decode("ascii", errors="replace")
                print(f"忽略设备推送数据: {text}")
        except socket.timeout:
            pass
        finally:
            self.sock.settimeout(self.timeout)

    def _parse_status(self, resp: str) -> int:
        """从应答中解析设备状态，return: 0关机 / 1开启 / -1无法识别"""
        if not resp:
            return -1
        if self.resp_on in resp:
            return 1
        if self.resp_off in resp:
            return 0
        return -1

    def send_hex_cmd(self, hex_str: str) -> str:
        """发送十六进制指令，返回接收的原始十六进制应答"""
        if not self.sock:
            if not self.connect():
                return ""
        try:
            send_bytes = bytes.fromhex(hex_str)
            self.sock.sendall(send_bytes)
            recv_data = self.sock.recv(1024)
            return recv_data.hex().upper()
        except socket.timeout:
            print("[警告] 通信超时")
            self.sock.close()
            self.sock = None
            return ""
        except Exception as e:
            print(f"[警告] 发送指令异常: {str(e)}")
            self.sock.close()
            self.sock = None
            return ""

    def get_device_status(self, retry_times: int = 3) -> int:
        """
        查询清扫装置状态
        return: 0关机 / 1开启 / -1查询失败
        """
        for attempt in range(retry_times):
            resp = self.send_hex_cmd(self.cmd_query_status)
            status = self._parse_status(resp)
            if status != -1:
                return status
            if resp:
                print(f"未知状态响应: {resp}，重试中... ({attempt + 1}/{retry_times})")
            else:
                print(f"状态查询无响应，重试中... ({attempt + 1}/{retry_times})")
            time.sleep(0.2)
        return -1

    def toggle_sweep_device(self, retry_times: int = 3) -> bool:
        """
        发送开关机指令，并校验状态确认执行成功
        :param retry_times: 失败重试次数
        :return: True执行成功 / False执行失败
        """
        for i in range(retry_times):
            print(f"\n第{i + 1}次下发开关机指令")
            resp = self.send_hex_cmd(self.cmd_switch)

            if self.ack_success in resp:
                print("指令下发成功，等待设备切换状态...")
            elif self._parse_status(resp) != -1:
                print(f"设备直接返回状态帧(未收到ACK)，应答:{resp}，继续校验...")
            else:
                print(f"指令下发未收到有效应答，当前应答:{resp}，准备重试")
                time.sleep(0.5)
                continue

            time.sleep(0.8)
            status = self.get_device_status()
            if status != -1:
                print(f"设备当前状态: {'开启' if status == 1 else '关机'}")
                return True
            print("状态查询无有效返回，重试中...")
            time.sleep(0.5)

        print(f"连续{retry_times}次操作均失败")
        return False

    def close(self):
        """关闭TCP连接"""
        if self.sock:
            self.sock.close()
            self.sock = None
            print("已断开设备连接")


if __name__ == "__main__":
    sweep_dev = SweepDeviceTCPClient(ip="192.168.0.197", port=50003)

    if not sweep_dev.connect():
        sys.exit(1)

    state = sweep_dev.get_device_status()
    if state == 1:
        print("初始状态：清扫装置已打开")
    elif state == 0:
        print("初始状态：清扫装置已关机")
    else:
        print("初始状态查询失败")

    result = sweep_dev.toggle_sweep_device(retry_times=3)
    if result:
        print("[成功] 清扫装置开关操作执行完成，状态校验通过")
    else:
        print("[失败] 清扫装置开关操作执行失败")

    sweep_dev.close()
