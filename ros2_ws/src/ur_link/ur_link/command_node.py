# -*- coding: utf-8 -*-
"""UR 指令节点：
- Dashboard(29999)：上电/松刹车/加载程序/运行/停止/安全弹窗 等(Trigger 服务)
- Secondary(30002)：订阅 ~/urscript 话题(std_msgs/String)即发送 URScript
"""
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# 预设服务名 -> Dashboard 命令
DASHBOARD_PRESETS = {
    "power_on": "power on",
    "power_off": "power off",
    "brake_release": "brake release",
    "close_safety_popup": "close safety popup",
    "stop": "stop",
    "pause": "pause",
    "play": "play",
    "robotmode": "robotmode",
    "is_program_running": "is program running",
    "get_loaded_program": "get loaded program",
}


class DashboardClient:
    """Dashboard 29999：纯文本行命令，一行一答(OK / ERROR ...)。"""

    def __init__(self, host, port=29999, timeout=2.0):
        self.host, self.port, self.timeout = host, port, timeout
        self.sock = None
        self._lock = threading.Lock()

    def _connect(self):
        if self.sock is None:
            self.sock = socket.create_connection((self.host, self.port),
                                                 timeout=self.timeout)
            self.sock.settimeout(self.timeout)
            # Dashboard 服务器一连接先发一条欢迎语(如 "Connected: ...")，
            # 把它读掉，避免被当成命令应答。
            try:
                self.sock.settimeout(1.0)
                greeting = self.sock.recv(4096)
                self.sock.settimeout(self.timeout)
            except socket.timeout:
                pass
        return self.sock

    def cmd(self, text: str) -> str:
        with self._lock:
            for _ in range(2):
                try:
                    s = self._connect()
                    s.sendall((text + "\n").encode())
                    # Dashboard 每条命令回复一行(可能多条，读到非空行即可)
                    lines = []
                    deadline = time.time() + self.timeout
                    while time.time() < deadline and len(lines) < 3:
                        line = s.recv(1024).decode(errors="replace")
                        if line:
                            lines += [ln for ln in line.splitlines() if ln]
                            if lines:
                                break
                    return lines[0] if lines else "(no answer)"
                except OSError:
                    self.close()
            return "(command failed / connection lost)"

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


class UrCommandNode(Node):
    def __init__(self):
        super().__init__("ur_command_node")
        self.robot_ip = self.declare_parameter("robot_ip", "192.168.1.3").value
        self.dash_port = self.declare_parameter("dashboard_port", 29999).value
        self.script_port = self.declare_parameter("urscript_port", 30002).value

        self._dash = DashboardClient(self.robot_ip, self.dash_port)
        self._script_sock = None

        for name, cmd in DASHBOARD_PRESETS.items():
            self.create_service(Trigger, f"ur_link/dashboard/{name}",
                                self._make_handler(cmd, name))

        self.create_subscription(String, "ur_link/urscript", self._on_urscript, 10)
        self.get_logger().info(
            f"ur_command_node -> {self.robot_ip} (dashboard {self.dash_port}, "
            f"urscript {self.script_port})\n"
            "用法: ros2 topic pub /ur_link/urscript std_msgs/String "
            "data:'movej([0,-1.5708,0,-1.5708,0,0], a=1.4, v=1.05)' -1")

    def _make_handler(self, dash_cmd, name):
        def handler(req, res):
            ans = self._dash.cmd(dash_cmd)
            res.success = ans.startswith("OK")
            res.message = f"[{dash_cmd}] -> {ans}"
            return res
        return handler

    def _get_script_sock(self):
        if self._script_sock is None:
            self._script_sock = socket.create_connection(
                (self.robot_ip, self.script_port), timeout=3.0)
            self._script_sock.settimeout(0.3)
        return self._script_sock

    def _on_urscript(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        # UR 客户端接口要求：脚本用 def/sec 包裹、行首缩进、end 收尾，
        # 否则 URControl 会静默丢弃。运动指令只能用主程序 def（sec 线程禁运动）。
        if not text.startswith(("sec ", "def ")):
            indented = "\n".join("  " + ln for ln in text.splitlines())
            text = f"def ros_cmd():\n{indented}\nend"
        self.get_logger().info(f"发送 URScript -> {self.script_port}:\n{text}")
        for _ in range(2):
            try:
                s = self._get_script_sock()
                s.sendall((text + "\n").encode())
                # 长连接：保持打开并读应答(版本消息/状态流)，别发完就关，
                # 否则脚本可能被丢弃(官方文档明确警告)。
                try:
                    s.recv(4096)
                except socket.timeout:
                    pass
                return
            except OSError as e:
                self.get_logger().error(f"30002 发送失败: {e}")
                self._script_sock = None


def main(args=None):
    rclpy.init(args=args)
    node = UrCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._dash.close()
        if node._script_sock:
            try:
                node._script_sock.close()
            except OSError:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
