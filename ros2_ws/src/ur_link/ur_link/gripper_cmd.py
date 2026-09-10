# -*- coding: utf-8 -*-
"""
ROS ↔ UR 通用命令桥（"像 ROS 控制一样"的按需控制）。

示教器跑通用执行器(pendant_command_executor.urscript)，主动连到本节点监听的端口；
本节点把 ROS 话题转成 socket 命令发给执行器，实现随时抓/松/移动。

用法:
    ros2 run ur_link ur_gripper_cmd --ros-args -p port:=30010
话题:
    /ur_cmd/gripper  std_msgs/Int32        0=松开, 255=闭合
    /ur_cmd/move     std_msgs/Float64MultiArray  [x,y,z,rx,ry,rz]
"""
import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32


class UrGripperCmd(Node):
    def __init__(self):
        super().__init__("ur_gripper_cmd")
        self.port = self.declare_parameter("port", 30010).value
        self._conn = None
        self._lock = threading.Lock()

        self.create_subscription(Int32, "ur_cmd/gripper", self._on_gripper, 10)
        self.create_subscription(Float64MultiArray, "ur_cmd/move", self._on_move, 10)

        self._server_thread = threading.Thread(target=self._server, daemon=True)
        self._server_thread.start()
        self.get_logger().info(f"ur_gripper_cmd 监听 :{self.port}，等示教器执行器连入...")

    def _server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self.port))
        srv.listen(1)
        while rclpy.ok():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            conn.settimeout(10)
            with self._lock:
                if self._conn:
                    try:
                        self._conn.close()
                    except OSError:
                        pass
                self._conn = conn
            self.get_logger().info(f"执行器已连入: {addr}")

    def _send(self, data: bytes) -> bool:
        with self._lock:
            if not self._conn:
                self.get_logger().warn("执行器未连接！请先播放示教器通用执行器程序")
                return False
            try:
                self._conn.sendall(data)
                # 读回执(ok)
                self._conn.settimeout(5)
                buf = b""
                try:
                    while not buf.endswith(b"\n"):
                        c = self._conn.recv(1)
                        if not c:
                            break
                        buf += c
                except socket.timeout:
                    pass
                self._conn.settimeout(10)
                if buf.strip():
                    self.get_logger().info(f"执行器回执: {buf.decode(errors='replace').strip()!r}")
                return True
            except OSError as e:
                self.get_logger().error(f"发送失败: {e}")
                self._conn = None
                return False

    def _on_gripper(self, msg: Int32):
        if msg.data <= 127:
            self._send(b"release\n")
        else:
            self._send(b"grip\n")

    def _on_move(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) != 6:
            self.get_logger().warn(f"move 需要 6 个值，收到 {len(d)}")
            return
        line = ",".join(f"{v:.6f}" for v in d)
        self._send(b"move\n")
        with self._lock:
            if self._conn:
                try:
                    self._conn.sendall((line + "\n").encode())
                except OSError:
                    self._conn = None

    def destroy_node(self):
        if self._conn:
            try:
                self._conn.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UrGripperCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
