# -*- coding: utf-8 -*-
"""UR 状态节点：连 30001 Primary 客户端接口，解析关节状态并发布。"""
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, UInt8

from .ur_protocol import URStreamClient

# UR10/CB3 标准关节名(与 ur_description/urdf 一致)
JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class UrStateNode(Node):
    def __init__(self):
        super().__init__("ur_state_node")
        self.robot_ip = self.declare_parameter("robot_ip", "192.168.1.3").value
        self.state_port = self.declare_parameter("state_port", 30001).value
        self.reconnect_s = self.declare_parameter("reconnect_seconds", 2.0).value

        self.js_pub = self.create_publisher(
            JointState, "joint_states", QoSProfile(depth=5))
        self.prog_pub = self.create_publisher(Bool, "ur_link/is_program_running", 5)
        self.mode_pub = self.create_publisher(UInt8, "ur_link/robot_mode", 5)
        self.speed_pub = self.create_publisher(Float64, "ur_link/speed_fraction", 5)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"ur_state_node -> {self.robot_ip}:{self.state_port}")

    def _loop(self):
        while not self._stop.is_set():
            client = URStreamClient(self.robot_ip, self.state_port)
            try:
                client.connect()
                self.get_logger().info("已连接 Primary 客户端接口(30001)，开始读状态")
            except OSError as e:
                self.get_logger().error(f"连接失败: {e}，{self.reconnect_s}s 后重试")
                self._stop.wait(self.reconnect_s)
                continue
            try:
                while not self._stop.is_set():
                    st = client.read_state()
                    if st is None:
                        continue
                    self._publish(st)
            except (ConnectionError, OSError) as e:
                self.get_logger().warn(f"读流断开: {e}，重连中...")
            finally:
                client.close()

    def _publish(self, st):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES[:len(st.q_actual)]
        js.position = [float(q) for q in st.q_actual]
        js.velocity = [float(v) for v in st.qd_actual]
        js.effort = [float(c) for c in st.current]
        self.js_pub.publish(js)

        b = Bool(); b.data = bool(st.is_program_running)
        self.prog_pub.publish(b)
        mode = int(st.robot_mode)
        if mode < 0 or mode > 255:
            mode = 0
        m = UInt8(); m.data = mode
        self.mode_pub.publish(m)
        s = Float64(); s.data = float(st.speed_fraction)
        self.speed_pub.publish(s)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UrStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
