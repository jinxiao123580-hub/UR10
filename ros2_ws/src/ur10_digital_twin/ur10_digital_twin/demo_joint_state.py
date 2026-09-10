"""Deterministic offline motion source for digital-twin verification."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .joint_state_bridge import ARM_JOINTS


class DemoJointState(Node):
    def __init__(self):
        super().__init__("digital_twin_demo_joint_state")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.started_ns = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.05, self._publish)
        self.get_logger().info("离线模式：发布 20Hz 可重复关节运动")

    def _publish(self):
        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.started_ns) / 1e9
        pose = [
            0.35 * math.sin(0.35 * elapsed),
            -1.15 + 0.22 * math.sin(0.27 * elapsed + 0.5),
            1.35 + 0.25 * math.sin(0.31 * elapsed + 1.1),
            -1.55 + 0.20 * math.sin(0.41 * elapsed),
            -1.57 + 0.18 * math.sin(0.23 * elapsed + 0.7),
            0.30 * math.sin(0.37 * elapsed + 1.7),
        ]
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(ARM_JOINTS)
        msg.position = pose
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DemoJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
