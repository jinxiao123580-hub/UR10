"""Copy real UR joint states into an isolated, prefixed digital-twin tree."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("digital_twin_joint_state_bridge")
        self.prefix = self.declare_parameter("tf_prefix", "twin_").value
        self.input_topic = self.declare_parameter("input_topic", "/joint_states").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/digital_twin/joint_states").value
        self.gripper_opening = float(
            self.declare_parameter("gripper_opening", 0.04).value)

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.publisher = self.create_publisher(JointState, self.output_topic, 10)
        self.subscription = self.create_subscription(
            JointState, self.input_topic, self._on_joint_state, input_qos)
        self.received = 0
        self.get_logger().info(
            f"只读同步 {self.input_topic} -> {self.output_topic} (prefix={self.prefix})")

    def _on_joint_state(self, source):
        values = dict(zip(source.name, source.position))
        missing = [name for name in ARM_JOINTS if name not in values]
        if missing:
            if self.received == 0:
                self.get_logger().warning("输入缺少关节: " + ", ".join(missing))
            return

        target = JointState()
        target.header = source.header
        target.name = [self.prefix + name for name in ARM_JOINTS]
        target.position = [float(values[name]) for name in ARM_JOINTS]
        target.name.extend([
            self.prefix + "left_finger_joint",
            self.prefix + "right_finger_joint",
        ])
        target.position.extend([self.gripper_opening, self.gripper_opening])
        self.publisher.publish(target)
        self.received += 1


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
