#!/usr/bin/env python3
"""Publish the controller's live joint state on /joint_states for RViz.

Read-only: it opens a UR realtime port, reads ``q_actual``, and republishes it as a
``sensor_msgs/JointState``.  It never opens 30002 and never commands motion.

Pair it with ``robot_state_publisher`` (which turns the URDF plus these joint
values into TF) and RViz, and the render follows the real arm:

    # terminal 1
    python3 scripts/publish_joint_states.py
    # terminal 2
    source /opt/ros/humble/setup.bash
    ros2 run robot_state_publisher robot_state_publisher \\
        --ros-args -p robot_description:="$(cat outputs/vision/ur10_with_camera.urdf)"
    # terminal 3
    rviz2 -d config/rviz_ur10_camera.rviz
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--port", type=int, default=30003,
                        help="UR read-only realtime port")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--topic", default="/joint_states")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from record_ur_trajectory import RealtimeReader

    rclpy.init()
    node = Node("ur10_joint_state_relay")
    publisher = node.create_publisher(JointState, args.topic, 10)
    reader = RealtimeReader(args.host, args.port)
    node.get_logger().info("publishing %s from %s:%d at %.0f Hz" %
                           (args.topic, args.host, args.port, args.hz))
    period = 1.0 / max(args.hz, 1.0)
    published = 0
    try:
        while rclpy.ok():
            started = time.monotonic()
            try:
                q, _tcp, velocity = reader.read()
            except Exception as exc:  # keep the relay alive across reconnects
                node.get_logger().warn("state read failed: %r" % exc)
                time.sleep(0.5)
                continue
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = [float(value) for value in np.asarray(q, dtype=float)]
            message.velocity = [float(value) for value in np.asarray(velocity,
                                                                     dtype=float)]
            publisher.publish(message)
            published += 1
            if published % 500 == 0:
                node.get_logger().info("published %d joint states" % published)
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
