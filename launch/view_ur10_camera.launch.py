#!/usr/bin/env python3
"""Show the live UR10 + camera assembly in RViz.

Starts:
  * robot_state_publisher  - URDF (with the camera assembly) -> TF
  * publish_joint_states.py - the controller's q_actual (30003) -> /joint_states
  * rviz2                  - with the bundled config

Read-only with respect to the robot: nothing here opens 30002.

    source /opt/ros/humble/setup.bash
    ros2 launch ~/UR10/launch/view_ur10_camera.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

ROOT = os.path.expanduser("~/UR10")


def generate_launch_description():
    with open(os.path.join(ROOT, "outputs/vision/ur10_with_camera.urdf"),
              encoding="utf-8") as stream:
        robot_description = stream.read()
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(ROOT, "scripts/publish_joint_states.py"),
                 "--hz", "50"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["rviz2", "-d", os.path.join(ROOT, "config/rviz_ur10_camera.rviz")],
            output="screen",
        ),
    ])
