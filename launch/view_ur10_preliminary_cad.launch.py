#!/usr/bin/env python3
"""Read-only live RViz view of the preliminary end-effector collision model."""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

ROOT = os.path.expanduser("~/UR10")
URDF = os.path.join(ROOT, "outputs/vision/ur10_preliminary_cad_meshes.urdf")


def generate_launch_description():
    with open(URDF, encoding="utf-8") as stream:
        robot_description = stream.read()
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="ur10_preliminary_model_publisher", output="screen",
             parameters=[{"robot_description": robot_description}]),
        ExecuteProcess(cmd=["python3", os.path.join(ROOT, "scripts/publish_joint_states.py"),
                            "--hz", "50"], output="screen"),
        ExecuteProcess(cmd=["rviz2", "-d", os.path.join(ROOT, "config/rviz_ur10_preliminary_cad.rviz")],
                       output="screen"),
    ])
