"""launch 文件：ur_state_node + ur_command_node。用法：
ros2 launch ur_link ur_link.launch.py robot_ip:=192.168.1.3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.3",
                              description="UR 控制柜 IP"),
        Node(package="ur_link", executable="ur_state_node", name="ur_state_node",
             parameters=[{"robot_ip": robot_ip}]),
        Node(package="ur_link", executable="ur_command_node", name="ur_command_node",
             parameters=[{"robot_ip": robot_ip}]),
    ])
