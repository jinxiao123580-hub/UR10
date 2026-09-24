# UR10-specific Mech-Eye launch: direct IP connection, no fake static TF.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mecheye_ros_interface",
            executable="start",
            name="mechmind_camera_publisher_service",
            output="screen",
            parameters=[
                {"save_file": False},
                {"camera_ip": "192.168.1.33"},
                {"camera_fw_version": "2.5.0"},
            ],
        ),
    ])
