from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    start_state_node = LaunchConfiguration("start_state_node")
    robot_ip = LaunchConfiguration("robot_ip")
    tf_prefix = LaunchConfiguration("tf_prefix")
    gripper_opening = LaunchConfiguration("gripper_opening")
    open_rviz = LaunchConfiguration("rviz")

    package_share = FindPackageShare("ur10_digital_twin")
    model = PathJoinSubstitution([package_share, "urdf", "ur10_cell.urdf.xacro"])
    rviz_config = PathJoinSubstitution([package_share, "rviz", "digital_twin.rviz"])
    robot_description = ParameterValue(
        Command(["xacro ", model, " tf_prefix:=", tf_prefix, " ur_type:=ur10"]),
        value_type=str,
    )

    real_mode = IfCondition(PythonExpression(["'", mode, "' == 'real'"]))
    offline_mode = IfCondition(PythonExpression(["'", mode, "' == 'offline'"]))

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="real", description="real or offline"),
        DeclareLaunchArgument("start_state_node", default_value="false",
                              description="Start ur_link state reader in real mode"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.3"),
        DeclareLaunchArgument("tf_prefix", default_value="twin_"),
        DeclareLaunchArgument("gripper_opening", default_value="0.04"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="ur_link",
            executable="ur_state_node",
            name="digital_twin_ur_state_node",
            parameters=[{"robot_ip": robot_ip}],
            condition=IfCondition(PythonExpression([
                "'", mode, "' == 'real' and '", start_state_node, "' == 'true'",
            ])),
            output="screen",
        ),
        Node(
            package="ur10_digital_twin",
            executable="demo_joint_state",
            condition=offline_mode,
            output="screen",
        ),
        Node(
            package="ur10_digital_twin",
            executable="joint_state_bridge",
            parameters=[{
                "tf_prefix": tf_prefix,
                "gripper_opening": gripper_opening,
            }],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="digital_twin_robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", "/digital_twin/joint_states")],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(open_rviz),
            output="screen",
        ),
    ])
