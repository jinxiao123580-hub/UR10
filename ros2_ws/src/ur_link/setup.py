from setuptools import find_packages, setup

package_name = "ur_link"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/ur_link.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UR Lab",
    maintainer_email="lab@example.com",
    description="Minimal UR ROS2 bridge over raw TCP client interfaces",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ur_state_node = ur_link.state_node:main",
            "ur_command_node = ur_link.command_node:main",
            "ur_gripper_cmd = ur_link.gripper_cmd:main",
        ],
    },
)
