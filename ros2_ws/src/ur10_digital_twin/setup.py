from glob import glob
import os

from setuptools import find_packages, setup


package_name = "ur10_digital_twin"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UR Lab",
    maintainer_email="lab@example.com",
    description="Read-only RViz digital twin for the UR10 grasping cell.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "joint_state_bridge = ur10_digital_twin.joint_state_bridge:main",
            "demo_joint_state = ur10_digital_twin.demo_joint_state:main",
        ],
    },
)
