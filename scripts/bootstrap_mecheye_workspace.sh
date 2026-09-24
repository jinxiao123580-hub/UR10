#!/usr/bin/env bash
# Recreate the non-vendored Mech-Eye ROS package used by this repository.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace=${1:-"$HOME/colcon_ws"}
package="$workspace/src/mecheye_ros2_interface"
commit=0123b8bea87ef865958417630088e9c5e48fdb44
patch="$root/third_party/mecheye_ros2_interface/ur10-camera-2.5.0.patch"
launch="$root/third_party/mecheye_ros2_interface/start_camera_ur10.py"

if ! command -v ros2 >/dev/null; then
  echo "ROS 2 Humble must be installed and sourced first." >&2
  exit 2
fi
if [ ! -d "$package/.git" ]; then
  mkdir -p "$workspace/src"
  git clone https://github.com/MechMindRobotics/mecheye_ros2_interface.git "$package"
fi
git -C "$package" fetch origin
git -C "$package" checkout "$commit"
if git -C "$package" apply --reverse --check "$patch" >/dev/null 2>&1; then
  echo "UR10 Mech-Eye patch already applied."
else
  git -C "$package" apply "$patch"
fi
install -m 0644 "$launch" "$package/launch/start_camera_ur10.py"
source /opt/ros/humble/setup.bash
cd "$workspace"
colcon build --packages-select mecheye_ros_interface
echo "Done. Source $workspace/install/setup.bash before starting the camera."
