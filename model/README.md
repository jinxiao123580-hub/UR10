# 末端模型

本目录是 UR10 仓库中唯一的末端模型源，不依赖本机的 ROS 工作区路径。

| 路径 | 用途 | RViz 状态 |
| --- | --- | --- |
| `solidworks/end_effector/` | 用户从 SolidWorks 导出的完整末端装配：ATI 六轴力传感器、连接件、Robotiq 2F-85（闭合）和 Mech-Eye 相机 STL；另保留原始 STEP。 | 当前 CAD 可视化来源 |
| `collision/ur10_cell.collision.xacro` | 根据实测尺寸建立的简化包络碰撞模型。 | 仅供碰撞/间隙初步检查，不能作为真机安全判据 |

## 查看完整 CAD

```bash
cd ~/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
python3 scripts/build_urdf_with_cad_meshes.py
ros2 launch launch/view_ur10_preliminary_cad.launch.py
```

生成的 `outputs/vision/ur10_preliminary_cad_meshes.urdf` 和其中的 `cad_meshes/` 是可直接由 RViz 载入的副本；源文件始终在本目录。装配相对 `tool0` 的初版安装方向由 `config/robot_attachments.preliminary-cad-20260921.yaml` 固化，当前为已人工确认的 `tool_yaw_deg: 270`。
