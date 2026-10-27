# MU 毛笔 ROS 仿真模型快照

本目录不是一个完整的 ROS 包，是 2026-10-27 同步到 MU 现有 `fusion_model_ros2_beta` 的模型相关文件。依赖已有包中的 launch、UR10 本体网格与控制器配置。

- `fusion_model_ros2_beta/urdf/ur10_official.urdf`：本机实际 CAD 装配模型，硬件插件为 `mock_components/GenericSystem`。
- `fusion_model_ros2_beta/fusion_model_ros2_beta/ur10_actual_kinematics.py`：现有工厂运动学及更新后的相机碰撞包络。
- `fusion_model_ros2_beta/config/camera_mount_model_20261027.yaml`：模型来源与状态。
- 相机网格使用仓库的 `model/solidworks/end_effector/camera_direct_mount_20261027.stl`，在已有 Beta 包中对应 `meshes/end_effector/end_effector_camera.stl`。
- 其余三个末端网格复用本仓库原 SolidWorks 文件；对应已有包的 `end_effector_force_sensor.stl`、`end_effector_connector.stl`、`end_effector_2f85_closed.stl`，没有修改。

本机旧光学帧保留为 `mecheye_camera_optical_frame_legacy_20260921`。它不代表本次新安装；新模型位置是外壳位置，不是光学原点或新的手眼标定。

MU 上源目录与 `install_actual_ur10` 的既有符号链接已核对一致，无需重建即可在下次启动时加载。没有启动 ROS 图、发布轨迹或操作真实机械臂。本机模型/仿真启动约束 7 项检查通过；网格与碰撞包络一致性通过。
