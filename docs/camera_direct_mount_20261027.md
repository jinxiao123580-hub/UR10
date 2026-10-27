# Mech-Eye PRO XS 相机直接安装模型（2026-10-27）

相机拆除橙色中间连接件，直接安装到保留的灰色连接件上。本版依据用户最终确认的实物照片：**长边水平、宽侧面贴支架侧面、相机顶部与贴靠处的支架上表面齐平**。

![旧装配与最终模型](../model/camera_direct_mount_20261027/assembly_comparison.png)

## 几何与来源

原 `末端执行器 - 3D相机-1.STL` 是相机与中间支架的布尔合并体。新脚本从已有相机八个角点恢复长方体，并补齐分离后的接触面，不改变力传感器、灰色连接件和夹爪。

保留原 CAD 简化相机的 130 × 85 × 50 mm 外形，安装后 CAD X/Y/Z 尺寸为 50 × 85 × 130 mm。这是源 CAD 的外形，不是重新测量的产品尺寸。130 × 85 mm 宽面贴在连接件最小 X 侧面；顶部 CAD Y 为 175.73761 mm。传感器安装座另有 13 mm 高台阶，不能用它对齐相机。CAD Z 中心对齐。安装孔和光学原点未由此确认。

详细参数：[assembly.json](../model/camera_direct_mount_20261027/assembly.json)。相机外壳中心在 tool0 下约为 `[0, 0.155, 0.1055]` m；长边垂直于工具轴，在示意书写姿态下水平。名义观察方向为 tool0 +Z，会随末端姿态转动。

## 文件与离线生成

新网格是 `model/solidworks/end_effector/camera_direct_mount_20261027.stl`。原 STEP 和组合 STL 作为历史来源保留。网页默认加载新网格；CAD URDF 默认也使用新模型。

在仓库根目录运行，以下仅生成文件，不访问设备：

```bash
python3 scripts/camera_direct_mount.py   --source-dir model/solidworks/end_effector   --output model/solidworks/end_effector/camera_direct_mount_20261027.stl   --report model/camera_direct_mount_20261027/assembly.json
python3 scripts/build_urdf_with_cad_meshes.py
```

使用项目现有 NumPy/SciPy、YAML 和几何运行环境。后一入口保留已有工厂 URDF、TCP 记录与装配约定；`--camera-mount legacy` 生成历史组合装配。生成的 `outputs/vision/ur10_preliminary_cad_meshes.urdf` 和 `cad_meshes/cad_attachment_1.stl` 同步更新相机视觉与碰撞网格。已提交 URDF 的绝对资源地址沿用既有 jx 路径，其他主机应重新生成。

## MU ROS 同步与验证范围

[本机 ROS 模型快照及说明](../model/camera_direct_mount_20261027/ros2/README.md)包含本次更新后的 URDF、运动学/碰撞包络和来源记录，可供共同维护时核对。它不是完整 ROS 工作区。

已核对：闭合长方体、原始外形尺寸保留、宽面贴合、相机端顶部齐平、URDF XML、网页 JavaScript 语法、相机网格与碰撞包络一致。本机 Beta 包及其仓库副本各有 7 项模型/仿真启动约束检查通过。未进行新的实机标定或运动验证。

本次提交只更新模型、显示与模型资料，不修改手眼标定文件及其验证状态，也不替换实机控制中的附件保护配置。不能把新外壳模型变换当作新的 `tool0_from_camera` 光学标定。
