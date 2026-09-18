# UR10 视觉力觉抓取实验平台

UR10 CB3、Robotiq 二指夹爪、ATI Net F/T 与 Mech-Eye PRO XS 的课程与实验仓库。

**唯一的接手入口是 [`HANDOVER.md`](HANDOVER.md) 顶部的 `0A. 最新接手状态`。** 它覆盖旧
实验记录和历史文档中的结论。

## 现在可直接使用

先阅读 [`docs/00-当前可用操作.md`](docs/00-当前可用操作.md)。其中只保留已验收的、当前
环境可运行的命令；所有运动都必须由单独的已验收实验方案授权。

```bash
cd ~/UR10
python3 scripts/diagnose.py
python3 scripts/verify_ur_payload.py
python3 scripts/monitor_ur_safety.py --duration 3
```

相机验证需单独启动相机终端（重启后重新执行）：

```bash
# 终端 A：启动 Mech-Eye
cd ~/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch ~/colcon_ws/src/mecheye_ros2_interface/launch/start_camera_ur10.py

# 终端 B：采集并保存验证数据
cd ~/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
python3 scripts/validate_mecheye_capture.py
```

## 当前边界

- 不执行电脑端流式 IK 真机运动。
- 不把未独立验证的 ATI 重力补偿用于控制或抓取判定。
- 不运行历史抓放脚本；其负载参数未验证。
- 相机图像虽有颜色话题名，但 PRO XS 的 2D 传感器是单色（硬件规格），实测 bgr8 三通道逐像素相同。
- 视觉物体中心尚未独立验收；已完成的抓放含人工修正，不是自主抓取。
- 保留的抓放示例是 [`scripts/position_pick_place.py`](scripts/position_pick_place.py)，仅为已确认点位的位置控制，不含力控；当前只通过 dry-run，尚未完成本机真机抓放验收。
- 夹爪实时反馈验证使用 [`scripts/verify_gripper_motion.py`](scripts/verify_gripper_motion.py)，不移动机械臂。

## 文档导航

| 文档 | 当前用途 |
|---|---|
| [`docs/00-当前可用操作.md`](docs/00-当前可用操作.md) | 已验证命令和禁用入口 |
| [`docs/01-硬件与网络.md`](docs/01-硬件与网络.md) | 硬件、网络和端口 |
| [`docs/02-官方ROS2驱动.md`](docs/02-官方ROS2驱动.md) | 官方 ROS 2 驱动启动与只读确认 |
| [`docs/04-夹爪控制.md`](docs/04-夹爪控制.md) | Robotiq ASCII 通道和状态解释 |
| [`docs/08-六维力与3D相机.md`](docs/08-六维力与3D相机.md) | ATI、Mech-Eye 接入和坐标约定 |
| [`docs/09-MechEye相机接入清单.md`](docs/09-MechEye相机接入清单.md) | 相机接入与采集验收 |
| [`docs/11-机器人自身标定.md`](docs/11-机器人自身标定.md) | TCP、负载和运动学标定 |
| [`docs/12-力传感器重力补偿.md`](docs/12-力传感器重力补偿.md) | 重力补偿的失败结论与限制 |
| [`docs/14-待办与已知问题.md`](docs/14-待办与已知问题.md) | 未完成项目的唯一清单 |

`experiments/` 和 `legacy/` 存放历史过程与失败实验，不是操作教程。新结论必须保存原始数据，
经独立验证后才写入 `docs/`。
