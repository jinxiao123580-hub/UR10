# 交接文档 · UR10 视觉力觉抓取系统

最后更新：2026-09-18。本文只保留当前可用状态；原始数据、失败实验和历史过程在
`outputs/`、`experiments/` 与 `legacy/`。

## 0A. 最新接手状态

### 设备与环境

| 项目 | 当前值 |
|---|---|
| 主机 | Ubuntu 22.04，ROS 2 Humble，用户 `jx` |
| UR10 CB3 | `192.168.1.3`，URSoftware `3.15.8.106339` |
| ATI Net F/T | `192.168.1.2`，RDT UDP `49152`，只允许一个消费者 |
| Mech-Eye PRO XS | `192.168.1.33`，SDK `2.5.0` |
| Robotiq | 机器人 `192.168.1.3:63352`，电脑直控 ASCII |

### 已独立验证的结论

- 官方 `ur_robot_driver 2.14.0` 可在 ROS 2 Humble 启动；工厂运动学文件为
  `config/ur10_factory_calibration.yaml`。录制姿态的 FK 对控制器 TCP 最大偏差
  `0.017312 mm / 0.004694°`；机器人基坐标必须使用 `base`，不能使用 `base_link`。
- Eye-in-hand 棋盘格标定使用 `10×7` 方格、`9×6` 内角点、格长 `6 mm`。Park 解在
  5 个留出样本的闭环 RMS 为 `2.642 mm / 1.051°`，最大 `3.855 mm / 1.837°`；冻结后
  一组独立样本为 `2.614 mm / 0.401°`。参数在
  `config/handeye_eye_in_hand_20260917.yaml`。
- 图像 PnP 与组织点云逐角点验收 54/54 有效，XYZ RMS `0.692 mm`、P95 `1.197 mm`、
  最大 `1.332 mm`。
- ATI 原始数据稳定发布到 `/ft_sensor/wrench_raw`，约 200 Hz；UR 30003/30013 状态约
  125 Hz。Mech-Eye 采集接口正常，但所谓颜色图和纹理点云实测均为单色。
- 一次物块抓起、提升和原位放回曾通过，但使用的是“视觉候选 + 人工修正”。视觉候选到
  人工确认中心的 XY 偏差为 `35.84 mm`，因此自主视觉抓取尚未通过。
- 当前保留的位置式抓放示例为 `scripts/position_pick_place.py`，使用控制器原生 `movel`
  和 Robotiq ASCII，不使用 ATI 或重力补偿；`config/position_pick_place.example.json` 中的
  点位仅用于离线示例，真机执行前必须现场重新确认。

### 当前严格禁止

- 不执行电脑端流式 IK 真机运动；此前 2 mm 流式实验触发 `C153A0` 保护停。
- 不把 ATI 重力补偿用于力控、碰撞停止或抓取判定。所有现有重力模型均为 `valid: false`；
  新数据只用 `/ft_sensor/wrench_raw`。
- 不运行历史抓放程序；它含未独立验证的 `8.0 kg / [-0.03, 0.03, -0.06]` 负载估计。
- 不把控制器回读的 `3.5 kg / [0, 0, 0.1] m` 当作独立测量结果。
- 不把 Mech-Eye 的颜色话题当作真彩色：PRO XS 的 2D 传感器是单色，属硬件规格而非故障，
  `/mechmind/color_image` 的 bgr8 三通道由 SDK 按 `Blue = Gray, Green = Gray, Red = Gray`
  复制而来。型号证据见 `outputs/camera/model-identification-20260918.json`。

### 当前可直接执行的只读检查

```bash
cd ~/UR10
python3 scripts/diagnose.py
python3 scripts/verify_ur_payload.py
python3 scripts/monitor_ur_safety.py --duration 3
python3 scripts/check_current_docs.py
```

相机验证需另开终端（重启后重新执行）：

```bash
# 终端 A
cd ~/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch ~/colcon_ws/src/mecheye_ros2_interface/launch/start_camera_ur10.py

# 终端 B
cd ~/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
python3 scripts/validate_mecheye_capture.py
```

完整的当前入口见 [`docs/00-当前可用操作.md`](docs/00-当前可用操作.md)。

## 下一步

1. 完成课程第 4 节：以闭合左指尖进行固定点多姿态 TCP 标定。采 12 个静态样本、留出 3 个
   独立验证；只读脚本是 `scripts/calibrate_fingertip_tcp.py`。
2. 课程第 5 节的 ATI 重力补偿仍需重新建模：先调研官方/同行方法，再采集足够正负方向姿态；
   模型必须用未参与拟合的姿态独立验收，未通过不得恢复补偿进程。
3. 课程第 6 节先解决视觉物体中心定位：在多个重新摆放的已知物块上测量中心误差；通过门限前
   只允许视觉悬停验证和人工确认，禁止自动下降抓取。
4. 所有新的运动方案遵循：资料依据 → 离线数值门禁 → 官方假硬件 → 单段低速真机 → 完整流程。
   每一层保存数值证据。

## 资料与存档规则

- 所有结论必须有数值证据；失败实验写 `experiments/`，原始 CSV/JSON 写 `outputs/`。
- 只有独立验证通过的稳定结论才能写入 `docs/`。
- 更新脚本、接口、路径或有效操作方式时，同一提交中更新 `docs/00-当前可用操作.md`、本文件和
  相关专题文档，并运行 `python3 scripts/check_current_docs.py`。
- 新方案实施前先查官方文档、论文或原始实现，记录来源、版本、适用条件及本机差异。
