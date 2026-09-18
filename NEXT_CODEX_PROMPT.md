# 给下一个 Codex 的接手提示词

你接手的是 `/home/jx/UR10` 的 UR10 CB3 + Robotiq 夹爪 + ATI Net F/T + Mech-Eye PRO XS 项目。

第一步必须阅读：

1. `HANDOVER.md`，尤其是顶部 `0A. 最新接手状态`；
2. `docs/14-待办与已知问题.md`；
3. `docs/12-力传感器重力补偿.md`；
4. `docs/09-MechEye相机接入清单.md`；
5. `experiments/2026-09-16-官方ROS2轨迹控制接入与未完成门禁.md`。

不要假设旧文档中的“已完成”仍然代表当前可用，当前状态以 `HANDOVER.md` 顶部为准。

## 已知硬件

- 电脑：Ubuntu 22.04、ROS 2 Humble，用户 `jx`。
- UR10 CB3：`192.168.1.3`，URSoftware `3.15.8.106339`。
- ATI Net F/T：`192.168.1.2`，RDT UDP `49152`，只能有一个消费者。
- Mech-Eye PRO XS：`192.168.1.33`，SDK 2.5.0。
- 夹爪：机器人 `192.168.1.3:63352`，电脑直控 Robotiq ASCII。

## 当前禁止事项

- 不要执行 `scripts/test_ik_move.py --execute`。此前 2 mm 流式 IK 真机试验触发 `C153A0` 保护停。
- 不要使用重力补偿话题做力控、碰撞停止或抓取判断。旧模型独立验证失败，配置已标记 `valid: false`。
- 只使用 `/ft_sensor/wrench_raw` 做新的重力数据采集。
- 不要运行 `scripts/ur_pick_place_full.py`。该脚本仍使用未验证的
  `8.0 kg / [-0.03,0.03,-0.06]` 负载估计值。
- 不要把控制器当前回读的 `3.5 kg / [0,0,0.1] m` 当成独立测量结果。
- 不要把 `/mechmind/color_image` 当成真彩色。PRO XS 的 2D 传感器是单色（硬件规格，
  官方规格表 `2D image color: Monochrome`，型号表无 `C` 后缀彩色版本），实测 bgr8
  三通道完全相同、纹理点云 RGB 也完全相同，因为 SDK 的 `getColorImage()` 在
  Monochrome 时按 `Blue = Gray, Green = Gray, Red = Gray` 复制。这不是待修复的故障，
  不要为“恢复彩色”排查参数或链路。证据见 `outputs/camera/model-identification-20260918.json`。

## 已验证工具

```bash
cd ~/UR10
python3 scripts/diagnose.py
python3 scripts/verify_ur_payload.py
python3 scripts/monitor_ur_safety.py --duration 3
python3 scripts/validate_mecheye_capture.py
```

## 下一步任务

当前主目标已经改为手眼标定。重力补偿暂停，其数据索引在
`archives/ft_gravity_20260916/README.md`，不得删除或把任何候选改成 `valid: true`。

任何新方案实施前，必须先调研行业常用方法和官方推荐做法，优先查 UR/ATI/ROS 2
官方版本化文档及原始论文/实现，记录来源、适用条件和与本机 CB3 3.15.8 的差异，
然后再写代码或上真机。

先现场确认相机安装类型和标定板型号：相机随末端运动用 eye-in-hand，固定外置用
eye-to-hand。先核对 Mech-Eye SDK 2.5.0/Mech-Vision 官方标定板检测能力，再决定
使用官方标准标定还是 easy_handeye2 + 独立目标检测。初期只用示教器手动采样，
至少 15 个有效姿态，不自动运动。结果必须做留出姿态和已知空间点独立验收。

已知标定板格子边长为 `0.6 cm = 0.006 m`；仍需现场确认图案类型、横纵内角点/
圆心数量，不能用方格数量代替棋盘格内角点数量。

建立含夹爪和偏心相机的碰撞模型后，才可在用户监护下进行单段低速真机试验；
不得直接完整回放。自动采集后的重力模型仍必须用未参与拟合的姿态独立验收。

所有结论必须有数值证据。失败实验写到 `experiments/`，原始 CSV/JSON 写到 `outputs/`，只有独立验证通过的结论写入 `docs/`。本轮不需要推送 GitHub，先在本地完成并报告：做了什么、证据是什么、下一步是什么。
