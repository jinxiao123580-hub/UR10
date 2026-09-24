# 新会话用 Prompt：UR10 眼在手上棋盘格标定（重标 + 独立验收）

> 用法：把下面 `===== PROMPT 开始 =====` 与 `===== PROMPT 结束 =====` 之间的全文
> 复制到新对话里作为第一条消息。它是自包含的：新会话不需要看这段对话的历史。

===== PROMPT 开始 =====

我在做 UR10 机械臂工程实践课程的第 5 节（机器人感知），本轮任务是**眼在手上（eye-in-hand）
棋盘格手眼标定：重新标定 + 独立验收**。请先只读摸清现状，再按下面的门禁推进。

## 0. 先读这三个文件（不要假设旧文档里的"已完成"仍然有效）

1. `~/UR10/HANDOVER.md` 顶部「0A. 最新接手状态」——**唯一权威状态**，含「当前严格禁止」清单；
2. `~/UR10/docs/00-当前可用操作.md` —— 当前可执行入口；
3. `~/UR10/experiments/2026-09-18-视觉中心误差差向量诊断P0-B.md` —— 视觉几何的已知错因（必读，
   里面那条 50 mm 符号错误很容易在新代码里重犯）。

## 1. 设备与已验证现状

| 项目 | 值 |
|---|---|
| 主机 | Ubuntu 22.04 + ROS 2 Humble，用户 `jx`，仓库 `~/UR10` |
| UR10 CB3 | `192.168.1.3`，URSoftware `3.15.8.106339` |
| Mech-Eye PRO XS | `192.168.1.33`，SDK `2.5.0`，SN `RAM35238A3020004`（**2D 传感器是单色**，`/mechmind/color_image` 是灰度复制，不是故障） |
| ATI Net F/T | `192.168.1.2`，RDT UDP `49152`，**单消费者**（同时只能一个实例收流） |
| 棋盘格 | `10×7` 方格、`9×6` 内角点、格长 `6 mm` |
| 已有标定 | `config/handeye_eye_in_hand_20260917.yaml`（Park 解，`tool0_from_camera`，15 训练 + 5 留出；留出闭环 RMS `2.642 mm / 1.051°`，最大 `3.855 mm / 1.837°`；冻结后独立样本 `2.614 mm / 0.401°`） |
| **棋盘格位置** | **今天（2026-09-18）已被移动过** ⇒ 旧标定的绝对坐标不再对应现场，必须重标 |
| 机械臂位姿 | 当前 TCP ≈ `(0.647, -0.268, 0.276) m`，看不到旧场景；需要现场用**示教器**把臂点动到能同时看到棋盘格与物块的位姿 |

## 2. 硬性约束（违反即停）

- **禁止**电脑端流式 IK 真机运动（历史上触发过 `C153A0` 保护停）。
- **禁止**运行历史抓放程序（含未独立验证的 `8.0 kg / [-0.03,0.03,-0.06]` 负载估计）。
- **禁止**把 ATI 重力补偿用于力控、碰撞停止或抓取判定（所有现有模型 `valid: false`）。
- **禁止**把控制器回读的 `3.5 kg / [0,0,0.1] m` 当作独立测量结果。
- ATI 需要 RDT 数据时只允许**一个** `scripts/ati_netft_node.py` 实例；仪表盘在跑就会占着流。
- 真机运动前的顺序：**资料依据 → 离线数值门禁 → 官方假硬件 → 单段低速真机 → 完整流程**，
  每层保存数值证据。
- 采集期间机器人必须**静止**：脚本自带门禁 `max_position_motion_mm ≤ 0.5`、
  `max_rotation_motion_deg ≤ 0.2`，超限必须丢样本重采。
- 证据规则：**失败写 `experiments/`，原始 CSV/JSON/点云写 `outputs/`，只有独立验证通过的
  稳定结论才进 `docs/`**。
- **可能有另一个 agent 在同一仓库并行工作**：开工前跑 `git log --oneline -5` 与
  `ps aux | grep -E "codex|ros2|python3"` 看一眼；若它在给机器人发指令，**不要同时动**；
  也避免与它同时改 `HANDOVER.md` / `docs/00` / `README.md`（今天的提交就是它做的）。

## 3. 开工前必须先做的三项检查（全部只读）

```bash
cd ~/UR10
python3 scripts/diagnose.py                     # 网络/端口/模式，期望全绿
python3 scripts/read_ur_state.py                # 期望 robotmode RUNNING / safetystatus NORMAL
python3 scripts/check_current_docs.py           # 结束时也要跑
```

**相机链路必须先确认（今天已知异常）**：我在 2026-09-18 实测
`ros2 service call /device_info`、`/capture_color_image` 与
`python3 scripts/validate_mecheye_capture.py` 全部**超时无响应**（40 s / 60 s），
重启相机节点无效；相机硬件本身正常（SDK 日志显示已连上相机并打印型号/序列号），
ROS 服务机制也正常（本地自检服务可正常应答）。见
`experiments/2026-09-18-入口可用性审计.md` §3.2。存在两种可能：真实链路故障，
或仅是我那次运行环境（只读沙箱，SDK 报 `No write permission for directory /var/log/`）导致。

所以你要做的第一件事是**在普通终端复现**：

```bash
# 终端 A
cd ~/UR10 && source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
ros2 launch ~/colcon_ws/src/mecheye_ros2_interface/launch/start_camera_ur10.py
# 终端 B
cd ~/UR10 && source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
python3 scripts/validate_mecheye_capture.py
```

- 若**仍超时**：先把现象、日志（`~/.ros/log/…`、SDK 输出）、复现命令写进
  `experiments/2026-09-18-相机采集链路不通.md`，**不要**跳过这步硬上标定。
- 若**正常**：把它记进实验记录，然后继续第 4 步。

## 4. 标定流程（按顺序做，每步留数值证据）

1. **布置与基准**
   - 棋盘格固定在台面（平放或倾斜都可，但需同一帧内可见、不反光、不被相机/夹爪自遮挡）。
   - 用尺量方格实长（验证打印比例：应 ≈ `6 mm`，记录实测值与量具）。
2. **采集**
   - 复用现有脚本：`python3 scripts/collect_handeye_sample.py …`（看 `--help` 取参数）。
   - 采 **≥20 个样本**，姿态**分布要散**：平移至少覆盖 3 个位置、朝向至少 3 个不同方向
     （只有平移或只有旋转会退化，手眼解会不稳）。
   - 每个样本落盘到 `outputs/handeye/samples/<id>/`：`image.png` + `sample.json`
     ——**务必同时保存原始点云/中间量**（09-17 只存摘要，导致无法离线复核，是那次最大的证据缺口）。
3. **求解**
   - `python3 scripts/solve_handeye_checkerboard.py …`：Park 解 **+ 与 Horaud（或 Tsai）交叉比对**；
     报设计矩阵条件数、训练残差、留出残差。
   - 留出集固定 **≥5 个样本**，且必须**不参与**求解。
4. **独立验收**
   - 冻结 yaml 之后**再采一组新样本**（完全不参与拟合），报平移/旋转误差；
   - 另外做**固定棋盘格闭环**：同一世界点在多个姿态下的重投影/闭环一致性；
   - 两项都要给数字，不给"看起来对"。
5. **落文档**
   - 全部门限通过 → 写 `docs/`（手眼章节）+ 同步 `docs/00` 与 `HANDOVER.md`
     （注意与并行 agent 的冲突：改前先 `git status` 看有没有别人刚改）。
   - 未通过 → 写 `experiments/`，并把 `valid: false` 与失败数值写进 yaml。
6. **收尾**：`python3 scripts/check_current_docs.py` 必须通过。

## 5. 判据（写进结论，不达标就是 `valid: false`）

| 项目 | 门限 |
|---|---|
| 留出集闭环 平移 | RMS ≤ 3 mm、最大 ≤ 5 mm |
| 留出集闭环 旋转 | RMS ≤ 1°、最大 ≤ 2° |
| 冻结后独立样本 | 平移 ≤ 5 mm、旋转 ≤ 2° |
| 交叉方法差异（Park vs Horaud/Tsai） | 平移 ≤ 1 mm、旋转 ≤ 0.5° |
| 标定板重投影 | RMS ≤ 0.5 px |

## 6. 关键坑（都是本仓库踩过的，别重犯）

1. **单可见侧面推物体中心时，偏移方向由"相机在哪一侧"决定**（凸体只露朝向相机的那个面）。
   旧脚本 `scripts/locate_cube_near_checkerboard.py` 用"远离棋盘格中心"判方向，
   在 2026-09-17 的实测里**差了 50 mm**（可见侧面在 target `x=+0.0528 m`，相机在 `x=+0.3641 m`）。
2. **尺寸先验必须实测验证**：09-17 把"50 mm 标称边长"硬编码进推算，而实测可见跨度只有
   `44.28 mm`，先验本身没被验证过。
3. **原始点云/中间量必须落盘**，只存摘要等于没存。
4. **不要用 `ros2 node list` 判断节点是否启动**：本机实测它返回空，而 `ros2 topic list`
   正常；用 `ros2 topic list` / `ros2 service list` / `ps`。
5. 相机是**单色**（PRO XS 硬件规格），别把 `bgr8` 三通道当彩图。
6. 多网卡环境（有线 + 无线 + tun）：相机 SDK 会抱怨
   "Failed to obtain the IP address of the computer Ethernet port connected to the device"，
   但会按 IP 直连成功——该警告本身不代表故障。

## 7. 顺带要做（同一轮，视觉几何闭环）

标定完成后，用本轮已经做好的新工具做**物块中心测量验收**（这才是第 6 节的实际前置）：

```bash
# 现场：相机已启动、机器人静止，同一帧同时看到棋盘格与物块
python3 scripts/measure_cube_geometry.py --capture \
  --save-cloud outputs/vision/clouds/cube-<tag>.npz \
  --output outputs/vision/cube-measurement-<tag>.json
# 之后可离线复算（不需要相机/机器人）：
python3 scripts/measure_cube_geometry.py --npz outputs/vision/clouds/cube-<tag>.npz
```

- 该工具用**顶面平面 + 最小外接矩形足印 + 参考站立面**算中心，带硬门禁（覆盖率 ≥ 0.85、
  与棋盘格面平行 ≤ 12°、长宽比 ≤ 1.10、点数下限），**不满足就拒识**，
  并且会把"实测边长 ≠ 标称 50 mm"单独报成 `measured_size_prior_mismatch`。
- 它的离线数值门禁已通过 6/6（含"掠射视角只见侧面必须拒识"）：
  `python3 scripts/test_measure_cube_geometry.py` →
  `outputs/vision/measure-cube-offline-selftest-20260918.json`。
- **验收要求**：在**≥3 个重新摆放位置**各测一次，用**独立真值**（钢板尺/卡尺实测，
  或顶面+侧面交叉解算，或落点后的接触点）比对，报告误差分布。
  **不要**再用 2026-09-17 那次"人工目测纠正点"当基准——它与任何单侧面重建都不自洽
  （残差 22.2 mm，见 P0-B 记录）。

## 8. 交付物清单

- `config/handeye_eye_in_hand_<日期>.yaml`：`valid`、方法、训练/留出样本数、各项残差、
  验收门限与结论、`validation_failures`（失败时逐条写数值）。
- `outputs/handeye/samples/<id>/`：图 + JSON + **原始点云/中间量**。
- `outputs/vision/clouds/*.npz` + `cube-measurement-*.json`（多位置验收）。
- `experiments/2026-09-<日期>-眼在手棋盘格重标定.md`：过程、数值、失败也写。
- 若相机链路不通：`experiments/2026-09-18-相机采集链路不通.md`。
- 结束时 `python3 scripts/check_current_docs.py` 通过。

先给我一份"现状核对 + 你打算怎么采（姿态数量与分布）+ 预计风险"的简短计划，
我确认后再开始采样本。

===== PROMPT 结束 =====
