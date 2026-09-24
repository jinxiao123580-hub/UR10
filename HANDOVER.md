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
  点位仅用于离线示例。当前只通过 dry-run，尚未完成本机真机抓放验收，真机执行前必须现场重新确认。
- 已完成夹爪独立开合反馈验证：空夹 `POS=3 → 22 → 103 → 191 → 230`，张开回到 `POS=3`，
  目标闭合为 `PRE=255`；完整记录在 `outputs/gripper/motion-verification.json`。
- 2026-09-18 完成一次位置抓放闭环；闭合反馈为 `POS=229 / OBJ=3`，用户现场确认实际夹住后，
  继续完成抬升、搬运、释放和离开。该证据证明位置运动链可执行，但不构成自动 `OBJ=2` 验收。
- `scripts/position_pick_place.py --demo` 是固定点课程演示模式：显式忽略 `OBJ` 并继续位置流程；
  不得把演示结果写成抓取成功，也不使用力控。重启后的完整命令在 `docs/00-当前可用操作.md`。
- 课程展示的完整三终端流程已整理到 `docs/15-位置抓放展示流程.md`。

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
python3 scripts/read_ur_state.py
python3 scripts/check_current_docs.py

# 离线分析（不需要相机/机器人）
python3 scripts/analyze_ft_mass_scale.py          # P0-A：质量尺度 vs 常量力偏置（只读传感器 HTTP + 存档数据）
python3 scripts/analyze_cube_center_error.py      # P0-B：中心误差差向量 + 相机侧符号检查
python3 scripts/test_measure_cube_geometry.py     # 视觉几何离线数值门禁（6/6）
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

### 网页监控台（2026-09-18 修复启动/停止语义）

`bash scripts/start_dashboard.sh`（**必须先 `cd ~/UR10`**，脚本用的是相对路径）
起桥 + ATI（+ 可选相机），起完会**自检 `http://127.0.0.1:8080/` 真的应答**才打印地址，
否则打印端口占用与桥日志尾部并以退出码 1 结束。停止用
`bash scripts/start_dashboard.sh --stop`：SIGTERM → 最多 3 s → SIGKILL，并**校验 8080/9090
已释放**才报成功。修复前 `ros_web_bridge.py` 会吃掉 SIGTERM/SIGINT（`rclpy.spin` 在守护线程）、
`--stop` 打印"已停止"却留着进程占端口、启动脚本只看 `pgrep` 不看端口 → 页面"打不开"。
详见 `experiments/2026-09-18-入口可用性审计.md` §3.1。

## 课程进度（机械臂工程实践 8 节）

本轮做**第 1–6 节**；第 7（转盘动态抓取）、第 8（毛笔字）延后，且不采购新硬件。
第 4 节用闭合并拢的**左夹爪指尖代替笔尖**。完整方案与逐节做法见
[`experiments/2026-09-18-八节课实施计划草案.md`](experiments/2026-09-18-八节课实施计划草案.md)。

| 课 | 状态 |
|---|---|
| 1 整体认识 | ✅ 设备清单 + 三条通道已写入 [`docs/01`](01-硬件与网络.md) §7 |
| 2 示教器运动 | ✅ `scripts/read_ur_state.py` 已真机只读实测；示教器操作见 §8 |
| 3 ROS 控制运动 | ⏳ 入口已有（`scripts/position_pick_place.py`，**仅 dry-run**）；阻塞在负载未验证 |
| 4 关节标定 | ⏳ TCP 脚本就绪未采集；100.5 mm 直线实验未做 |
| 5 机器人感知 | ⏳ P0-A（力标定 2 倍矛盾）**已诊断**：是单姿态反推忽略常量力偏置的假象，传感器/换算无误；
  修复与绝对尺度基准**按用户决定延后**；时延测量仍未做 |
| 6 视觉自动抓取 | ⏳ 几何方案已换为"测顶面"并带离线门禁（6/6 通过），但**现场未验收**；
  旧的 35.84 mm 参照本身不成立（见 P0-B） |
| 7 转盘动态抓取 | 延后（现有相机 3D 采集 0.7–1.1 s，慢的是 3D 不是 2D，见计划 §5） |
| 8 毛笔字 | 延后（依赖第 4、5 节） |

## 两个必须先解决的问题 —— 均已定位（2026-09-18 当日做完）

### P0-A · 力标定的 2 倍质量矛盾 —— **已诊断为方法学假象，修复延后**

原问题：6 次独立拟合的质量一致收敛到 **3.20–3.29 kg**，而 2026-09-10 单姿态 `|F|/g` 反推
**6.56 kg**，相差 **2.03 倍**且可重复。

**结论（有数值证据）**：单姿态反推**忽略了传感器系内约 25–36 N 的常量力偏置**，
"`|F|` 与姿态无关且等于 `m·g`"这个前提本身不成立。

- 传感器只读配置（HTTP `192.168.1.2/netftapi2.xml`）：`cfgcpf = cfgcpt = 1000000`
  ⇒ `scripts/ati_netft_node.py:49-50` 的硬编码值**等于该传感器自己的标定值**，单位无误；
  `setbias = 0;0;0;0;0;0` ⇒ 硬件 Bias 全零，"错误 Bias 导致翻倍"不成立；量程 `165/165/495/15/15/15`。
- 68 个存档静态姿态的球面拟合（`F` 必落在半径 `m·g`、球心 `b` 的球面上，**不用 FK、不用安装旋转**）：
  6 个批次独立给出 **m = 3.14–3.51 kg**、`|b| = 24.8–35.8 N`，与既有多姿态拟合一致。
- 否证纯尺度错误：若真是 2 倍尺度错，`|F|` 必须恒定；实测 `|F|` 跨 **12.34–69.58 N（5.64 倍）**，
  单姿态法给出的质量在 **1.26–7.10 kg** 间摆动。

证据：`outputs/ft_calibration/mass-scale-diagnosis-20260918.json`、
`experiments/2026-09-18-力标定2倍质量矛盾诊断P0-A.md`；复现 `python3 scripts/analyze_ft_mass_scale.py`。
**仍开放（延后）**：① ~29.6 N 常量偏置的物理来源未定（零点/线缆侧向力/安装预载/配置残留）——
它不改变尺度结论，但直接违反"无外力应读零"；② 绝对牛顿尺度仍依赖质量基准，
控制器回读 `3.5 kg` 只是旁证、按本文规定不算独立测量。

### P0-B · 第 6 节视觉中心误差 —— **假设被否证，查出 50 mm 符号错误，改为"测顶面"**

原假设：`√(25²+25²) = 35.36 mm` 与实测 35.84 mm 只差 0.48 mm ⇒ 推测"两轴各偏半边长"。

**实测差向量否证该假设**：base 差向量 `(+31.36, -17.36) mm`（模长 35.84 mm），
在棋盘格系是 `(36.22, 13.94) mm` —— 只有模长巧合，分量根本不是 25/25。

**查出的硬错误**：可见竖直侧面在 target `x = +0.0528 m`，而相机原点在 target `x = +0.3641 m`
且光轴朝 `-x` ⇒ 凸体只露朝向相机的面，体心必须在**另一侧**（`x = 0.0278 m`）；
旧 `scripts/locate_cube_near_checkerboard.py` 用"远离棋盘格中心"判方向（本场景物块是沿 **-y** 偏
~110 mm 摆放，该启发式无意义）⇒ **符号反了 50 mm**。另外"50 mm 边长先验"从未被测量验证
（可见侧面 y 实测跨度仅 44.28 mm）；与人工确认点残差在符号修正后仍有 **22.2 mm**，
且与"50 mm 立方体 + 单可见侧面"几何上不自洽 ⇒ **那个人工目测点不能当验收基准**。

**修法已实现**：`scripts/measure_cube_geometry.py`（顶面平面 + 最小外接矩形足印 + 参考站立面，
带硬门禁：覆盖率 ≥0.85、与棋盘格面平行 ≤12°、长宽比 ≤1.10、点数下限；不满足**拒识**；
实测边长 ≠ 标称时单独报 `measured_size_prior_mismatch`；支持原始点云落盘与离线复算）。
离线数值门禁 6/6 通过（含"掠射视角只见侧面**必须拒识**"）：
`python3 scripts/test_measure_cube_geometry.py`。
证据：`experiments/2026-09-18-视觉中心误差差向量诊断P0-B.md`、`outputs/vision/center-error-diagnosis-20260918.json`。
**仍未验收**：需现场采集（**必须落盘原始点云**）+ ≥3 个重新摆放位置的**独立真值**比对。

## 已排除，不要再查（省时间）

| 嫌疑 | 结论 |
|---|---|
| 点云单位 mm/m 混用（1000× 错误） | **排除**：实测 `xyz_median ≈ 0.259`，与工作距离 0.3–0.6 m 同量级 ⇒ 是米 |
| 米制尺度完全未验证 | **部分已验**：6 mm 格长即独立长度基准，"逐角点 RMS 0.692 mm"已把尺度误差约束到 ~1.4%；缺的是全视场量程的独立基准 |
| 用了非官方 ROS 驱动 | **排除**：官方英文文档话题表漏了 `color_image` 是**笔误**；中文页与源码均确认该话题是官方的 |
| `base` 与 `base_link` 相差 1.182 m 是平移 | **改为**：UR 官方《Robot Frames》明确两者**同位置、仅绕 Z 轴转 180°**；180° yaw 使半径 r 处点位移 **2r**，r≈0.591 m ⇒ 正好 1.182 m。现场 `tf2_echo base base_link` 可确认 |

## 下一步（按顺序）

1. **P0-A 已诊断、按用户决定延后**：2 倍矛盾是单姿态反推忽略 ~29.6 N 常量力偏置的假象，
   尺度无需修正。延后的是"偏置的物理来源"与"绝对牛顿尺度的独立质量基准"。
2. **P0-B 已定位并实现修法**：下一步是**现场验收**——同一帧看到棋盘格与物块采集并落盘原始点云，
   在 ≥3 个重新摆放位置用独立真值比对（`scripts/measure_cube_geometry.py --capture --save-cloud`）。
3. **时延测量**：全库 grep `延迟|时延|latency|时间同步|时间戳对齐` **零命中**，
   而第 5 节明确要求"特别注意数据时间上的延迟"。
4. 第 4 节：TCP 标定 → 100.5 mm 直线实验
   （⚠️ 纯平移的 `movel` 下，TCP 定义误差**不改变**笔尖位移量 —— 该实验检验的是**运动学精度**，不是 TCP 定义）
5. 第 5 节：重力补偿重新建模 + 独立验收。⚠️ **原计划的门禁要改**：
   "拟合质量须与单姿态 `|F|/g` 一致（<5%）"**数学上无效**（单姿态法忽略了偏置，见 P0-A），
   应改为"未参与拟合的多姿态残差 + 姿态方向覆盖（可辨识性）"双门禁。
6. 第 6 节：P0-B 的几何修法 → 多位置中心误差验收（独立真值）→ 才可接自动抓取
7. **相机服务待定性**：2026-09-18 实测 `/device_info`、`/capture_color_image` 与
   `scripts/validate_mecheye_capture.py` 均无响应（40–60 s 超时，重启相机节点无效），
   相机硬件与 ROS 服务机制本身正常；需在**普通终端**复核一次再定性，见
   `experiments/2026-09-18-入口可用性审计.md` §3.2。

所有新的运动方案遵循：资料依据 → 离线数值门禁 → 官方假硬件 → 单段低速真机 → 完整流程。
每一层保存数值证据。

## 资料与存档规则

- 所有结论必须有数值证据；失败实验写 `experiments/`，原始 CSV/JSON 写 `outputs/`。
- 只有独立验证通过的稳定结论才能写入 `docs/`。
- 更新脚本、接口、路径或有效操作方式时，同一提交中更新 `docs/00-当前可用操作.md`、本文件和
  相关专题文档，并运行 `python3 scripts/check_current_docs.py`。
- 新方案实施前先查官方文档、论文或原始实现，记录来源、版本、适用条件及本机差异。
