# 交接文档 · UR10 视觉力觉抓取系统

> **本文件是交接的唯一入口。** 接手者请按顺序读完本文件 → 再看 `docs/` 里的细化文档。
> 所有结论均标注了**验证方式**；凡未实测的，一律写明"未验证"。
> 最后更新：2026-09-10 · 当前状态以 `main` 分支最新提交为准

---

## 0. 一分钟上手

```bash
# 1) 体检（先跑这个，九成问题能定位）
python3 ~/UR10/scripts/diagnose.py

# 2) 起网页监控台（力数据 + 相机画面）
bash ~/UR10/scripts/start_dashboard.sh          # 浏览器打开 http://127.0.0.1:8080/
bash ~/UR10/scripts/start_dashboard.sh --stop   # 全停

# 3) 跑一轮抓放（需要 ur_command_node 在跑）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/UR10/scripts/ur_pick_place_full.py 1
```

仓库：<https://github.com/jinxiao123580-hub/UR10>（本地 `~/UR10`，分支 `main`，SSH 走 443 已在 `~/.ssh/config` 配好）

---

## 1. 系统全貌

### 1.1 硬件与网络（全部实测确认）

```
                    192.168.1.0/24  同一个二层网络（网线直连/交换机）
  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌────────────────┐
  │ 电脑          │  │ UR10 机器人   │  │ ATI Net F/T  │  │ Mech-Eye 相机   │
  │ 192.168.1.10 │  │ 192.168.1.3  │  │ 192.168.1.2  │  │ 192.168.1.33   │
  │ ROS 2 Humble │  │ CB3 PolyScope│  │ 六维力/力矩   │  │ 3D 相机         │
  │              │  │ 3.15.8       │  │              │  │                │
  └──────────────┘  └──────────────┘  └──────────────┘  └────────────────┘
        enp0s31f6（主板网口，静态 IP）      TCP 80 + UDP 49152   TCP 5577 + UDP 3956
```

| 设备 | 型号/标识 | 关键参数 | 接入方式 |
|---|---|---|---|
| 机械臂 | **UR10**（CB3 代） | SN `2022300602`，URSoftware **3.15.8.106339**，主机名 `ur-2022300602` | Dashboard 29999 / 状态 30001 / URScript 30002 |
| 末端夹爪 | **Robotiq 2F 二指** | 经 URCap 1.8.13.22852 的 `drivergripper` 守护进程 | 电脑直连 **TCP 63352** |
| 六维力 | **ATI Net F/T** | SN `1106WNT072`，固件 2.0.12(2009-07-08)，标定 `SI-165-15`（165N/495N/15N·m），MAC `00:16:BD:00:08:2F` | HTTP :80 管理页 + **RDT UDP 49152** |
| 3D 相机 | **Mech-Eye PRO XS**（梅卡曼德） | 版本 **2.5.0**，SN `RAM35238A3020004`，MAC `24:c5:d3:55:00:00` | **GigE Vision**（GVCP UDP 3956 / TCP 5577） |
| 电脑 | Ubuntu 22.04 + ROS 2 Humble | 网口 `enp0s31f6` = `192.168.1.10/24`；另有 USB 网卡 `10.115.3.x` 上网（跑 sing-box 代理） | — |

**机器人能力**：`get robot model` → UR10；`robotmode` → RUNNING；`safetystatus` → NORMAL。
**机器人 root SSH**：`ssh root@192.168.1.3`，密码 `easybot`（出厂默认，未改）。

### 1.2 两条核心通道（本项目的立身之本）

```
        ┌────────────────────────── 电脑 ──────────────────────────┐
        │  ur_arm.py ──ROS话题 /ur_link/urscript──▶ 30002 ──▶ 关节运动
        │  rq_gripper.py ─────────TCP 63352──────────────▶ 夹爪开合
        │  ati_netft_node.py ─────UDP 49152(RDT)─────────▶ 六维力
        │  ros_web_bridge.py ─────WebSocket 9090─────────▶ 网页看板
        └───────────────────────────────────────────────────────────┘
                       示教器全程不参与（这是关键结论）
```

---

## 2. 当前状态：什么能用、什么待办

### ✅ 已完成并**真机验证**

| # | 能力 | 验证方式与结果 |
|---|---|---|
| 1 | **ROS 控制机械臂运动** | `scripts/ur_arm.py nudge 0 0 -0.003` → 实测位移 **3.1mm** ✔ |
| 2 | **电脑直控夹爪开合** | `scripts/rq_gripper.py test` → POS **3 ↔ 230** 实际开合 ✔ |
| 3 | **完整抓放闭环（抓→搬→放）** | `ur_pick_place_full.py 1` → 夹爪 `POS=104 OBJ=2`（夹住），物体成功搬运 ✔ |
| 4 | **抓放三种方向模式** | 单向 / `--swap`（搬回）/ `--pingpong`（往返）均实测 ✔ |
| 5 | **六维力数据接入 ROS 2** | `/ft_sensor/wrench` 稳定 **200Hz**，实测 14 秒 2796 帧 ✔ |
| 6 | **正运动学 TF 链** | `scripts/check_fk.py` → TF `base→tool0` 与真机 TCP 偏差 **2.5mm**、姿态 **0.0033rad** ✔ |
| 7 | **网页监控台** | HTTP 200 + WS 服务调用闭环；真实彩色图 **1280×1024** JPEG、点云统计、力 200Hz 与 10 秒滚动曲线 ✔ |
| 8 | **Mech-Eye 真机出图** | SDK **2.5.0** + ROS 2 接口编译通过；彩色/深度/点云服务均 `error_code=0` ✔ |

### ⏳ 待办（按优先级，详见 §6）

| 优先级 | 任务 | 阻塞点 |
|---|---|---|
| **P1** | 手眼标定（相机坐标→机器人坐标） | P0 已完成，可开始 |
| P2 | 视觉引导抓取（点云→目标位姿→抓取） | 依赖 P1 |
| P2 | 力控应用（碰撞检测 / 力引导插装） | 无阻塞，数据已通 |
| P3 | 监控台扩展（点云 3D 预览 / 关节角面板 / CSV 记录） | 无阻塞 |

### 🔧 环境现状（进程状态会变，**接手时先自己查一遍**）

> ⚠️ 本节描述的是"**怎么查、怎么起**"，不是"此刻一定在跑"。
> 长时间运行的进程可能已被终止（本机实测：监控台三件套曾被外部 SIGTERM 终止，端口释放）。
> **开工第一步用下面的命令确认现场，不要相信任何"应该还在跑"。**

```bash
# 一眼看全（端口没占用就说明没在跑）
ss -tln | grep -E ':8080|:9090|:30001|:63352'
ps -eo pid,etime,cmd | grep -E '[u]r_state_node|[u]r_command_node|[r]os_web_bridge|[a]ti_netft'
python3 ~/UR10/scripts/diagnose.py         # 硬件侧体检（不依赖这些进程）
```

| 进程 | 作用 | 依赖关系 |
|---|---|---|
| `ur_state_node` | 发 `/joint_states` | **TF 链依赖它**（`start_ur_tf.sh` 要先起它） |
| `ur_command_node` | 收 `/ur_link/urscript` 发 30002 | **任何机械臂运动命令都需要它在跑** |
| `ros_web_bridge.py` + `ati_netft_node.py` | 网页监控台（力数据） | 见 `docs/10` |
| `mecheye_ros_interface` | Mech-Eye 真相机服务与话题 | SDK 2.5.0；由监控台脚本自动启动 |
| `fake_mecheye_publisher.py` | 无相机时的页面模拟器 | 与真相机话题同名，**绝不能同时开**；启动脚本发现 SDK 后会主动停止它 |

**启动/停止**：

```bash
# 监控台三件套（力 + 相机 + 桥 + 网页）
bash ~/UR10/scripts/start_dashboard.sh
bash ~/UR10/scripts/start_dashboard.sh --stop

# 机械臂驱动（两个都要，忘了 command_node 就会"发脚本没反应"）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ur_link ur_state_node   --ros-args -p robot_ip:=192.168.1.3 &
ros2 run ur_link ur_command_node --ros-args -p robot_ip:=192.168.1.3 &
```

> 💡 **一条经验**：起 ROS 节点时**别在多处重复启动同一个节点**。
> 本机踩过的坑：两个 `ati_netft_node` 实例会互相抢 ATI 的 RDT 流（数据每 5 秒一跳，见铁律 4）。

---

## 3. 代码仓库地图

```
~/UR10/  （= GitHub jinxiao123580-hub/UR10）
├── README.md                    总览 + 快速开始 + 五个必知坑
├── HANDOVER.md                  ← 本文件
├── CODEX_PROMPT.md              给 AI 接手者的提示词（可直接粘贴）
├── docs/
│   ├── 01-硬件与网络.md          拓扑/静态IP/绕代理/端口全表/上电流程
│   ├── 02-ROS2驱动-ur_link.md    话题与服务清单、构建、离线自测
│   ├── 03-机械臂控制.md          「def 必须显式调用」坑、30001 位姿解析
│   ├── 04-夹爪控制.md            ★ 63352 架构、rq_activate() 死锁真因、ASCII 协议全表
│   ├── 05-完整抓放.md            三种方向模式、日志解读、参数调优
│   ├── 06-排查手册.md            症状速查表 + 六个坑完整复盘
│   ├── 07-机器人内部访问.md      root SSH、.urp 是 gzip+XML、程序读写
│   ├── 08-六维力与3D相机.md      ★ RDT 协议实测、GVCP 发现、base/base_link 坑、手眼标定
│   ├── 09-MechEye相机接入清单.md 相机一步步接入（含 SD卡 安装、四个官方坑）
│   └── 10-网页监控台.md          ★ 桥协议、实测数据、排查
├── scripts/
│   ├── diagnose.py               ★ 一键体检（网络/端口/夹爪/机械臂）
│   ├── rq_gripper.py             ★ 夹爪直控（可当库：open/close/status/object_detected）
│   ├── ur_arm.py                 ★ 机械臂控制（见 §5 铁律 1）（含姿态转换工具）
│   ├── ur_pick_place_full.py     ★ 完整抓放（单向/--swap/--pingpong）
│   ├── ati_netft_node.py         ★ ATI 力传感器 → /ft_sensor/wrench（含自愈）
│   ├── ros_web_bridge.py         ★ ROS→WebSocket 桥（网页数据源）
│   ├── launch_dashboard…(start_dashboard.sh) ★ 一键起监控台
│   ├── setup_mecheye.py          相机接入助手（自动打补丁+编译）
│   ├── fake_mecheye_publisher.py 模拟相机（无 SDK 也能验证页面）
│   ├── start_ur_tf.sh / check_fk.py   TF 链 + FK 自检
│   ├── ur_capture_pose.py        标定抓取点/放置点 → poses.json
│   ├── ur_rel_move.py / ur_circle.py  相对移动 / 画圆
│   ├── network_setup.sh / setup_pc_ip.sh  配网（静态IP + 绕代理）
│   └── fake_ur_server.py         离线模拟 UR 服务器
├── web_dashboard/                index.html + app.js + style.css（ECharts 折线 + canvas 画面）
├── ros2_ws/src/ur_link/          ROS2 驱动包（机器人侧零安装）
├── legacy/                       已废弃的示教器 socket 方案（含"为什么放弃"）
└── poses.json                    抓取点/放置点标定值
```

**⚠️ 另有一个外部依赖目录**（不在本仓库）：

```
~/colcon_ws/src/mecheye_ros2_interface/   梅卡曼德官方 ROS 2 接口（已克隆，已被脚本打补丁，未编译）
~/ros2_ws/                                ur_link 的编译工作区（已 build，install/ 里是产物）
```

---

## 4. 五条铁律（血泪教训，违反必踩坑）

### 铁律 1 · 30002 发的 `def` **必须显式调用**，否则机器人纹丝不动且不报错

```python
"def f():\n  movel(...)\nend"          # ❌ 只定义，不执行，且无任何报错
"def f():\n  movel(...)\nend\nf()"     # ✅
```
`scripts/ur_arm.py::send_script(call=True)` 已自动补调用，**用它就别手写**。
验证手法：`ur_arm.py nudge 0 0 -0.003` 会打印"位移 X 米 ✔动了/✘没动"。

### 铁律 2 · **绝不要调 `rq_activate()`**（会死锁）

它的等待循环是 `while(ACT!=0 or STA!=0)`，而夹爪正常态 `ACT=1` → **永远出不去**。
症状：socket 连上了、命令没回执、连接也不断开（整个程序卡死）。
夹爪平时就是激活的；真要激活直接发 ASCII 命令 `SET ACT 1`。

### 铁律 3 · 视觉目标必须用 **`base`** 系，不是 `base_link`

URDF 里 `base_link` 相对 `base` **绕 Z 转 180°**（X/Y 镜像）。
实测：用错坐标系偏差 **1.29 米**（跑到反方向去抓）。
另外 UR 的 `movel(p[x,y,z,rx,ry,rz])` 里 `rx,ry,rz` 是**轴角(rotation vector)**，
**不是 RPY**——`ur_arm.py` 里有 `quat_to_rotvec()` 等转换工具（往返误差 4.4e-16）。
验证：`python3 scripts/check_fk.py`。

### 铁律 4 · **ATI RDT 是单目的地单播**——只允许一个消费者

它只把数据推给"最后一个发 start 的客户端"。两个进程同时读会**互相抢流**，
表现为**数据每 5 秒一跳**（不是传感器坏）。
`ati_netft_node.py` 已内置自愈（超时自动重申请），但正确做法是**只留一个消费者**。
注意：跑 `ati_netft_node.py --check` 会**抢走**正在运行节点的流。

### 铁律 5 · 改任何文件**先备份**；外部源码改动留补丁文件

- 本仓库的改动靠 git 兜底（每次改动都提交）
- **外部目录**（如 `~/colcon_ws/src/mecheye_ros2_interface/`）必须单独备份：
  已有备份在 `~/ur_learn/generated/backups/`（含 `.orig` 原版、`.patched` 改后版、`.patch` 差异）
- 还原命令：
  ```bash
  cd ~/colcon_ws/src/mecheye_ros2_interface && git checkout -- src/MechMindCamera.cpp include/MechMindCamera.h
  # 或 cp ~/ur_learn/generated/backups/MechMindCamera.cpp.orig src/MechMindCamera.cpp
  ```

---

## 5. 各子系统操作手册（速查）

### 5.1 机械臂（ROS → 30002）

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 scripts/ur_arm.py check                 # 读当前 TCP 位姿
python3 scripts/ur_arm.py nudge 0 0 -0.003      # 相对微动 3mm（验证通路）
python3 scripts/ur_circle.py 0.02               # 末端画圆
ros2 service call /ur_link/dashboard/stop std_srvs/srv/Trigger   # 停
```
库用法：`from ur_arm import Arm; Arm().movel([x,y,z,rx,ry,rz], v=0.05)`

### 5.2 夹爪（TCP 63352，Robotiq ASCII）

```bash
python3 scripts/rq_gripper.py status    # ACT/STA/FLT/POS/OBJ
python3 scripts/rq_gripper.py test      # 开合自检（第一次必跑）
python3 scripts/rq_gripper.py open|close
```
关键协议速查：

| 目的 | 命令 | 说明 |
|---|---|---|
| 开/合 | `SET POS 0` / `SET POS 255` | 0=全开，255=全闭 |
| **必须补** | `SET GTO 1` | 设完 POS 不发这个**不动** |
| 有无夹到 | `GET OBJ` | **2=夹住了**，3=抓空，0=运动中 |
| 位置 | `GET POS` | 停在 104/230 说明**夹住东西了**（正常） |
| 复位 | `SET ACT 0` 再 `SET ACT 1` | 有故障码 FLT≠0 时 |

⚠️ 响应是**补零/带括号格式**（`FLT 00`、`SID [9]`），别直接 `int()`，用正则抠数字。

### 5.3 六维力（UDP 49152 RDT）

```bash
python3 scripts/ati_netft_node.py --check --hz 10     # 命令行看数（会抢流，注意）
python3 scripts/ati_netft_node.py                     # ROS 节点 → /ft_sensor/wrench
ros2 service call /ft_sensor/tare std_srvs/srv/Trigger  # 软件去皮
```

**RDT 协议（实测钉死）**：
- 启动：发 8 字节 `12 34 00 <采样数> 00 00 00 00`；停止：采样数字节改 `00`
- **采样数只能 1（200Hz）或 2（822Hz）**；`10`/`255` **收不到任何数据**
- 数据包 **36 字节 = 9 个大端 int32**：
  `[0]rdt_seq [1]ft_seq [2]status [3..8]Fx Fy Fz Tx Ty Tz`
- 物理量 = `counts / 1000000`（本机 counts_per_force = counts_per_torque = 1e6）
- **status 非 0 = 某轴饱和**；`FLT` 类故障见 HTTP 页

**⚠️ 传感器从未置零**：`manuf.htm` 显示 Bias 全 0，因此当前读数含**工装自重**
（实测 `Fz≈62N`）。挂好工具后必须 tare。
**活动配置是 `#16 End of line test`**（出厂测试用），`#1` 名为 `KUKA_FTCtrl_!DoNotChange!`
（说明这盒子来自 KUKA 系统），正式用前建议在 `config.htm` 建自己的配置。

### 5.4 相机（Mech-Eye PRO XS，**已真机出图**）

现状：SDK **2.5.0** 已安装，ROS 2 接口已编译并按 IP 直连真机。

```bash
python3 scripts/setup_mecheye.py --check     # 预检：自动发现相机 + 查 SDK/依赖
```

实测发现结果（GVCP 广播，`UDP 3956`）：
```
192.168.1.33  Mech-Mind Robotics  Mech-Eye PRO XS  (固件 2.5.0, SN RAM35238A3020004)
```

**已就绪**：依赖全齐（opencv/cv-bridge/pcl/pcl-conversions/colcon）、
官方接口位于 `~/colcon_ws/src/mecheye_ros2_interface/`，补丁、专用 launch 与编译均完成。

**重装步骤（需人工，sudo 要密码）**：
1. <https://downloads.mech-mind.com.cn/?tab=tab-sdk> 注册下载 `Mech-Eye_API_2.5.0_amd64.zip`
2. `sudo apt-get install libarchive-zip-perl && crc32 <zip>` 校验 → `unzip` → `sudo dpkg -i *.deb`
3. `python3 scripts/setup_mecheye.py`（自动编译 + 生成 launch + 验证）

**启动**：`ros2 launch ~/colcon_ws/src/mecheye_ros2_interface/launch/start_camera_ur10.py`
话题：`/mechmind/point_cloud`、`/mechmind/depth_map`(32FC1 米)、`/mechmind/color_image`(bgr8)。
**⚠️ 工业相机是"服务触发式采集"**：`ros2 service call /capture_point_cloud ...` 才有数据，不是连续推流。

### 5.5 网页监控台

```bash
bash scripts/start_dashboard.sh          # 一键起 → http://127.0.0.1:8080/
```
- 力数据 **200Hz** 折线（实测）+ Mech-Eye 真实彩色/深度画面
- 真相机彩色画面 **1280×1024**；页面已按现场安装方向做上下翻转
- 力曲线为固定 **10 秒**滚动窗口，横轴随最新数据持续前移
- 协议与排查见 [`docs/10-网页监控台.md`](docs/10-网页监控台.md)
- 桥是自研的（**没用 rosbridge_suite，它要 sudo 装**）：`websockets` 库 + rclpy

---

## 6. 下一步待办（含具体执行方案）

### P0 · 装 Mech-Eye SDK 让相机出图（**已完成，2026-09-10**）

验收证据：彩色、深度、点云服务均返回 `error_code=0`；ROS 实收彩色图宽度 1280，
点云 `frame_id=mechmind_camera/point_cloud`；WebSocket 实收 1280×1024 JPEG，假相机进程已停止。

### P1 · 手眼标定（P0 完成后）

- 推荐 **eye-to-hand**（相机固定在工作台外）：`base→相机` 是常量，换夹爪不用重标
- 工具：`easy_handeye2`（**不在 apt**，需源码编译）。⚠️ 实测其"自动移动机器人"在 master 上
  被 `if False:` 短路禁用，且依赖的 `moveit_commander` 在 MoveIt 2 Humble 中不存在 →
  **只能用 `freehand_robot_movement:=true` 手动摆位取点**（正好适配本机无 MoveIt 的 CB3）
- 采样要求：**15~25 个姿态，姿态多样性比数量重要**（绕每根轴、双向、尽量接近 90°）
- ⚠️ 相机 frame 必须是**光学系**（Z 前/X 右/Y 下），误填 `camera_link` 会**平移对、姿态全歪**
- 验证顺序：重投影误差 → **实抓端到端**（最终判据）
- 细节见 [`docs/08` §3.5](docs/08-六维力与3D相机.md)

### P2 · 视觉引导抓取（P1 完成后）

数据流：`点云 → 分割/位姿估计 → tf2 变换到 base → 加夹爪偏置+安全高度 → Arm.movel() → 夹爪闭合 → OBJ 校验`
把 `ur_pick_place_full.py` 的 `poses.json` 换成实时算出的位姿即可，**其余不用改**。

### P2 · 力控应用（无阻塞，数据已通）

| 用途 | 做法 |
|---|---|
| 碰撞检测 | 监测 `/ft_sensor/wrench` 幅值超阈值即停（`movel` 前先 tare） |
| 力引导插装 | 监测特定方向力 + 搜索策略；UR 侧也可用内置 `force_mode()`（URScipt 自带，**不需要外部传感器**） |
| 抓取成功确认 | 抬起瞬间看 Fz 是否等于物体重量（比 `OBJ` 更定量） |

### P3 · 监控台扩展

点云 3D 预览（three.js）、关节角/末端位姿面板、力数据存 CSV、历史回看（ECharts dataZoom）。

---

## 7. 验证清单（每个子系统一条命令）

| 子系统 | 命令 | 期望 |
|---|---|---|
| 全部 | `python3 scripts/diagnose.py` | 全绿，结论"可以干活" |
| 力传感器 | `python3 scripts/ati_netft_node.py --check --hz 5 --count 4` | 打印 4 行六轴数据 + "正常" |
| 夹爪 | `python3 scripts/rq_gripper.py test` | POS 3→230→3，实际开合 |
| 机械臂通路 | `python3 scripts/ur_arm.py nudge 0 0 -0.003` | "位移 0.0031m ✔ 动了" |
| TF/FK | `bash scripts/start_ur_tf.sh ur10` + `python3 scripts/check_fk.py` | base 偏差 <10mm，并提示 base_link 镜像坑 |
| 相机（发现） | `python3 scripts/setup_mecheye.py --check` | 报出 PRO XS / 2.5.0 / 序列号 |
| 网页 | `bash scripts/start_dashboard.sh` 后 `curl -sI http://127.0.0.1:8080/` | HTTP 200 |

---

## 8. 已知限制与风险

| 项 | 说明 |
|---|---|
| 手眼标定未做 | 点云还在相机坐标系，**不能直接用于抓取**（P1） |
| 力传感器未置零 | 读数含工装自重（Fz≈62N），**用前必须 tare** |
| 配置是出厂测试项 | ATI 活动配置 `#16 End of line test`，建议建自己的 |
| 机器人 root 密码是默认值 | `root/easybot`，**这是台对内网敞开的设备**，注意网络安全 |
| `base_link` 镜像坑 | 任何坐标系变换的逻辑都必须走 `base`，见铁律 3 |
| 相机 TF 不能用官方 launch 的假变换 | 官方 `start_camera.py` 发布 `map→相机` 的**凭空捏造**固定变换（0,0,1），**比没有更危险**；我们的 launch 故意不发 |
| 网页无认证 | 桥监听 `0.0.0.0`，同网段可直接访问，跨网段需自行加认证 |

---

## 9. 参考资料

| 内容 | 链接 |
|---|---|
| URScript / 客户端接口 | `~/ur_learn/urscript_cn.txt`、`~/ur_learn/ur3_manual.txt`（本地手册） |
| Mech-Eye ROS 2 接口 | <https://github.com/MechMindRobotics/mecheye_ros2_interface> |
| Mech-Eye SDK 下载（国内镜像） | <https://downloads.mech-mind.com.cn/?tab=tab-sdk> |
| Mech-Eye 官方文档（ROS 2） | <https://docs.mech-mind.net/zh/eye-3d-camera/2.5.0/api/ros2.html> |
| easy_handeye2（手眼标定） | <https://github.com/marcoesposito1988/easy_handeye2> |
| ATI Net F/T 手册 | ATI 官网（HTTP 管理页里有链） |
| 第三方相机/标定调研报告 | `/home/jx/ros2-humble-3d-camera-ur10-report.md`（1410 行，含大量实测否证） |

---

## 10. 接手建议（工作方式）

这套系统里**所有的坑，本质都是"以为通了实际没通"**。因此建议：

1. **每一步都要客观证据**（位姿数值、POS/OBJ 数值、端口状态、HTTP 码），不要靠"看起来在跑"判断
2. **先问硬件本体**：能直连设备就直接问（夹爪 63352、力传感器 HTTP 页/RDT、相机 GVCP），
   比翻文档和猜快得多——本项目的两个关键突破（夹爪直控、相机型号）都是这么来的
3. **协议文档不在手边时就穷举**：ATI 的启动命令试 `0x01`/`0x02` 全失败，实际是 `0x00`+采样数
4. **改文件先备份**（铁律 5）
5. **机械臂运动时人手必须放在急停上**；先空跑验证轨迹，再放物体
