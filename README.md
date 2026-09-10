# UR10 机械臂 + Robotiq 夹爪 —— 电脑 ROS2 全流程控制教程

用一台普通 Linux 电脑，通过 **ROS 2** 控制 UR10 机械臂运动、**直连**控制 Robotiq 二指夹爪开合，
最终跑通"抓取 → 搬运 → 放置"完整闭环。

**不需要**：示教器里跑程序、URCap 编程、socket 转发服务、MoveIt、官方 ur_robot_driver。
**只需要**：一根网线 + 电脑上的 ROS 2 + 本仓库的脚本。

> 📌 **接手这个项目？先读 [`HANDOVER.md`](HANDOVER.md)**（交接主文档：设备参数、当前状态、
> 五条铁律、待办清单、验证命令）。

---

## 0. 实测环境（本文档所有结论均在此环境验证）

| 项目 | 值 |
|---|---|
| 机器人 | **UR10**（CB3 代，PolyScope **3.15.8.106339**，Debian 7） |
| 机器人序列号 | 2022300602（主机名 `ur-2022300602`） |
| 机器人 IP | `192.168.1.3` |
| 电脑 | Ubuntu + **ROS 2 Humble** |
| 电脑网口 | `enp0s31f6`，静态 IP **`192.168.1.10/24`** |
| 末端夹爪 | **Robotiq 2F 二指夹爪**（经 URCap 1.8.13.22852 的守护进程驱动） |

> 结论对 UR3 / UR5 / UR10（CB3 与 e-Series）通用：URScipt 语法与三个原生 TCP 接口是同一套。
> 需要注意的差别见 [`docs/01-硬件与网络.md`](docs/01-硬件与网络.md) 的接口表。

---

## 1. 核心：两条独立通道（本项目的全部秘密）

整件事最关键的认知是——**机械臂和夹爪走两条完全不同的路，互不干扰**：

```
        ┌─────────────────────────── 电脑 ───────────────────────────┐
        │                                                            │
        │  ur_arm.py ──ROS话题──▶ ur_command_node ──30002──▶ [URControl] ──▶ 关节运动
        │                                                            │
        │  rq_gripper.py ─────────TCP 63352──────────────▶ [drivergripper] ──▶ 夹爪开合
        │                                                            │
        └────────────────────────────────────────────────────────────┘
                       同一个机器人控制器，两条互不干扰的通道
```

| 通道 | 电脑怎么发 | 机器人侧谁收 | 干什么 |
|---|---|---|---|
| **机械臂** | ROS 话题 `/ur_link/urscript`（std_msgs/String） | `URControl` 端口 **30002** | 执行 URScript：`movel` / `movej` → 机械臂运动 |
| **夹爪** | 裸 TCP 连 **`192.168.1.3:63352`** | `drivergripper`（Robotiq URCap 守护进程） | Robotiq ASCII 协议：`SET POS` → 夹爪开合 |

这样一来：**示教器全程不用碰**，也不用担心 URCap 往程序里塞代码。

---

## 2. 快速开始（5 步跑通抓放）

### 第 1 步 · 网络

```bash
# 网线插到控制柜 Network 口和电脑主板网口，然后：
sudo bash scripts/network_setup.sh          # 配静态 IP + 绕过代理隧道
python3 scripts/diagnose.py                 # 一键体检（推荐，先跑这个）
```

期望看到：`ping OK`、`29999/30001/30002 开`、`63352 开`、`夹爪 ACT=1`、`机械臂位姿 ...`。

### 第 2 步 · 机器人上电

示教器上：开机 → 解除急停 → **ON（上电）** → **START（松刹车）**。
（也可以用 dashboard 命令，见 [`docs/01-硬件与网络.md`](docs/01-硬件与网络.md)。）

### 第 3 步 · 起 ROS 驱动

```bash
cd ~/ros2_ws
colcon build --packages-select ur_link      # 首次
source install/setup.bash
ros2 run ur_link ur_command_node --ros-args -p robot_ip:=192.168.1.3
```

### 第 4 步 · (可选) 标定抓取点/放置点

手推机械臂（或示教器点动）到抓取点，然后：

```bash
python3 scripts/ur_capture_pose.py pick     # 记录抓取点位姿
python3 scripts/ur_capture_pose.py place    # 记录放置点位姿
```

生成 `~/ur_learn/poses.json`。

### 第 5 步 · 跑抓放

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

python3 scripts/ur_pick_place_full.py 1               # 1 轮：起点 → 终点
python3 scripts/ur_pick_place_full.py 1 --swap        # 反向：终点 → 起点（把物体抓回来）
python3 scripts/ur_pick_place_full.py 4 --pingpong    # 往返 4 轮，物体来回搬
```

---

## 3. 目录导航

| 文档 | 内容 |
|---|---|
| [`docs/01-硬件与网络.md`](docs/01-硬件与网络.md) | 网络拓扑、静态 IP、绕代理、机器人接口/端口全表、上电流程 |
| [`docs/02-ROS2驱动-ur_link.md`](docs/02-ROS2驱动-ur_link.md) | ROS2 驱动包 ur_link：话题/服务清单、构建、离线仿真自测 |
| [`docs/03-机械臂控制.md`](docs/03-机械臂控制.md) | 发 URScript 让臂动、**`def` 必须显式调用**这个坑、位姿读取、画圆/相对移动 |
| [`docs/04-夹爪控制.md`](docs/04-夹爪控制.md) | **本项目最有价值的部分**：63352 守护进程、Robotiq ASCII 协议、`rq_activate()` 死锁陷阱 |
| [`docs/05-完整抓放.md`](docs/05-完整抓放.md) | 抓放脚本用法、三种方向模式、参数调优、抓空检测 |
| [`docs/06-排查手册.md`](docs/06-排查手册.md) | 症状 → 原因 → 解决 速查表 + 完整踩坑记录 |
| [`docs/07-机器人内部访问.md`](docs/07-机器人内部访问.md) | root SSH 进机器人、`.urp` 是 gzip+XML、直接读写程序文件 |
| [`docs/08-六维力与3D相机.md`](docs/08-六维力与3D相机.md) | **ATI Net F/T 力传感器 + Mech-Eye 3D 相机** 接入（RDT 协议实测、GVCP 发现、官方 ROS 2 接口、坐标系统一） |
| [`docs/09-MechEye相机接入清单.md`](docs/09-MechEye相机接入清单.md) | 梅卡曼德相机**一步步接入清单**（SDK 安装、自动打补丁、四个官方坑） |
| [`docs/10-网页监控台.md`](docs/10-网页监控台.md) | **网页看相机画面 + 六轴力折线图**（自研 ROS→WebSocket 桥、协议、实测数据） |

| 脚本 | 作用 |
|---|---|
| `scripts/diagnose.py` | **一键体检**：网络/端口/夹爪/机械臂全查 |
| `scripts/start_dashboard.sh` | **一键起网页监控台**（力+相机+桥+浏览器） |
| `scripts/ros_web_bridge.py` | ROS 话题 → WebSocket 桥（网页数据源） |
| `scripts/ati_netft_node.py` | **ATI Net F/T 力传感器 → ROS 2**（`/ft_sensor/wrench`，带 `--check` 命令行看数） |
| `scripts/setup_mecheye.py` | 梅卡曼德相机接入助手（自动发现 + 打补丁 + 编译 + 验证） |
| `scripts/fake_mecheye_publisher.py` | 模拟相机（没 SDK 也能验证网页画面链路） |
| `scripts/start_ur_tf.sh` | 起 UR10 的 TF 链（`base → tool0`），视觉抓取的地基 |
| `scripts/check_fk.py` | **FK 自检**：比对 TF 与真机 TCP，并检查 `base`/`base_link` 镜像坑 |
| `scripts/rq_gripper.py` | 夹爪直控模块（`status`/`open`/`close`/`test`），可当库用 |
| `scripts/ur_arm.py` | 机械臂控制模块（`check`/`nudge`），可当库用 |
| `scripts/ur_pick_place_full.py` | **完整抓放**（三种方向模式） |
| `scripts/ur_capture_pose.py` | 记录当前位姿到 poses.json |
| `scripts/ur_rel_move.py` | 相对移动（走 ROS） |
| `scripts/ur_circle.py` | 末端画圆（走 ROS） |
| `scripts/network_setup.sh` | 一键配网（静态 IP + 策略路由绕代理） |
| `scripts/fake_ur_server.py` | 模拟 UR 服务器，没机器人也能调代码 |
| `ros2_ws/src/ur_link/` | ROS2 驱动包（最小 UR 桥，机器人侧零安装） |
| `legacy/` | 已废弃的示教器 socket 方案（留档，说明为什么放弃） |

---

## 4. 五个必知的坑（全部踩过并已解决）

1. **`rq_activate()` 会死锁** —— 它的等待循环是 `while(ACT!=0 or STA!=0)`，
   而夹爪正常状态 `ACT=1`，于是**永远转不出去**。示教器程序"连上但永不回话"就是这个原因。
   → 别调它，夹爪默认已激活（`ACT=1`）。详见 [`docs/04`](docs/04-夹爪控制.md)。
2. **30002 发的 `def xxx(): ... end` 必须显式再调用 `xxx()`** —— 光定义机器人**不动**，
   而且**不报错**，非常隐蔽。详见 [`docs/03`](docs/03-机械臂控制.md)。
3. **夹爪不需要经示教器转发** —— 电脑能直连 `63352`。之前折腾的 30010 socket 方案是弯路。
4. **示教器 Script 节点可能是 `type="File"` 且带过期 `cachedContents`** ——
   你改了脚本文件，跑的还是旧缓存。
5. **代理会干扰** —— 电脑上跑着 sing-box/Clash 时，去 `192.168.1.0/24` 的流量可能被劫持，
   需要策略路由把它排除（`network_setup.sh` 已含）。

---

## 5. 安全须知（重要）

- **机械臂旁必须有人，手放在急停上**。本文档所有运动命令都是真机运动。
- 脚本默认速度 `v=0.05 m/s`（5cm/s）、加速度 `a=0.3`，是很保守的值，先慢后快。
- 任何时刻停止：
  ```bash
  ros2 service call /ur_link/dashboard/stop std_srvs/srv/Trigger
  # 或直接按急停
  ```
- 首次运行请先把物体拿走，空跑一轮确认轨迹没问题，再放物体。
- 机器人处于**保护性停止**或**本地模式**时，30002 的运动命令会被**静默忽略**（不报错），
  先查 `robotmode`。

---

## 6. 许可

本仓库为课程/实验室自用整理，代码可自由取用。URScript 与 Robotiq 协议细节参考各自官方文档。
