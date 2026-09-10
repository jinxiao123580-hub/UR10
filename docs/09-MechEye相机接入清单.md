# 09 · Mech-Eye 3D 相机接入清单（照着做）

> 目标：把梅卡曼德 **Mech-Eye PRO XS** 接进 ROS 2，能采到点云，并能接到机器人坐标系里。
> 本机相机实测信息：**IP `192.168.1.33`**，固件/版本 **2.5.0**，SN **RAM35238A3020004**，
> MAC `24:c5:d3:55:00:00`，控制端口 **TCP 5577** 已开。

> **当前状态（2026-09-10）**：SDK 2.5.0 已安装，接口已编译，彩色/深度/点云均已真机采集成功。

---

## 步骤 0 · 先跑预检（不改任何东西）

```bash
python3 scripts/setup_mecheye.py --check
```

它会自动发现相机（GVCP 广播，**IP 不在本网段也能找到**）并检查 SDK / 依赖。
正常输出：

```
[1/6] 发现相机（GVCP 广播）
   ★ 192.168.1.33  Mech-Mind Robotics Mech-Eye PRO XS (固件 2.5.0, SN RAM35238A3020004)  MAC 24:c5:d3:55:00:00
[2/6] 检查 Mech-Eye SDK
   SDK 安装目录 /opt/mech-mind/mech-eye-sdk: ✘ 不存在
[3/6] 检查依赖
   libopencv-dev ✔ / ros-humble-cv-bridge ✔ / libpcl-dev ✔
   ros-humble-pcl-conversions ✔ / python3-colcon-common-extensions ✔
```

**本机现状**：SDK 2.5.0 与依赖均已安装。此预检输出保留作重装/排障参考。

---

## 步骤 1 · 装 Mech-Eye SDK（需要你操作）

SDK 要**官网注册后下载**，我这边下载不了。

| 来源 | 地址 |
|---|---|
| 国内镜像（推荐，快） | <https://downloads.mech-mind.com.cn/?tab=tab-sdk> |
| 国际站点 | <https://downloads.mech-mind.com/?tab=tab-sdk> |

下载得到 **`Mech-Eye_API_2.5.0_amd64.zip`**，然后在**你自己的终端**执行：

```bash
sudo apt-get install libarchive-zip-perl       # 提供 crc32 命令
crc32 Mech-Eye_API_2.5.0_amd64.zip             # 与下载页给出的校验码比对
unzip Mech-Eye_API_2.5.0_amd64.zip             # 解压出 .deb
sudo dpkg -i Mech-Eye_API_2.5.0_amd64.deb      # 安装
dpkg -l | grep mecheyeapi                      # 确认已装
ls /opt/mech-mind/mech-eye-sdk/                # 确认目录（脚本就认这个路径）
```

> **版本怎么选？** 本机相机是 **2.5.0**。文档站各版本都有一份
> （[2.5.0 安装指南](https://docs.mech-mind.net/zh/eye-3d-camera/2.5.0/api/software-installation.html)、
> [2.6.0 安装指南](https://docs.mech-mind.net/zh/eye-3d-camera/2.6.0/api/software-installation.html)）。
> **本机已实测：必须安装 2.5.0。** SDK 2.6.0 启动时会明确报告
> `Mech-Eye PRO XS` 不受该版本支持，并要求使用 2.5.0 或更低版本。

> 顺便：官方 launch 用 `prefix="xterm -e"` 所以需要 `xterm`；
> **我们的 launch 不用 xterm**，所以可以不装。

---

## 步骤 2 · 一键完成剩余步骤

```bash
python3 scripts/setup_mecheye.py
```

脚本会做四件事（**可重复运行，幂等**）：

| 步骤 | 做什么 |
|---|---|
| 4/6 | 克隆/更新 [`mecheye_ros2_interface`](https://github.com/MechMindRobotics/mecheye_ros2_interface) 到 `~/colcon_ws/src/` |
| 5/6 | **自动打补丁** + 生成专用 launch（见下面的坑） |
| 6/6 | `colcon build --packages-select mecheye_ros_interface` |

---

## 步骤 3 · 启动并验证点云

```bash
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
ros2 launch ~/colcon_ws/src/mecheye_ros2_interface/launch/start_camera_ur10.py
```

> ⚠️ **工业相机是"服务触发式"采集，不是连续推流！**
> 启动后话题存在，但**不调服务就没有数据**：

```bash
# 另开终端
ros2 service call /capture_point_cloud   mecheye_ros_interface/srv/CapturePointCloud
ros2 service call /capture_depth_map     mecheye_ros_interface/srv/CaptureDepthMap
ros2 service call /capture_color_image   mecheye_ros_interface/srv/CaptureColorImage

# 看结果
ros2 topic echo /mechmind/point_cloud --field header --once
rviz2        # 加 PointCloud2 → /mechmind/point_cloud
```

### 本机验收记录（2026-09-10）

| 验收项 | 实测结果 |
|---|---|
| 相机连接 | `192.168.1.33`，PRO XS，SN `RAM35238A3020004`，固件 2.5.0 |
| 彩色服务 | `CaptureColorImage_Response(error_code=0, error_description='')` |
| 深度服务 | `CaptureDepthMap_Response(error_code=0, error_description='')` |
| 点云服务 | `CapturePointCloud_Response(error_code=0, error_description='')` |
| 彩色消息 | 宽度 1280；网页桥收到 1280×1024 JPEG |
| 点云消息 | 有效时间戳；`frame_id=mechmind_camera/point_cloud` |

服务耗时实测：彩色约 **2.435s**、深度约 **2.359s**、点云约 **4.512s**。
因此刷新瓶颈主要在相机曝光/结构光采集与 GigE 传输，不在网页或台式机 CPU。

### 话题与 frame_id（注意这些 frame 名字）

| 话题 | 类型 | frame_id |
|---|---|---|
| `/mechmind/point_cloud` | `sensor_msgs/PointCloud2` | `mechmind_camera/point_cloud` |
| `/mechmind/textured_point_cloud` | `sensor_msgs/PointCloud2` | `mechmind_camera/textured_point_cloud` |
| `/mechmind/depth_map` | `sensor_msgs/Image`(32F) | `mechmind_camera/depth_map` |
| `/mechmind/color_image` | `sensor_msgs/Image`(bgr8) | `mechmind_camera/color_map` |
| `/mechmind/stereo_color_image_left/right` | `sensor_msgs/Image` | `mechmind_camera/left_color_map` / `right_color_map` |
| `/mechmind/camera_info` | `sensor_msgs/CameraInfo` | — |

**这些 frame 名字就是接到机器人 TF 树上的挂点**（见步骤 4）。

---

## 步骤 4 · 接到机器人坐标系（手眼标定）

点云现在活在 `mechmind_camera/point_cloud` 这个孤立坐标系里，**必须做手眼标定**
才能把物体坐标换算到机器人 `base` 系。方法见
[`docs/08` 第 3.5 节](08-六维力与3D相机.md#35-手眼标定怎么做适配本机的-cb3-方案)
（`easy_handeye2` 的 `freehand_robot_movement` 模式，适配我们没有 MoveIt 的 CB3）。

标定前先起 TF 链：

```bash
ros2 run ur_link ur_state_node --ros-args -p robot_ip:=192.168.1.3   # /joint_states
bash scripts/start_ur_tf.sh ur10                                     # base → tool0
python3 scripts/check_fk.py                                          # 验证（必做）
```

> 🔴 **务必用 `base` 系（不是 `base_link`）**——实测两者 X/Y 镜像，用错差 **1.29 米**。
> 详见 [`docs/08` 第 3.1 节](08-六维力与3D相机.md#31-tf--坐标系先解决-base-与-base_link-的坑已实测)。

---

## ⚠️ 四个坑（不处理会浪费你半天）

### 坑 1 · 官方源码里"按 IP 连接"那段是注释掉的

`src/MechMindCamera.cpp` 里虽然声明了 `camera_ip` 参数，但**实际连接用的是
`findAndConnect(camera)` 自动发现**，会**交互式让你输入相机序号**。按 IP 连接的那段被注释了：

```cpp
// Uncomment the following lines and comment the above if function to connect to a specific
// camera by its IP address.
// info.firmwareVersion = mmind::eye::Version("2.3.4");   ← 还写死了 2.3.4！
```

**`2.3.4` 是官方示例相机的版本，我们相机是 `2.5.0`** —— 照抄会连不上。

`scripts/setup_mecheye.py` 的补丁做法更稳：**保留官方自动发现作为 `else` 分支**，
只在 `camera_ip` 非空时按 IP 直连，且固件版本**从相机自动读出来**（不写死）：

```cpp
// >>> UR10-setup
if (!camera_ip.empty()) {
    mmind::eye::CameraInfo info;
    info.ipAddress = camera_ip;
    info.port = 5577;                                   // 实测相机该端口已开
    info.firmwareVersion = mmind::eye::Version(camera_fw_version.c_str());
    auto connect_status = camera.connect(info);
    if (!connect_status.isOK()) throw connect_status;
} else
// <<< UR10-setup
if (!findAndConnect(camera)) throw ...;                 // 官方原逻辑，未改动
```

### 坑 2 · 官方 launch 参数名写错了

`launch/start_camera.py` 里传的是：

```python
{"user_external_intri": False},      # ✘ 官方 launch 里写错了
```

而代码里声明的是 **`use_external_intri`**。结果：**这个参数根本没生效**（用外部内参的意图被忽略）。
我们生成的 launch 用正确名字。

### 坑 3 · 官方 launch 发布的是"假的" static TF

```python
# 官方 start_camera.py
arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_camera/point_cloud']
```

它在 `map` 和相机之间发布了一个**凭空捏造的固定变换**（位置 0,0,1）。
**这比没有更危险**——点云会"看起来"在机器人坐标系里，但位置是错的，且**不报错**。

我们的 launch **不发布任何相机 TF**，强制你先做手眼标定。

### 坑 4 · 忘了它是"服务触发"

工业相机 ≠ RealSense。启动后**话题是空的**，要调 `capture_point_cloud` 才有数据。
第一次用容易以为"驱动坏了"。

---

## 故障排查

| 症状 | 原因 | 解决 |
|---|---|---|
| `--check` 发现不了相机 | 相机没上电 / 不在同一二层网络 / 防火墙 | 确认网线；`ping 192.168.1.33`；关掉电脑上的代理 |
| colcon 编译失败 `MechEyeApi not found` | SDK 没装或路径不对 | `ls /opt/mech-mind/mech-eye-sdk/`；重装 SDK |
| 启动报 `Camera not found` | `camera_ip` 没打补丁生效 / IP 错 | 跑 `setup_mecheye.py`（会自动打补丁）；确认 IP |
| `connect` 失败/版本不匹配 | 固件版本写死成 2.3.4 | 我们的补丁会从相机读取版本；若仍失败，改用官方自动发现（`camera_ip` 留空） |
| 话题在但点云为空 | **没调采集服务** | `ros2 service call /capture_point_cloud ...` |
| `save_file` 导致写入失败 | 官方示例会往 `/tmp/` 存文件 | 我们的 launch 已设 `save_file: False` |
| rviz2 里看不到点云 | frame 不对 / QoS | Fixed Frame 设 `mechmind_camera/point_cloud` 先验证，再谈坐标变换 |
| 点云位置明显不对 | 用了官方那个假 static TF | 删掉，改用手眼标定结果 |

---

## 依赖版本一览（实测）

| 组件 | 版本/状态 |
|---|---|
| Ubuntu / ROS | 22.04 + Humble（**官方推荐组合**） |
| `Mech-Eye SDK` | ✘ 待装（下载需注册） |
| `libopencv-dev` / `ros-humble-cv-bridge` | ✔ 已装 |
| `libpcl-dev` / `ros-humble-pcl-conversions` | ✔ 已装 |
| `python3-colcon-common-extensions` | ✔ 已装 |
| `mecheye_ros2_interface` | 已克隆到 `~/colcon_ws/src/`（package 名 `mecheye_ros_interface`，v0.0.2） |
