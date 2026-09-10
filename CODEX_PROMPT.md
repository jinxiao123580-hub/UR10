# 交给 Codex 的提示词

**使用方法**：把 `---` 以下的内容**整段复制**粘给 Codex（它看不到之前的对话，所以这段是自包含的）。
如果 Codex 能访问本机文件系统，它会自己读仓库；否则请把 `HANDOVER.md` 一并附上。

---

# 任务：接手「UR10 机械臂 + 夹爪 + 六维力 + 3D 相机」视觉力觉抓取项目

你接手一个**已经在真机上跑通多个闭环**的机器人项目，前一位工程师完成了大量实测与踩坑。
你的价值**不在于重新探索，而在于沿着已验证的结论继续推进**，并且**不要重复踩已知的坑**。

仓库位置：`~/UR10`（远程：`https://github.com/jinxiao123580-hub/UR10`，分支 `main`）
工作机：Ubuntu 22.04 + ROS 2 Humble，用户 `jx`，`sudo` **需要密码**（所以系统级安装要给用户命令，由用户执行）

## 一、第一步：先读文档，不要急着动手

按这个顺序读（都在 `~/UR10/`）：

1. **`HANDOVER.md`** ← 交接主文档，**最重要**。含设备参数、当前状态、五条铁律、待办、验证命令
2. `docs/08-六维力与3D相机.md` ← 力传感器 RDT 协议（实测钉死）、相机发现、**base/base_link 坐标系坑**
3. `docs/09-MechEye相机接入清单.md` ← 相机接入步骤与官方代码的四个坑
4. `docs/04-夹爪控制.md` ← 夹爪直控架构 + **`rq_activate()` 死锁**根因
5. `docs/10-网页监控台.md` ← 网页看板架构与自研 ROS→WebSocket 协议

## 二、项目是什么

用**一台普通电脑上的 ROS 2**，同时控制四样东西，**不依赖示教器编程**：

```
机械臂(UR10 CB3)  ← ROS 话题 /ur_link/urscript → 端口 30002 → URScript movel/movej
夹爪(Robotiq 2F)  ← 电脑直连 TCP 63352       → Robotiq ASCII 协议
六维力(ATI Net F/T)← UDP 49152 (RDT 协议)     → /ft_sensor/wrench (WrenchStamped)
3D相机(Mech-Eye PRO XS) ← GigE Vision         → /mechmind/* （待装 SDK）
网页看板          ← 自研 WebSocket 桥 9090 + HTTP 8080
```

设备 IP：机器人 `192.168.1.3`、力传感器 `192.168.1.2`、相机 `192.168.1.33`、电脑 `192.168.1.10`。

## 三、当前状态（已真机验证 / 未完成）

**已验证可用**：ROS 控制机械臂运动（实测位移 3.1mm）· 电脑直控夹爪开合 · **完整抓放闭环跑通**
（`ur_pick_place_full.py`，夹爪 `OBJ=2` 确认夹住）· 六维力 **200Hz** 进 ROS · 正运动学 TF 链
（与真机偏差 2.5mm）· 网页看板（力 200Hz 实时折线 + 相机画面，相机部分当前是模拟器）。

**未完成（你的任务）**：相机 SDK 未装（**取不到图**）→ 手眼标定未做 → 视觉引导抓取未做。

## 四、你的任务（按优先级）

### P0 · 让 Mech-Eye 相机出图（**需要用户协助，sudo 要密码**）

现状：相机已能被发现（GVCP 广播能拿到型号 `Mech-Eye PRO XS`、固件 `2.5.0`、SN `RAM35238A3020004`），
`~/colcon_ws/src/mecheye_ros2_interface/` 已克隆**且已打补丁**，但 **Mech-Eye SDK 未装**，所以编译不了、取不到图。

你要做的：
1. 先跑 `python3 ~/UR10/scripts/setup_mecheye.py --check` 确认现状
2. 让用户执行（SDK 需官网注册下载，你下载不了）：
   ```bash
   # 下载：https://downloads.mech-mind.com.cn/?tab=tab-sdk  → Mech-Eye_API_2.6.0_amd64.zip
   sudo apt-get install libarchive-tools && crc32 Mech-Eye_API_2.6.0_amd64.zip
   unzip Mech-Eye_API_2.6.0_amd64.zip && sudo dpkg -i Mech-Eye_API_2.6.0_amd64.deb
   ```
3. 装好后跑 `python3 ~/UR10/scripts/setup_mecheye.py`（自动发现相机 → 打补丁 → 编译 → 生成 launch → 验证）
4. 验收：`ros2 service call /capture_color_image mecheye_ros_interface/srv/CaptureColorImage`
   返回成功，且 `http://127.0.0.1:8080/` 左侧出现**真实画面**（替换掉 "FAKE CAMERA" 模拟器）

### P1 · 手眼标定（P0 完成后）

- 选 **eye-to-hand**（相机固定）：`base → 相机光学系` 是常量，换夹爪不用重标
- 工具 `easy_handeye2`（**不在 apt，需源码编译**）。⚠️ 其"自动移动机器人"在 master 上被
  `if False:` 短路禁用，且依赖的 `moveit_commander` 在 MoveIt 2 Humble 里不存在 →
  **只能用 `freehand_robot_movement:=true` 手动摆位取点**（正好适配本机无 MoveIt 的 CB3）
- 相机 frame 必须是**光学系**（Z 前/X 右/Y 下）；**15~25 个姿态，姿态多样性比数量重要**
- 验证：重投影误差 → **实抓端到端**（最终判据）

### P2 · 视觉引导抓取（P1 完成后）

`点云 → 分割/位姿估计 → tf2 变换到 base → 加夹爪偏置+安全高度 → Arm.movel() → 夹爪闭合 → OBJ 校验`
直接复用 `ur_pick_place_full.py`，只把 `poses.json` 换成实时算出的位姿，**其余不用改**。

### P2 · 力控应用（无阻塞，数据已通）

碰撞检测（超阈值即停，`movel` 前先 tare）· 力引导插装 · 用 Fz 判断抓取是否成功（比 `OBJ` 更定量）。

## 五、五条铁律（**不看你一定会踩，且会浪费几小时**）

1. **30002 发的 `def` 必须显式调用**：`"def f():\n movel(...)\nend"` **不动且不报错**，
   必须 `...end\nf()`。用 `scripts/ur_arm.py::send_script(call=True)` 就自动补了。
   验证通路用 `python3 scripts/ur_arm.py nudge 0 0 -0.003`（会打印"位移 X 米 ✔动了/✘没动"）。
2. **绝不要调 `rq_activate()`**：它的等待循环 `while(ACT!=0 or STA!=0)` 在夹爪正常态 `ACT=1` 时
   **永远出不去 → 程序死锁**（症状：socket 连上了但命令无回执、连接也不断开）。
   夹爪平时就是激活的；要激活直接发 ASCII `SET ACT 1`。
3. **视觉目标必须变换到 `base` 系，不是 `base_link`**：URDF 里 `base_link` 相对 `base`
   绕 Z 转 180°（X/Y 镜像），**用错偏差 1.29 米（跑到反方向去抓）**。
   且 `movel(p[x,y,z,rx,ry,rz])` 的 `rx,ry,rz` 是**轴角不是 RPY**——用 `ur_arm.py` 里的
   `quat_to_rotvec()` 转换。验证：`python3 scripts/check_fk.py`。
4. **ATI 力传感器 RDT 是单目的地单播**：只允许**一个**消费者，两个进程同时读会互相抢流，
   表现为**数据每 5 秒一跳**（不是传感器坏）。跑 `ati_netft_node.py --check` 会**抢走**运行中节点的流。
5. **改文件先备份**：本仓库改动靠 git 提交兜底；**外部目录**（`~/colcon_ws/src/mecheye_ros2_interface/`）
   的改动已有备份在 `~/ur_learn/generated/backups/`（`.orig`/`.patched`/`.patch`），
   还原：`cd ~/colcon_ws/src/mecheye_ros2_interface && git checkout -- src/ include/`

## 六、其他必须知道的实测细节

- **夹爪 ASCII**：`SET POS 0`(开)/`SET POS 255`(合)，**必须补 `SET GTO 1` 否则不动**；
  `GET OBJ` → **2=夹住了**，3=抓空；`POS` 停在 104/230 说明**夹住东西了**（正常）。
  响应是**补零格式**（`FLT 00`、`SID [9]`），别直接 `int()`，用正则抠数字。
- **ATI RDT**：启动命令 `12 34 00 <采样数> 00 00 00 00`，**采样数只能 1(200Hz) 或 2(822Hz)**，
  `10`/`255` 收不到数据；数据包 36 字节 = 9 个大端 int32
  `[0]rdt_seq [1]ft_seq [2]status [3..8]Fx Fy Fz Tx Ty Tz`；物理量 = counts/1e6。
  **传感器从未置零**（Bias 全 0），当前 `Fz≈62N` 是**工装自重**，用前必须 tare。
  活动配置是出厂测试项 `#16 End of line test`。
- **相机是服务触发式采集**，不是连续推流：`ros2 service call /capture_point_cloud ...` 才有数据。
  官方 launch 会发布一个**凭空捏造的 `map→相机` 静态变换（0,0,1）——比没有更危险**，别用；
  我们的 `start_camera_ur10.py` 故意不发布任何相机 TF。
- **iOS/网页部分**：`websockets` 库是 16.x，**没有 `send_str`**（要 `await ws.send()`），
  跨线程发送用 `asyncio.run_coroutine_threadsafe`。
- **机器人内部访问**：`ssh root@192.168.1.3`（密码 `easybot`）；`.urp` 是 **gzip 压缩的 XML**；
  程序文件在 `/programs/`。详见 `docs/07`。
- **网络**：电脑跑着 sing-box 代理，去 `192.168.1.0/24` 的流量需绕过（`scripts/network_setup.sh` 已含策略路由）。

## 七、验证命令速查（每个子系统一条）

```bash
python3 ~/UR10/scripts/diagnose.py                              # 全系统体检（先跑这个）
python3 ~/UR10/scripts/ati_netft_node.py --check --hz 5 --count 4   # 力传感器出数
python3 ~/UR10/scripts/rq_gripper.py test                       # 夹爪实际开合
python3 ~/UR10/scripts/ur_arm.py nudge 0 0 -0.003               # 机械臂通路
python3 ~/UR10/scripts/setup_mecheye.py --check                 # 相机发现 + SDK 状态
bash ~/UR10/scripts/start_dashboard.sh                          # 网页看板 → :8080
```

机械臂相关命令前需：`source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`

## 八、工作方式要求（这是本项目最看重的）

1. **证据驱动**：每个结论都要有客观证据（位姿数值、POS/OBJ 数值、端口状态、HTTP 码、字节布局）。
   **不要靠"看起来在跑"判断**——这个项目里所有的坑，本质都是"以为通了实际没通"。
2. **先问硬件本体**：能直连设备就直接问（夹爪 63352、力传感器 HTTP 页/RDT、相机 GVCP 广播）。
   本项目的两个关键突破（夹爪直控、相机型号）都是这么来的，比翻文档和猜快得多。
3. **协议不明就穷举**：ATI 的启动命令试 `0x01`/`0x02` 全失败，实际是 `0x00`+采样数。
4. **诚实标注**：未验证的写"未验证"，不要编造 API/包名/参数。
5. **安全第一**：**机械臂旁必须有人，手放急停**。运动前先空跑验证轨迹；速度默认 `v=0.05 m/s` 很保守，
   改速度要逐步来。急停/停止：
   `ros2 service call /ur_link/dashboard/stop std_srvs/srv/Trigger`
6. **报告风格**：说清"做了什么、证据是什么、下一步是什么"，中文回答。

## 九、开始

读完 `HANDOVER.md` 与上述文档后，先做两件事再动手：
1. 跑 `python3 ~/UR10/scripts/diagnose.py`，把实际状态对照交接文档确认一遍（避免文档与现场不一致）；
2. 告诉我：现在相机 SDK 装了没有、你想让我（用户）先执行哪些需要 sudo 的命令。

然后从 **P0（相机出图）** 开始推进。
