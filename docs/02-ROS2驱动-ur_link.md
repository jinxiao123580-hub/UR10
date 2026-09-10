# 02 · ROS2 驱动 `ur_link`

`ur_link` 是一个**最小 UR ROS2 桥**：把 UR 控制器原生的三个 TCP 接口包成 ROS 话题/服务，
**机器人侧不需要安装任何东西**（不用 ur_robot_driver、不用 External Control URCap、不用改示教器）。

适合"先把通路打穿"的场景（课程/实验室）。要 MoveIt 实时规划请另外上官方 `ur_robot_driver`。

## 1. 包结构

```
ros2_ws/src/ur_link/
├── package.xml
├── setup.py / setup.cfg
├── launch/ur_link.launch.py          # 一次起两个节点
├── README.md
└── ur_link/
    ├── ur_protocol.py                # 30001/30002/29999 三个口的裸协议封装
    ├── state_node.py                 # → /joint_states + 状态话题（读 30001）
    ├── command_node.py               # /ur_link/urscript + dashboard 服务（写 30002/29999）
    └── gripper_cmd.py                # 旧示教器夹爪方案（已废弃，留档）
```

## 2. 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select ur_link
source install/setup.bash
```

```bash
# 方式 A：一次起全
ros2 launch ur_link ur_link.launch.py robot_ip:=192.168.1.3

# 方式 B：分开起（调试时推荐）
ros2 run ur_link ur_state_node   --ros-args -p robot_ip:=192.168.1.3
ros2 run ur_link ur_command_node --ros-args -p robot_ip:=192.168.1.3
```

## 3. 接口清单（实测 `ros2 topic list` / `service list`）

### 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 6 关节 位置/速度/电流（10Hz，读 30001） |
| `/ur_link/is_program_running` | `std_msgs/Bool` | 是否有程序在跑 |
| `/ur_link/robot_mode` | `std_msgs/Int32` | 机器人模式 |
| `/ur_link/speed_fraction` | `std_msgs/Float64` | 速度系数 |
| **`/ur_link/urscript`** | `std_msgs/String` | **发任意 URScript → 30002**（机械臂运动就用它） |

### 服务（全部 `std_srvs/srv/Trigger`）

```
/ur_link/dashboard/power_on          /ur_link/dashboard/power_off
/ur_link/dashboard/brake_release     /ur_link/dashboard/close_safety_popup
/ur_link/dashboard/play              /ur_link/dashboard/pause
/ur_link/dashboard/stop              /ur_link/dashboard/robotmode
/ur_link/dashboard/is_program_running /ur_link/dashboard/get_loaded_program
```

### 用法示例

```bash
ros2 topic echo /joint_states --once

# 上电 + 松刹车（机械臂旁必须有人按着急停）
ros2 service call /ur_link/dashboard/power_on      std_srvs/srv/Trigger
ros2 service call /ur_link/dashboard/brake_release std_srvs/srv/Trigger

# 让它动 —— 注意末尾必须有个调用，见 docs/03 的坑
ros2 topic pub --once /ur_link/urscript std_msgs/String \
  "data: 'def go():\n  movej([0,-1.5708,0,-1.5708,0,0], a=1.4, v=1.05)\nend\ngo()'"

# 停
ros2 service call /ur_link/dashboard/stop std_srvs/srv/Trigger
```

## 4. `command_node` 的关键实现细节

```python
def _on_urscript(self, msg: String):
    text = msg.data.strip()
    # UR 客户端接口要求：脚本用 def/sec 包裹、行首缩进、end 收尾，
    # 否则 URControl 会静默丢弃。运动指令只能用主程序 def（sec 线程禁运动）。
    if not text.startswith(("sec ", "def ")):
        indented = "\n".join("  " + ln for ln in text.splitlines())
        text = f"def ros_cmd():\n{indented}\nend"
    ...
    s.sendall((text + "\n").encode())
    # 长连接：保持打开并读应答，别发完就关，否则脚本可能被丢弃
```

**两条硬规则**：

1. **必须 `def` 包裹**，裸语句会带缩进自动包成 `def ros_cmd(): ... end`；
2. **包成 `def` 后必须再调用一次**（`arm_cmd()`），否则只是"定义了一个函数"，机器人**纹丝不动且不报错**。
   本仓库 `scripts/ur_arm.py` 里的 `send_script(..., call=True)` 已自动在末尾补上调用。

> 历史遗留：早期 `ur_rel_move.py` / `ur_circle.py` 里只发 `def ros_move(): movel(...) end` 没调用，
> 是能"看似成功"却不动的原因之一（当时靠 30002 的其它行为掩盖了）。

## 5. 离线自测（手边没机器人也能调代码）

```bash
# 终端 1：起一个假的 UR 服务器（监听 127.0.0.1）
python3 scripts/fake_ur_server.py 0

# 终端 2：让驱动连假的机器人
ros2 run ur_link ur_state_node --ros-args -p robot_ip:=127.0.0.1
ros2 topic echo /joint_states
```

这样可以先验证"话题通、脚本拼对、解析没错"，再去碰真机器人。

## 6. 注意事项

- 机器人**没上电 / 急停 / 保护性停止**时，30002 发的运动命令会被**静默忽略**（不报错！先查 `robotmode`）。
- e-Series 处于**本地(local)模式**时 `30001/30002/30003` 会断开，改连 `30011/30012/30013`。
- CB3 老固件（<3.5）没有帧头流，本包退化为尽力解析，仍能读 `q_actual`。
- 速度/加速度别超默认上限；**始终有人在急停旁**。
