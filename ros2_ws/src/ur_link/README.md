# ur_link —— 最小 UR ROS2 桥（无需机械臂侧安装任何东西）

把 UR 控制器三个原生 TCP 接口包成 ROS2 节点，适合课程/实验室“先打通”场景：
CB3(固件≥3.5) 与 e-Series 均适用。要 MoveIt 实时规划控制请走官方
ur_robot_driver + externalcontrol URCap(需要示教器侧安装)。

## 接口
| ROS 侧 | 说明 | UR 侧 |
|---|---|---|
| `/joint_states` (sensor_msgs/JointState) | 6 关节 位置/速度/电流 | 30001 Primary @10Hz |
| `/ur_link/is_program_running`, `/ur_link/robot_mode`, `/ur_link/speed_fraction` | 运行/模式/速度系数 | 同上 |
| `ur_link/dashboard/power_on` 等 Trigger 服务 | power_on/brake_release/close_safety_popup/stop/pause/play/robotmode/... | 29999 Dashboard |
| `~/urscript` (std_msgs/String 话题) | 发任意 URScript | 30002 Secondary |

## 构建
```bash
cd ~/ros2_ws
colcon build --packages-select ur_link
source install/setup.bash
```

## 运行
```bash
# 实机
ros2 launch ur_link ur_link.launch.py robot_ip:=192.168.1.3
# 或分开跑：
ros2 run ur_link ur_state_node --ros-args -p robot_ip:=192.168.1.3
ros2 run ur_link ur_command_node --ros-args -p robot_ip:=192.168.1.3
```

## 用起来（示例）
```bash
ros2 topic echo /joint_states --once

# 上电+松刹车（机械臂旁必须有人按住急停随时准备）
ros2 service call /ur_link/dashboard/power_on std_srvs/srv/Trigger
ros2 service call /ur_link/dashboard/brake_release std_srvs/srv/Trigger

# 让它动：发 URScript(先载入任意程序并 play 才有程序上下文；或直接发脚本也会执行)
ros2 topic pub --once /ur_link/urscript std_msgs/String \
  "data: 'movej([0,-1.5708,0,-1.5708,0,0], a=1.4, v=1.05)'"
# 建议先加  sleep(0.5) 等待编译：
ros2 topic pub --once /ur_link/urscript std_msgs/String \
  "data: 'sleep(0.5)\nmovej([0,-1.5708,0,-1.5708,0,0], a=1.4, v=1.05)'"
```

## 离线自测（不需要机械臂）
```bash
python3 ~/ur_learn/scripts/fake_ur_server.py 0     # 模拟 127.0.0.1 的 UR
ros2 run ur_link ur_state_node --ros-args -p robot_ip:=127.0.0.1
ros2 topic echo /joint_states
```

## 注意
- 机械臂没上电/急停/保护性停止时，30002 发的运动命令会被忽略，先看 robotmode。
- e-Series 处于“本地(local)”模式时 30001/30002/30003 断开，改连 30011/30012。
- 老 CB3 固件(<3.5)没有帧头流，本包会退化为尽力解析，仍能读 q_actual。
- 速度/加速度给默认上限以内；始终有人在急停旁。
