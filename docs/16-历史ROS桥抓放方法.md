# 16 · 历史 ROS 桥抓放方法（原样留档）

本页按原样保留一套旧对话中的 ROS 抓放方法，**不修改其中任何命令或脚本**。它依赖旧的
`ur_link` ROS 桥和 `~/ur_learn` 副本，与当前原生 Python 位置演示不是同一入口。

当前展示入口仍是 [`15-位置抓放展示流程.md`](15-位置抓放展示流程.md)。本页只用于复现旧环境、
课程对照和历史记录；不要把它与当前官方 ROS 2 Driver 或当前安全状态混用。

## 原始两个终端命令

终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ur_link ur_command_node --ros-args -p robot_ip:=192.168.1.3
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ur_learn/scripts/ur_pick_place_full.py 2 --pingpong
```

这里的 `2 --pingpong` 表示执行 2 轮往返：第一轮 A→B，第二轮 B→A。脚本和命令均保持
原样，本页没有对其增加负载、力控或安全逻辑。
