#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抓取-放置演示：从 pick 点抓取，移动到 place 点放下。
读 ~/ur_learn/poses.json 里用 ur_capture_pose.py 记录的两个点位。

用法（需已 source ROS 环境）:
    python3 ur_pick_place.py pick place [安全高度m] [--forever|--repeat N]
    python3 ur_pick_place.py pick place 0.05 --no-gripper      # 不带夹爪只走点位
    python3 ur_pick_place.py pick place 0.05 --open 0 --close 255   # 自定义开/闭位置

流程（URScript，command_node 自动 def 包裹+长连接发送）:
    抓取点上方 → 下降 → 夹爪闭合 → 抬起 → 移到放置点上方 → 下降 → 夹爪张开 → 抬起

夹爪：Robotiq 2F-85/2F-140（URCap 已装），用 rq_activate() + rq_move_to(位置)。
     位置 0=张开, 255=闭合（2F-85 满行程）；可用 --open/--close 调。
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

POSES_FILE = os.path.expanduser("~/ur_learn/poses.json")


def P(x, y, z, rx, ry, rz):
    return f"p[{x!r},{y!r},{z!r},{rx!r},{ry!r},{rz!r}]"


if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) < 2:
        print("用法: python3 ur_pick_place.py <pick点> <place点> [安全高度m] [--forever|--repeat N] [--no-gripper] [--open P] [--close P]")
        sys.exit(1)
    pick_name, place_name = args[0], args[1]
    h = float(args[2]) if len(args) > 2 and args[2].replace(".", "").isdigit() else 0.05
    repeat = 1
    forever = False
    # 注意：Robotiq 的 rq_* 函数在“客户端注入脚本”里不可用(已验证，会导致脚本中止)，
    # 所以默认跳过夹爪，只做点位运动；带夹爪请用 demo/pendant_pick_place.urscript 在示教器跑。
    use_gripper = False
    open_pos = 0
    close_pos = 255
    if "--forever" in args:
        forever = True
    if "--repeat" in args:
        repeat = int(args[args.index("--repeat") + 1])
    if "--gripper" in args:  # 强制启用(不推荐，注入脚本里 rq_* 会中止)
        use_gripper = True
    if "--open" in args:
        open_pos = int(args[args.index("--open") + 1])
    if "--close" in args:
        close_pos = int(args[args.index("--close") + 1])

    if not os.path.exists(POSES_FILE):
        print(f"✘ 找不到 {POSES_FILE}，先用 ur_capture_pose.py 记录点位")
        sys.exit(1)
    poses = json.load(open(POSES_FILE))
    if pick_name not in poses or place_name not in poses:
        print(f"✘ 点位缺失，现有: {list(poses)}")
        sys.exit(1)

    px, py, pz, rx, ry, rz = poses[pick_name]
    qx, qy, qz = poses[place_name][:3]
    print(f"抓取点 '{pick_name}': [{px:.4f},{py:.4f},{pz:.4f}]")
    print(f"放置点 '{place_name}': [{qx:.4f},{qy:.4f},{qz:.4f}]  安全高度 {h}m"
          + ("  一直循环" if forever else f"  重复 {repeat} 次"))
    if use_gripper:
        print(f"夹爪: Robotiq 2F  rq_move_to({close_pos})闭合 / rq_move_to({open_pos})张开")
    else:
        print("夹爪: 已跳过(--no-gripper)，只做点位移动")

    # 一段动作（plain 语句，command_node 会 def 包裹）
    def one_cycle():
        lines = []
        if use_gripper:
            lines += ["rq_activate()", "sleep(0.5)"]       # 激活 Robotiq 夹爪(需一次)
        lines += [
            f"movel({P(px, py, pz + h, rx, ry, rz)}, a=0.3, v=0.05)",   # 抓取点上方
            f"movel({P(px, py, pz, rx, ry, rz)}, a=0.3, v=0.05)",       # 下降到抓取点
            "sleep(0.3)",
        ]
        if use_gripper:
            lines += [f"rq_move_to({close_pos})", "sleep(0.8)"]         # 夹爪闭合
        lines += [
            f"movel({P(px, py, pz + h, rx, ry, rz)}, a=0.3, v=0.05)",   # 抬起
            f"movel({P(qx, qy, qz + h, rx, ry, rz)}, a=0.3, v=0.05)",   # 移到放置点上方
            f"movel({P(qx, qy, qz, rx, ry, rz)}, a=0.3, v=0.05)",       # 下降到放置点
            "sleep(0.3)",
        ]
        if use_gripper:
            lines += [f"rq_move_to({open_pos})", "sleep(0.8)"]          # 夹爪张开
        lines += [
            f"movel({P(qx, qy, qz + h, rx, ry, rz)}, a=0.3, v=0.05)",   # 抬起离开
        ]
        return "\n".join(lines)

    if forever:
        indented = "\n".join("  " + ln for ln in one_cycle().splitlines())
        script = f"while True:\n{indented}\nend"
    elif repeat == 1:
        script = one_cycle()
    else:
        # 循环重复：用一个计数 while 包起来（def 里运动 OK）
        indented = "\n".join("  " + ln for ln in one_cycle().splitlines())
        script = f"n := 0\nwhile n < {repeat}:\n{indented}\n  n := n + 1\nend"

    rclpy.init()
    n = Node("ur_pick_place")
    pub = n.create_publisher(String, "/ur_link/urscript", 10)
    time.sleep(0.5)
    msg = String()
    msg.data = script
    pub.publish(msg)
    print("已发送 URScript:\n" + script)
    print("\n停止: ros2 service call /ur_link/dashboard/stop std_srvs/srv/Trigger  或按急停")
    rclpy.shutdown()
