#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
让 UR 末端在水平面（基座 XY 平面、Z 不变）画圆 —— 用 URScript 的 movec（圆弧插补）。

修正记录（2026-09-10，相对旧版）:
  ① 旧版用 `n := 0` —— URScript 赋值是 `=`，`:=` 语法错误（本地手册验证）→ 已改 `=`
  ② 旧版发裸脚本 —— 被 ur_command_node 包成 `def ros_cmd(): ... end` 但**不调用**，
     机器人不动且不报错（本项目铁律 1）→ 已改成 def + 显式调用 `circle()`
  ③ 发送走 ur_arm.send_script（自动补调用，和 nudge 验证同一条可靠路径）

用法（需已 source ROS 环境 + ur_command_node 在跑）:
    python3 ur_circle.py 0.02                 # 半径20mm，一直画(默认)
    python3 ur_circle.py 0.02 --rounds 3      # 画 3 圈后自动停下
    python3 ur_circle.py 0.05                 # 半径50mm

停止: 按急停，或
    ros2 service call /ur_link/dashboard/stop std_srvs/srv/Trigger
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ur_arm import Arm, read_packet          # noqa: E402

ROBOT_IP = "192.168.1.3"


def main():
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print("用法: python3 ur_circle.py 半径 [--rounds N]  例: ur_circle.py 0.02")
        return 1
    r = float(args[0])
    rounds = None
    if "--rounds" in args:
        rounds = int(args[args.index("--rounds") + 1])

    cx, cy, cz, rx, ry, rz = read_packet()
    print("当前 TCP 位姿: xyz=[%.4f,%.4f,%.4f] 姿态=[%.4f,%.4f,%.4f]"
          % (cx, cy, cz, rx, ry, rz))

    # 圆在水平面：4 个点（圆心(cx,cy,cz) 半径 r），两个 movec 拼一个整圆
    start = "p[%r,%r,%r,%r,%r,%r]" % (cx + r, cy, cz, rx, ry, rz)
    via1 = "p[%r,%r,%r,%r,%r,%r]" % (cx, cy + r, cz, rx, ry, rz)
    to1 = "p[%r,%r,%r,%r,%r,%r]" % (cx - r, cy, cz, rx, ry, rz)
    via2 = "p[%r,%r,%r,%r,%r,%r]" % (cx, cy - r, cz, rx, ry, rz)

    lines = ["def circle():",
             "  movel(%s, a=0.3, v=0.05)" % start,          # 先走到起点
             "  n = 0"]
    if rounds is None:
        lines.append("  while True:")
    else:
        lines.append("  while n < %d:" % rounds)
    lines += [
        "    movec(%s, %s, a=0.3, v=0.05, r=0.001)" % (via1, to1),
        "    movec(%s, %s, a=0.3, v=0.05, r=0.001)" % (via2, start),
        "    n = n + 1",
        "  end",
        "end",
        "circle()",                                        # ← 必须调用，否则不动
    ]
    script = "\n".join(lines)

    arm = Arm(ip=ROBOT_IP)
    arm.send_script(script, call=False)   # script 末尾已自带 circle()，这里不再补
    print("已发送画圆 URScript（半径 %gm，圆心 [%.4f,%.4f,%.4f]）:" % (r, cx, cy, cz))
    print(script)
    print("\n停止: ros2 service call /ur_link/dashboard/stop std_srvs/srv/Trigger  或按急停")
    return 0


if __name__ == "__main__":
    sys.exit(main())
