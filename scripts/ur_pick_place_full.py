#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR10 完整抓放（最终版）—— 两个通道都是电脑直控：

    机械臂：ROS 话题 /ur_link/urscript  →  30002  →  movel()
    夹  爪：TCP 192.168.1.3:63352       →  Robotiq ASCII 协议

不需要示教器跑任何程序、不需要 socket 转发、不受 URCap 干扰。

用法:
    python3 ur_pick_place_full.py 1              # 1 轮：起点 -> 终点
    python3 ur_pick_place_full.py 1 --swap       # 反向：终点 -> 起点（把物体抓回来）
    python3 ur_pick_place_full.py 5 --pingpong   # 往返：每轮交换起点/终点，物体来回搬
    python3 ur_pick_place_full.py --forever
    Ctrl-C 随时停止（会停在当前位置，不松爪）

三种方向的区别:
    默认        : 每轮都从 pick 抓到 place（单向，物体最后总在 place）
    --swap      : 每轮都从 place 抓到 pick（反向，物体最后总在 pick）
    --pingpong  : 第1轮去、第2轮回、第3轮去…（物体一直在两点间来回，适合连续演示）
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rq_gripper import RobotiqGripper          # noqa: E402
from ur_arm import Arm                          # noqa: E402

H = 0.05          # 安全高度(米)
V = 0.05          # 速度 5cm/s（保守）


def _find_poses():
    """点位文件：优先脚本同级目录，其次 ~/ur_learn/poses.json"""
    here = os.path.dirname(os.path.abspath(__file__))
    for p in (os.path.join(here, "poses.json"),
              os.path.expanduser("~/ur_learn/poses.json")):
        if os.path.exists(p):
            return p
    return os.path.expanduser("~/ur_learn/poses.json")


POSES_FILE = _find_poses()

# ── 负载标定（2026-09-10 实测）──────────────────────────────
# 工具装配: 法兰 → ATI 力传感器 → [偏心 3D 相机] + [正装夹爪]
#   实测: 传感器下方工具(相机+夹爪) = 6.56 kg（力传感器 |F|=64.4N）
#   传感器自重: ~1.5 kg（估算，看铭牌确认）
#   重心: 测得的工具重心横向偏移 ≈ (-31, +29) mm（传感器系，来自偏心相机），
#         高度按安装尺寸估 60mm 下方；最终精确值建议用示教器「负载」向导校准
# 设 None 则不动负载（保持机器人当前值）
PAYLOAD_MASS = 8.0          # 6.56 + ~1.5
PAYLOAD_COG = [-0.03, 0.03, -0.06]   # tool0 系，米


def up(pose, h=H):
    """返回该位姿上方 h 米的位姿（保持姿态不变）"""
    return [pose[0], pose[1], pose[2] + h, pose[3], pose[4], pose[5]]


def main():
    args = sys.argv[1:]
    count = None if "--forever" in args else (int(args[0]) if args and args[0].isdigit() else 1)
    swap = "--swap" in args            # 反向：place -> pick
    pingpong = "--pingpong" in args    # 往返：每轮交换

    poses = json.load(open(POSES_FILE))
    pick, place = poses["pick"], poses["place"]

    print("=== UR3 完整抓放（ROS 管臂 + 电脑直控夹爪）===")
    print("抓取点A: [%s]" % ", ".join("%.4f" % x for x in pick[:3]))
    print("抓取点B: [%s]" % ", ".join("%.4f" % x for x in place[:3]))
    mode = "往返(每轮换向)" if pingpong else ("反向 B->A" if swap else "单向 A->B")
    print("模式: %s  安全高度 %.0fmm  速度 %.0fmm/s  %s"
          % (mode, H * 1000, V * 1000,
             "一直循环" if count is None else "%d 轮" % count))

    # ---- 连接两侧 ----
    arm = Arm(payload=PAYLOAD_MASS, cog=PAYLOAD_COG)
    g = RobotiqGripper()
    g.connect()
    st = g.status()
    print("夹爪: ACT=%s STA=%s FLT=%s POS=%s" % (st["ACT"], st["STA"], st["FLT"], st["POS"]))
    if st["FLT"] not in (0, None):
        print("⚠ 夹爪有故障码 FLT=%s，继续但请留意" % st["FLT"])
    if not g.activate():
        print("⚠ 夹爪激活未确认，继续尝试")

    cur = arm.get_tcp_pose()
    if cur:
        print("机械臂当前: [%s]" % ", ".join("%.4f" % x for x in cur[:3]))
    else:
        print("✘ 读不到机械臂位姿，机器人上电了吗？")
        return 1

    i = 0
    try:
        while count is None or i < count:
            i += 1

            # 本轮方向：默认 A->B；--swap 恒定 B->A；--pingpong 奇数轮 A->B、偶数轮 B->A
            reverse = (i % 2 == 0) if pingpong else swap
            src, dst = (place, pick) if reverse else (pick, place)
            src_up, dst_up = up(src), up(dst)
            print("\n=== 第 %d 轮  %s -> %s ==="
                  % (i, "B" if reverse else "A", "A" if reverse else "B"))

            print("① 移到取物点上方")
            arm.movel(src_up, v=V)
            print("② 下降到取物点")
            arm.movel(src, v=V)

            print("③ 夹爪闭合")
            g.close()
            obj = g.object_detected()
            pos = g.query("POS")
            print("   夹爪 POS=%s OBJ=%s %s"
                  % (pos, obj,
                     {2: "✔ 夹住物体了", 1: "张开时碰到物体",
                      3: "✘ 到位置但没夹到东西(物体不在位?)",
                      0: "运动中"}.get(obj, "")))
            if obj == 3:
                print("   ⚠ 空抓！下面这趟会白跑，建议停下检查物体位置")

            print("④ 抬起")
            arm.movel(src_up, v=V)
            print("⑤ 移到放物点上方")
            arm.movel(dst_up, v=V)
            print("⑥ 下降到放物点")
            arm.movel(dst, v=V)

            print("⑦ 夹爪张开（放下）")
            g.open()
            print("   OBJ=%s" % g.object_detected())

            print("⑧ 抬起离开")
            arm.movel(dst_up, v=V)

            print("✔ 第 %d 轮完成，物体现在在 %s 点"
                  % (i, "A" if reverse else "B"))
    except KeyboardInterrupt:
        print("\n[停止] Ctrl-C，机械臂停在当前位置，夹爪状态保持")
    finally:
        g.close_socket()
        print("[结束] 共完成 %d 轮" % i)
    return 0


if __name__ == "__main__":
    sys.exit(main())
