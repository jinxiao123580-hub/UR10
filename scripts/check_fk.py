#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FK/TF 自检 —— 视觉抓取前必做的一步。

做两件事：
  1) 比对 ROS 里的 FK 结果（TF base→tool0）与机器人自己报的 TCP 位姿（30001），
     确认 URDF + /joint_states 这条链是可信的；
  2) **指出 base 与 base_link 的区别** —— 这是视觉抓取最容易踩的坑：
     UR 的 URDF 里 base_link 相对 base 绕 Z 转了 180°，
     机器人自报的 TCP 位姿用的是 base 系。
     **如果算出来的目标点误用 base_link 系，机械臂会跑到反方向去抓。**

用法（需已起 ur_state_node 和 robot_state_publisher，见 scripts/start_ur_tf.sh）:
    python3 scripts/check_fk.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ur_arm import read_packet, quat_to_rpy, quat_to_rotvec   # noqa: E402


def main():
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
    from rclpy.duration import Duration

    rclpy.init()
    node = Node("check_fk")
    buf = Buffer()
    TransformListener(buf, node)

    # 等 TF 树建起来
    print("等待 TF 树...")
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.25)
        if buf.can_transform("base", "tool0", rclpy.time.Time()):
            break
    else:
        print("✘ 等不到 TF: base → tool0")
        print("  检查：① ur_state_node 在发 /joint_states 吗（ros2 topic hz /joint_states）")
        print("        ② start_ur_tf.sh 起了 robot_state_publisher 吗")
        node.destroy_node(); rclpy.shutdown()
        return 1

    def look(target):
        t = buf.lookup_transform(target, "tool0", rclpy.time.Time(),
                                 timeout=Duration(seconds=2.0))
        tr = t.transform.translation
        rot = t.transform.rotation
        q = [rot.x, rot.y, rot.z, rot.w]
        # 注意：UR 的 movel 要的是轴角，这里两种表示都算出来
        return [tr.x, tr.y, tr.z], quat_to_rpy(q), quat_to_rotvec(q)

    pos_base, rpy_base, rv_base = look("base")
    pos_bl, rpy_bl, _ = look("base_link")

    real = read_packet()

    print("\n=== ① TF 算出来的末端位姿 ===")
    print("  base      → tool0:")
    print("     位置 [%8.4f, %8.4f, %8.4f]" % tuple(pos_base))
    print("     姿态 RPY      = [%6.3f, %6.3f, %6.3f]   (给人看的)"
          % tuple(rpy_base))
    print("     姿态 轴角(UR) = [%6.3f, %6.3f, %6.3f]   (能直接喂 movel)"
          % tuple(rv_base))
    print("  base_link → tool0: [%8.4f, %8.4f, %8.4f]"
          % tuple(pos_bl))

    print("\n=== ② 机器人自报的 TCP 位姿（30001，真值）===")
    if real:
        print("     位置 [%8.4f, %8.4f, %8.4f]" % tuple(real[:3]))
        print("     姿态 轴角   = [%6.3f, %6.3f, %6.3f]" % tuple(real[3:]))
    else:
        print("  ✘ 读不到（30001 不通 / 机器人没上电）")

    print("\n=== ③ 结论 ===")
    if real:
        d_base = max(abs(pos_base[i] - real[i]) for i in range(3))
        d_bl = max(abs(pos_bl[i] - real[i]) for i in range(3))
        print("  base      → tool0 与真值最大偏差: %.4f m" % d_base)
        print("  base_link → tool0 与真值最大偏差: %.4f m" % d_bl)
        if d_base < d_bl:
            print("  ✔ base 系对得上 → **视觉目标请变换到 base 系**")
        else:
            print("  ⚠ base_link 反而更接近？检查 URDF 的 base 定义")
        if d_base < 0.01:
            print("  ✔ FK 链可信（<10mm，残差主要来自标称 URDF 与出厂标定的差异）")
        elif d_base < 0.05:
            print("  △ 偏差 %dmm —— 标称 URDF 与出厂标定有差异，"
                  "做精细抓取前建议用真值修正" % round(d_base * 1000))
        else:
            print("  ✘ 偏差过大，检查 /joint_states 是否滞后或 URDF 机型是否选错")

        # 姿态比对（同样要转成同一表示法才能比）
        d_rot = max(abs(rv_base[i] - real[3 + i]) for i in range(3))
        print("  姿态偏差（轴角）: %.4f rad %s"
              % (d_rot, "✔ 一致" if d_rot < 0.02 else "△ 有差异，检查 joint_states 时序"))

        # base_link 镜像检查
        if (abs(pos_base[0] + pos_bl[0]) < 0.02 and abs(pos_base[1] + pos_bl[1]) < 0.02):
            print("  ⚠ base_link = base 绕 Z 转 180°（X/Y 镜像）—— 这就是那个坑，"
                  "用错坐标系会跑到反方向")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
