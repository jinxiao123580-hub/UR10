#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR3 机械臂 —— 电脑经 ROS 控制（复用 ur_link 的 /ur_link/urscript 话题 → 30002）

提供:
    Arm.movel(pose, a, v)      绝对位姿直线移动（阻塞到走完）
    Arm.movej(joints)          关节移动
    Arm.get_tcp_pose()         读当前 TCP 位姿（走 30001 状态流）
    Arm.get_joints()           读当前关节角（走 /joint_states 话题）

单独运行可做自检:
    python3 ur_arm.py check             # 只读当前位姿/关节角
    python3 ur_arm.py nudge 0 0 -0.003  # 相对微动（默认 3mm，安全）
"""
import os
import socket
import struct
import sys
import threading
import time

ROBOT_IP = os.environ.get("UR_IP", "192.168.1.3")
SCRIPT_PORT = 30002
STATE_PORT = 30001


# ---------------- 姿态表示法转换（视觉抓取必用） ----------------
# UR 的 movel(p[x,y,z,rx,ry,rz]) 里 rx,ry,rz 是**轴角(rotation vector)**，
# 不是 RPY！TF 给的是四元数/RPY，直接塞进 movel 姿态必错。
import math                                                            # noqa: E402


def quat_to_rotvec(q):
    """四元数 (x,y,z,w) → 轴角 (rx,ry,rz)，可直接喂给 movel 的姿态部分"""
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    x, y, z, w = x / n, y / n, z / n, w / n
    if w < 0:                       # 取短弧
        x, y, z, w = -x, -y, -z, -w
    angle = 2.0 * math.acos(max(-1.0, min(1.0, w)))
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    return (angle * x / s, angle * y / s, angle * z / s)


def rotvec_to_quat(v):
    """轴角 (rx,ry,rz) → 四元数 (x,y,z,w)"""
    rx, ry, rz = [float(c) for c in v]
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2.0) / angle
    return (rx * s, ry * s, rz * s, math.cos(angle / 2.0))


def quat_to_rpy(q):
    """四元数 (x,y,z,w) → RPY（仅用于显示/比对，别喂给 movel）"""
    x, y, z, w = [float(v) for v in q]
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    return (roll, pitch, math.atan2(siny, cosy))


def rpy_to_quat(rpy):
    """RPY → 四元数 (x,y,z,w)"""
    r, p, y = [float(c) for c in rpy]
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


# ---------------- 状态读取（不依赖 ROS，走 30001 状态流） ----------------
def read_packet(host=ROBOT_IP, want_type=4, timeout=3.0):
    """从 30001 读一个子包(type=4 是 Cartesian info, 返回 TCP 位姿)"""
    s = socket.create_connection((host, STATE_PORT), timeout=3)
    s.settimeout(timeout)
    buf = b""
    t0 = time.time()
    result = None
    while time.time() - t0 < timeout and result is None:
        try:
            buf += s.recv(65536)
        except socket.timeout:
            break
        off = 0
        while off + 4 <= len(buf):
            size = struct.unpack(">I", buf[off:off + 4])[0]
            if not (5 <= size <= 100000) or off + size > len(buf):
                break
            f = buf[off:off + size]
            off += size
            if len(f) < 5 or f[4] != 16:
                continue
            p = 5
            while p + 5 <= len(f):
                sz = struct.unpack(">I", f[p:p + 4])[0]
                ty = f[p + 4]
                if sz < 5 or p + sz > len(f):
                    break
                if ty == 4 and sz - 5 >= 48:
                    result = struct.unpack(">dddddd", f[p + 5:p + 53])
                    break
                p += sz
        if off:
            buf = buf[off:]
    s.close()
    return result


# ---------------- 机械臂控制 ----------------
class Arm:
    """经 ROS 话题 /ur_link/urscript 控制（需要 ur_command_node 在跑）"""

    def __init__(self, ip=ROBOT_IP, payload=None, cog=None):
        """payload: 负载质量 kg（None=不设置，沿用机器人当前值）
        cog: 重心在 tool0 系 [x,y,z] 米（默认 [0,0,0.05]，仅在设 payload 时用）"""
        self.ip = ip
        self._node = None
        self._pub = None
        self.payload = payload
        self.cog = cog if cog is not None else [0.0, 0.0, 0.05]

    # --- ROS 初始化（惰性） ---
    def _ensure_ros(self):
        if self._pub is not None:
            return
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("ur_arm_client")
        self._pub = self._node.create_publisher(String, "/ur_link/urscript", 10)
        time.sleep(0.6)          # 等发现/连接建立，否则第一条会丢

    def send_script(self, script, call=True):
        """发 URScript。call=True 时把 def 包起来并显式调用（不调用不会动！）

        若构造时给了 payload，会在脚本前加 set_payload(...)——
        因为 URScript 程序每次运行会重置负载（示例程序里有 set_payload(0.01)），
        必须在我们自己的脚本里也设置，否则负载标定形同虚设。
        """
        from std_msgs.msg import String
        self._ensure_ros()
        if self.payload is not None:
            script = "set_payload(%r, [%r, %r, %r])\n" % (
                self.payload, self.cog[0], self.cog[1], self.cog[2]) + script
        if call:
            body = "\n".join("  " + ln for ln in script.splitlines())
            script = "def arm_cmd():\n%s\nend\narm_cmd()" % body
        msg = String()
        msg.data = script
        for _ in range(2):           # 首条消息可能因 DDS 发现丢失，补发一次
            self._pub.publish(msg)
            time.sleep(0.15)
        return script

    def movel(self, pose, a=0.3, v=0.05, wait=True, settle=0.4):
        px, py, pz, rx, ry, rz = [float(x) for x in pose]
        self.send_script(
            "movel(p[%r,%r,%r,%r,%r,%r], a=%r, v=%r)" % (px, py, pz, rx, ry, rz, a, v))
        if wait:
            self._wait_arrive(pose, settle)

    def movej(self, joints, a=1.0, v=0.5, wait=True):
        j = ", ".join(repr(float(x)) for x in joints)
        self.send_script("movej([%s], a=%r, v=%r)" % (j, a, v))
        if wait:
            time.sleep(1.0)

    def _wait_arrive(self, target, settle=0.4, timeout=60.0):
        """轮询 30001 直到 TCP 到达目标(容差 1mm / 0.02rad)"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.25)
            try:
                cur = read_packet()
            except Exception:
                continue
            if not cur:
                continue
            dp = max(abs(cur[i] - target[i]) for i in range(3))
            dr = max(abs(cur[i] - target[i]) for i in range(3, 6))
            if dp < 0.001 and dr < 0.02:
                time.sleep(settle)
                return True
        return False

    def get_tcp_pose(self):
        return read_packet()

    def stop_program(self):
        """经 dashboard 停止当前程序（急停注入的运动）"""
        s = socket.create_connection((self.ip, 29999), timeout=3)
        s.recv(4096)
        s.sendall(b"stop\n")
        time.sleep(0.3)
        s.close()


# ---------------- 自检 ----------------
def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    arm = Arm()

    if cmd == "check":
        p = arm.get_tcp_pose()
        if p:
            print("当前 TCP 位姿: [%s]" % ", ".join("%.4f" % x for x in p))
        else:
            print("✘ 读不到 TCP 位姿（机器人没上电/网络不通？）")

    elif cmd == "nudge":
        offs = [float(x) for x in sys.argv[2:5]]
        cur = arm.get_tcp_pose()
        if not cur:
            print("✘ 读不到当前位姿")
            return 1
        target = list(cur)
        for i in range(3):
            target[i] += offs[i]
        print("当前: [%s]" % ", ".join("%.4f" % x for x in cur[:3]))
        print("目标: [%s]  (偏移 %s)" % (" ".join("%.4f" % x for x in target[:3]), offs))
        script = arm.send_script(
            "movel(p[%r,%r,%r,%r,%r,%r], a=0.3, v=0.05)" % tuple(target))
        print("已发送:\n" + script)
        time.sleep(3)
        after = arm.get_tcp_pose()
        if after:
            d = sum(abs(after[i] - cur[i]) for i in range(3))
            print("移动后: [%s]  位移 %.4fm %s"
                  % (" ".join("%.4f" % x for x in after[:3]), d,
                     "✔ 动了" if d > 0.0005 else "✘ 没动"))
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
