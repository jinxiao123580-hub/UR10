#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TCP（工具中心点）标定 —— 球拟合法

原理：
    让工具尖端（比如夹爪上一个固定尖点）去戳一个**固定点**（桌面上的尖针/记号），
    换多个姿态重复戳，每次记录 tool0 位姿 T_i。
    设 TCP 偏移（tool0 系下）为 t，固定点在 base 系为 P，则对每个姿态：
        P = R_i·t + d_i
    两两相减消去 P：(R_i − R_j)·t = d_j − d_i  → 最小二乘解出 t。

用法（需有人操作示教器/现场，机械臂旁手放急停）：
    python3 scripts/tcp_calib.py            # 交互式：每戳一次回车一次
    python3 scripts/tcp_calib.py --poses 6  # 指定采样数（默认 8）

流程：
    1) 机器人上电、松刹车、没在跑程序
    2) 用示教器点动/手动把工具尖端对准桌面固定尖点
    3) 回到电脑按回车记录姿态
    4) 换一个姿态（尽量转 60~90°，位置不变对准尖点），再回车
    5) 采样足够（≥6 个、姿态差异大）后自动算 TCP，打印结果

结果：
    t = [x, y, z]（tool0 系，米）—— 填到示教器「安装→TCP」，
    或写进 poses.json / URDF 的 tool0 偏置。
"""
import os
import socket
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ur_arm import read_packet          # noqa: E402


def solve_tcp(poses):
    """poses: list of 4x4 齐次矩阵（base→tool0）→ 返回 t (tool0 系 3 向量)"""
    n = len(poses)
    A, b = [], []
    for i in range(n):
        for j in range(i + 1, n):
            A.append(poses[i][:3, :3] - poses[j][:3, :3])
            b.append(poses[j][:3, 3] - poses[i][:3, 3])
    A, b = np.vstack(A), np.hstack(b)
    t, *_ = np.linalg.lstsq(A, b, rcond=None)
    return t


def main():
    args = sys.argv[1:]
    n_poses = 8
    if "--poses" in args:
        n_poses = int(args[args.index("--poses") + 1])

    print("=== TCP 球拟合标定 ===")
    print("步骤：用示教器点动/手推，把**工具尖端**对准桌面一个固定尖点，")
    print("      每次对准后回到电脑按回车记录。换姿态时**位置不变、角度大变**（60~90°）。")
    print("      ⚠️ 机械臂旁有人，手放急停。\n")

    poses = []
    prev_axis = None
    for k in range(1, n_poses + 1):
        input("  对准固定点后按回车（%d/%d）..." % (k, n_poses))
        p = read_packet()
        if not p:
            print("    ✘ 读不到位姿，重试")
            continue
        R = rotvec_to_R(p[3:])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p[:3]
        poses.append(T)
        # 简单提示姿态差异（用末端位置变化+旋转角）
        axis_angle = np.linalg.norm(p[3:])
        if prev_axis is not None and abs(axis_angle - prev_axis) < 0.3:
            print("    ⚠ 姿态变化较小，建议下次多转一些")
        prev_axis = axis_angle
        print("    ✔ 记录: pos=[%.4f, %.4f, %.4f]  旋转角=%.2f rad" % (p[0], p[1], p[2], axis_angle))

    if len(poses) < 4:
        print("✘ 有效采样不足（<4），不计算")
        return 1

    t = solve_tcp(poses)
    # 残差检查：把 t 代回去，看各姿态下的 P 是否一致
    Ps = [(T[:3, :3] @ t + T[:3, 3]) for T in poses]
    Ps = np.array(Ps)
    spread = Ps.max(axis=0) - Ps.min(axis=0)
    print("\n=== 结果 ===")
    print("  TCP 偏移（tool0 系）: t = [%.4f, %.4f, %.4f] m" % tuple(t))
    print("  残差检查：固定点在 base 系各采样应一致（差值越小越好）")
    print("    范围: [%.1f, %.1f, %.1f] mm" % tuple(spread * 1000))
    print("    若 >5mm：说明对准不准或姿态差异不够，重做并多采样")
    print("\n  填到示教器：安装 → TCP → 输入 X/Y/Z（米），然后确认")
    print("  或写进代码：poses.json 里的点位要减去这个偏移（抓取坐标是 tool0 系的）")
    return 0


def rotvec_to_R(v):
    """轴角 → 旋转矩阵"""
    x, y, z = v
    ang = np.linalg.norm(v)
    if ang < 1e-9:
        return np.eye(3)
    k = np.array(v) / ang
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


if __name__ == "__main__":
    sys.exit(main())
