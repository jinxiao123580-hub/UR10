#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模拟 Mech-Eye 相机 —— 没有相机（或没装 SDK）时，用它验证网页画面链路。

发布与真实 Mech-Eye 接口**同名同格式**的话题：
    /mechmind/color_image    sensor_msgs/Image  bgr8
    /mechmind/depth_map      sensor_msgs/Image  32FC1（米）
    /mechmind/point_cloud    sensor_msgs/PointCloud2（简单平面点云）

⚠️ 画面里会画上 "FAKE"，别当真相机数据。

用法:
    python3 scripts/fake_mecheye_publisher.py            # 2Hz
    python3 scripts/fake_mecheye_publisher.py --hz 5
    python3 scripts/fake_mecheye_publisher.py --size 1280x960
"""
import argparse
import math
import struct
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField


class FakeMechEye(Node):
    def __init__(self, w=640, h=480, hz=2.0):
        super().__init__("fake_mecheye")
        self.w, self.h = w, h
        self.pub_color = self.create_publisher(Image, "/mechmind/color_image", 2)
        self.pub_depth = self.create_publisher(Image, "/mechmind/depth_map", 2)
        self.pub_pcl = self.create_publisher(PointCloud2, "/mechmind/point_cloud", 2)
        self.t0 = time.time()
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            "模拟相机已启动：/mechmind/color_image + /mechmind/depth_map + /mechmind/point_cloud @%.1fHz (%dx%d)"
            % (hz, w, h))

    def _scene(self):
        t = time.time() - self.t0
        w, h = self.w, self.h
        # 彩色：渐变背景 + 一个绕圈的目标块（模拟被抓物）
        img = np.zeros((h, w, 3), np.uint8)
        grad = np.linspace(20, 90, w, dtype=np.uint8)
        img[:, :, 0] = grad[None, :]
        img[:, :, 1] = np.linspace(30, 70, h, dtype=np.uint8)[:, None]
        img[:, :, 2] = 60
        cx = int(w / 2 + (w / 4) * math.cos(t * 0.7))
        cy = int(h / 2 + (h / 5) * math.sin(t * 0.9))
        cv2.rectangle(img, (cx - 60, cy - 40), (cx + 60, cy + 40), (60, 200, 255), -1)
        cv2.putText(img, "FAKE CAMERA", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2)
        cv2.putText(img, "t=%.1fs" % t, (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (200, 220, 255), 1)

        # 深度：背景 1.2m，目标块 0.55m（可看出伪彩色差异）
        depth = np.full((h, w), 1.2, np.float32)
        depth[cy - 40:cy + 40, cx - 60:cx + 60] = 0.55
        return img, depth

    def tick(self):
        img, depth = self._scene()
        stamp = self.get_clock().now().to_msg()
        ci = Image()
        ci.header.stamp = stamp
        ci.header.frame_id = "mechmind_camera/color_map"
        ci.height, ci.width, ci.encoding, ci.step = self.h, self.w, "bgr8", self.w * 3
        ci.data = img.tobytes()
        self.pub_color.publish(ci)

        di = Image()
        di.header.stamp = stamp
        di.header.frame_id = "mechmind_camera/depth_map"
        di.height, di.width, di.encoding = self.h, self.w, "32FC1"
        di.step = self.w * 4
        di.data = depth.tobytes()
        self.pub_depth.publish(di)

        self.pub_pcl.publish(self._pcl(depth, stamp))

    def _pcl(self, depth, stamp):
        """把深度图变成点云（针孔模型，内参随便给一组）"""
        fx = fy = 600.0
        cx, cy = self.w / 2.0, self.h / 2.0
        step = 8                                    # 抽稀，别把带宽吃满
        ys, xs = np.mgrid[0:self.h:step, 0:self.w:step]
        z = depth[0:self.h:step, 0:self.w:step].astype(np.float32)
        x = (xs - cx) * z / fx
        y = (ys - cy) * z / fy
        pts = np.stack([x, -y, z], axis=-1).reshape(-1, 3)

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "mechmind_camera/point_cloud"
        msg.height, msg.width = 1, pts.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = pts.astype(np.float32).tobytes()
        return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--size", default="640x480")
    a = ap.parse_args()
    w, h = (int(x) for x in a.size.lower().split("x"))
    rclpy.init()
    n = FakeMechEye(w, h, a.hz)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
