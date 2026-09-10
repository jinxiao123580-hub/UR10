#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ATI Net F/T 六维力/力矩传感器 → ROS 2

实测环境：ATI Net F/T（固件 2.0.12，SN 1106WNT072，标定 SI-165-15）
          网络盒 IP 192.168.1.2，HTTP 管理页在 80 端口。

原理：Net F/T 用 **RDT (Raw Data Transfer)** 通过 **UDP 49152** 推流——
      发 8 字节命令 `12 34 00 <采样数>` 开始，发 `12 34 00 00` 停止。

数据包（36 字节，9 个大端 int32，实测确认）:
    [0] rdt_sequence   每包 +1
    [1] ft_sequence    传感器内部采样序号（高速率递增）
    [2] status         状态字，0 = 健康；bit0..5 = Fx..Tz 饱和标志
    [3..8] Fx Fy Fz Tx Ty Tz   （单位 = counts，除以 counts_per_* 得物理量）

用法:
    # 命令行看数（不需要 ROS）
    python3 ati_netft_node.py --check
    python3 ati_netft_node.py --check --hz 10

    # ROS 2 节点：发布 /ft_sensor/wrench (geometry_msgs/WrenchStamped)
    source /opt/ros/humble/setup.bash
    python3 ati_netft_node.py
    python3 ati_netft_node.py --ros-args -p ip:=192.168.1.2 -p rate:=200

    # 置零（软件去皮，只影响本节点发布的值，不改传感器）
    ros2 service call /ft_sensor/tare std_srvs/srv/Trigger
"""
import argparse
import socket
import struct
import sys
import threading
import time

DEFAULT_IP = "192.168.1.2"
RDT_PORT = 49152
# 命令包：0x12 0x34 | 命令(0x00) | 采样数 | 保留 4 字节
CMD_START = bytes([0x12, 0x34, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00])
CMD_STOP = bytes([0x12, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

# 本机标定参数（来自 Net F/T 的 Configurations 页）
DEFAULT_COUNTS_PER_FORCE = 1_000_000.0
DEFAULT_COUNTS_PER_TORQUE = 1_000_000.0

STATUS_BITS = ["Fx 饱和", "Fy 饱和", "Fz 饱和", "Tx 饱和", "Ty 饱和", "Tz 饱和"]


def decode_status(word):
    """把状态字翻译成人话（bit0..5 = 六轴饱和）"""
    msgs = [name for i, name in enumerate(STATUS_BITS) if word & (1 << i)]
    return "、".join(msgs) if msgs else "正常"


class NetFT:
    """ATI Net F/T 的 RDT 客户端"""

    def __init__(self, ip=DEFAULT_IP, counts_per_force=DEFAULT_COUNTS_PER_FORCE,
                 counts_per_torque=DEFAULT_COUNTS_PER_TORQUE, samples=1):
        self.ip = ip
        self.cpf = counts_per_force
        self.cpt = counts_per_torque
        self.samples = max(0, min(255, samples))
        self.sock = None
        self.bias = [0.0] * 6          # 软件去皮偏移

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0)
        self.sock.sendto(CMD_START, (self.ip, RDT_PORT))

    def stop(self):
        if self.sock:
            try:
                self.sock.sendto(CMD_STOP, (self.ip, RDT_PORT))
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def read(self):
        """收一个包，返回 (wrench6, status_word, rdt_seq, ft_seq) 或 None

        wrench6 = [Fx, Fy, Fz, Tx, Ty, Tz]，单位 N / N·m，已去软件皮
        """
        if not self.sock:
            self.start()
        while True:
            data, _ = self.sock.recvfrom(2048)
            if len(data) < 36:
                continue
            rdt_seq, ft_seq, status = struct.unpack(">III", data[:12])
            raw = struct.unpack(">6i", data[12:36])
            w = [raw[0] / self.cpf - self.bias[0],
                 raw[1] / self.cpf - self.bias[1],
                 raw[2] / self.cpf - self.bias[2],
                 raw[3] / self.cpt - self.bias[3],
                 raw[4] / self.cpt - self.bias[4],
                 raw[5] / self.cpt - self.bias[5]]
            return w, status, rdt_seq, ft_seq

    def tare(self):
        """把当前读数当作零点（软件去皮，不改传感器本身）"""
        b = [0.0] * 6
        for _ in range(20):                      # 多次平均，降噪
            r = self.read()
            if r:
                for i in range(6):
                    b[i] += r[0][i] / 20.0
        self.bias = b
        return b


# ---------------- 命令行看数 ----------------
def check_mode(ip, hz, count):
    ft = NetFT(ip=ip)
    print("连接 ATI Net F/T %s (RDT UDP %d)..." % (ip, RDT_PORT))
    ft.start()
    print("  Fx(N)      Fy(N)      Fz(N)      Tx(Nm)     Ty(Nm)     Tz(Nm)   状态")
    n = 0
    t0 = time.time()
    try:
        while count is None or n < count:
            if hz:
                time.sleep(1.0 / hz)
            w, st, rs, fs = ft.read()
            n += 1
            print("  %9.3f %10.3f %10.3f %10.4f %10.4f %10.4f   %s"
                  % (*w, decode_status(st)))
    except KeyboardInterrupt:
        pass
    finally:
        ft.stop()
    if n:
        print("共 %d 包，用时 %.1fs（≈%.0f Hz）" % (n, time.time() - t0, n / max(1e-9, time.time() - t0)))


# ---------------- ROS 2 节点 ----------------
def ros_main(args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import WrenchStamped
    from std_srvs.srv import Trigger

    class NetFTNode(Node):
        def __init__(self):
            super().__init__("ati_netft")
            self.declare_parameter("ip", DEFAULT_IP)
            self.declare_parameter("rate", 200.0)
            self.declare_parameter("frame_id", "ft_sensor")
            self.declare_parameter("counts_per_force", DEFAULT_COUNTS_PER_FORCE)
            self.declare_parameter("counts_per_torque", DEFAULT_COUNTS_PER_TORQUE)

            ip = self.get_parameter("ip").value
            rate = float(self.get_parameter("rate").value)
            self.frame_id = self.get_parameter("frame_id").value

            self.ft = NetFT(ip=ip,
                            counts_per_force=self.get_parameter("counts_per_force").value,
                            counts_per_torque=self.get_parameter("counts_per_torque").value)
            self.ft.start()

            self.pub = self.create_publisher(WrenchStamped, "ft_sensor/wrench", 10)
            self.create_service(Trigger, "ft_sensor/tare", self._on_tare)
            self.create_timer(1.0 / rate, self._tick)
            self.get_logger().info(
                "ATI Net F/T @%s，发布 ft_sensor/wrench @%.0fHz，frame_id=%s"
                % (ip, rate, self.frame_id))

        def _on_tare(self, req, res):
            b = self.ft.tare()
            res.success = True
            res.message = "已去皮: " + ", ".join("%.3f" % v for v in b)
            self.get_logger().info(res.message)
            return res

        def _tick(self):
            try:
                r = self.ft.read()
            except socket.timeout:
                self.get_logger().warn("RDT 超时（检查 %s / RDT 是否启用）" % self.ft.ip, once=True)
                return
            except OSError as e:
                self.get_logger().error("RDT 错误: %s" % e)
                return
            if not r:
                return
            w, status, rdt_seq, ft_seq = r
            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = w[0], w[1], w[2]
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = w[3], w[4], w[5]
            self.pub.publish(msg)
            if status:
                self.get_logger().warn("传感器状态: %s (0x%08X)" % (decode_status(status), status),
                                       once=True)

    rclpy.init(args=args)
    node = NetFTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ft.stop()
        node.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="ATI Net F/T → ROS 2")
    ap.add_argument("--check", action="store_true", help="不启 ROS，直接在终端看数")
    ap.add_argument("--ip", default=DEFAULT_IP)
    ap.add_argument("--hz", type=float, default=10, help="--check 时打印频率")
    ap.add_argument("--count", type=int, default=None, help="--check 时打印多少行后退出")
    args, ros_args = ap.parse_known_args()

    if args.check:
        check_mode(args.ip, args.hz, args.count)
    else:
        ros_main(ros_args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
