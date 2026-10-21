#!/usr/bin/env python3
"""Display-only causal first-order low-pass; raw/protection topics unchanged."""
import math
import time
import argparse


class LowPass:
    def __init__(self, cutoff_hz=5.0):
        if not math.isfinite(cutoff_hz) or cutoff_hz <= 0:
            raise ValueError('cutoff must be positive')
        self.tau = 1 / (2 * math.pi * cutoff_hz)
        self.last = None
        self.value = None

    def update(self, values, stamp):
        if len(values) != 6 or not all(math.isfinite(v) for v in values) or not math.isfinite(stamp):
            return None
        dt = None if self.last is None else stamp - self.last
        if dt is not None and dt <= 0:
            return None
        if dt is None or dt > .25:
            self.value = list(values)
        else:
            alpha = -math.expm1(-dt / self.tau)
            self.value = [old + alpha * (new - old) for old, new in zip(self.value, values)]
        self.last = stamp
        return self.value[:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cutoff-hz', type=float, default=5.0)
    args = parser.parse_args()
    filt = LowPass(args.cutoff_hz)
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    rclpy.init()
    node = rclpy.create_node('wrench_display_lowpass')
    pub = node.create_publisher(WrenchStamped, '/ft_sensor/wrench_compensated_filtered', 10)
    last_receipt = [None]

    def callback(msg):
        now = time.monotonic()
        age = node.get_clock().now().nanoseconds / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if not math.isfinite(age) or age < -.1 or age > .3:
            return
        if last_receipt[0] is not None and now - last_receipt[0] > .25:
            filt.last = None
        last_receipt[0] = now
        w = msg.wrench
        value = filt.update([w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z],
                            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if value is None:
            return
        out = WrenchStamped()
        out.header = msg.header
        out.wrench.force.x,out.wrench.force.y,out.wrench.force.z = value[:3]
        out.wrench.torque.x,out.wrench.torque.y,out.wrench.torque.z = value[3:]
        pub.publish(out)

    sub = node.create_subscription(WrenchStamped, '/ft_sensor/wrench_compensated', callback, 10)
    node.get_logger().info('显示专用低通 %.1f Hz；不改原始力、保护或控制；间断重置，不补造数据' % args.cutoff_hz)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
