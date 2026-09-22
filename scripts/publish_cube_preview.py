#!/usr/bin/env python3
"""Publish the accepted 5 cm cube measurement as RViz-only markers.

No robot, gripper, or camera command is sent.  The markers are deliberately a
scene-alignment aid, not a motion plan or an authorization to grasp.
"""
import argparse
import json
import os


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--measurement",
                        default="outputs/vision/cube-measurement-20260921-provisional-r2.json")
    parser.add_argument("--hover-plan", default="outputs/vision/fingertip-hover-plan-20260922.json",
                        help="optional non-executable fingertip hover plan to visualize")
    parser.add_argument("--frame", default="world")
    args = parser.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = args.measurement if os.path.isabs(args.measurement) else os.path.join(root, args.measurement)
    with open(path, encoding="utf-8") as stream:
        measurement = json.load(stream)
    if measurement.get("status") != "measured":
        raise SystemExit("cube measurement is not accepted: %s" % measurement.get("status"))
    if not measurement.get("size_prior", {}).get("matches_nominal_size"):
        raise SystemExit("cube size gate did not pass")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import Marker, MarkerArray

    center = measurement["center_base_m"]
    edges = measurement["top_face"]["footprint"]["edges_m"]
    height = measurement["measured_height_m"]
    rclpy.init()
    node = Node("cube_grasp_preview")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(MarkerArray, "/vision_grasp_preview", qos)

    now = node.get_clock().now().to_msg()
    cube = Marker()
    cube.header.frame_id, cube.header.stamp = args.frame, now
    cube.ns, cube.id, cube.type, cube.action = "cube_measurement", 0, Marker.CUBE, Marker.ADD
    cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = map(float, center)
    cube.pose.orientation.w = 1.0
    cube.scale.x, cube.scale.y, cube.scale.z = float(edges[0]), float(edges[1]), float(height)
    cube.color.r, cube.color.g, cube.color.b, cube.color.a = 1.0, 0.42, 0.04, 0.42

    point = Marker()
    point.header.frame_id, point.header.stamp = args.frame, now
    point.ns, point.id, point.type, point.action = "cube_center", 1, Marker.SPHERE, Marker.ADD
    point.pose.position.x, point.pose.position.y, point.pose.position.z = map(float, center)
    point.pose.orientation.w = 1.0
    point.scale.x = point.scale.y = point.scale.z = 0.014
    point.color.r, point.color.g, point.color.b, point.color.a = 1.0, 0.05, 0.05, 1.0

    label = Marker()
    label.header.frame_id, label.header.stamp = args.frame, now
    label.ns, label.id, label.type, label.action = "cube_center", 2, Marker.TEXT_VIEW_FACING, Marker.ADD
    label.pose.position.x, label.pose.position.y = float(center[0]), float(center[1])
    label.pose.position.z = float(center[2]) + height / 2.0 + 0.025
    label.pose.orientation.w, label.scale.z = 1.0, 0.025
    label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 0.85, 0.1, 1.0
    label.text = "measured 50 mm cube (preview only)"
    markers = [cube, point, label]
    hover_path = (args.hover_plan if os.path.isabs(args.hover_plan)
                  else os.path.join(root, args.hover_plan))
    if os.path.isfile(hover_path):
        with open(hover_path, encoding="utf-8") as stream:
            hover = json.load(stream)
        if hover.get("kind") in ("left_fingertip_hover_preview",
                                 "gripper_grasp_center_hover_preview"):
            location = hover["fingertip_hover_base_m"]
            target = Marker()
            target.header.frame_id, target.header.stamp = args.frame, now
            target.ns, target.id, target.type, target.action = "fingertip_hover", 3, Marker.SPHERE, Marker.ADD
            target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, location)
            target.pose.orientation.w = 1.0
            target.scale.x = target.scale.y = target.scale.z = 0.018
            target.color.r, target.color.g, target.color.b, target.color.a = 0.05, 0.85, 1.0, 1.0
            markers.append(target)
    publisher.publish(MarkerArray(markers=markers))
    node.get_logger().info("published cube preview from %s" % path)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
