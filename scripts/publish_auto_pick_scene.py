#!/usr/bin/env python3
"""Publish detected cube and checkerboard as standard MoveIt collision objects.

This is visualization/planning-scene input only.  It publishes no trajectory
and does not connect to a UR motion port.  The objects are in the same base
frame used by the camera localisation JSON.
"""
import argparse
import json
import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def quaternion(rotation):
    vector, _ = cv2.Rodrigues(np.asarray(rotation, dtype=float))
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    axis = vector.reshape(3) / angle
    return (*list(axis * np.sin(angle / 2.0)), float(np.cos(angle / 2.0)))


def box(name, frame, center, size, rotation=np.eye(3)):
    message = CollisionObject()
    message.id, message.header.frame_id = name, frame
    primitive = SolidPrimitive(); primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(map(float, size))
    pose = Pose(); pose.position.x, pose.position.y, pose.position.z = map(float, center)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion(rotation)
    message.primitives.append(primitive); message.primitive_poses.append(pose)
    message.operation = CollisionObject.ADD
    return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observation", default="outputs/vision/cube-board-observation.json")
    parser.add_argument("--frame", default="base")
    parser.add_argument("--board-size-m", type=float, nargs=2, default=[0.060, 0.042])
    parser.add_argument("--board-thickness-m", type=float, default=0.003)
    parser.add_argument("--cube-edge-m", type=float, default=0.050)
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()
    path = args.observation if os.path.isabs(args.observation) else os.path.join(ROOT, args.observation)
    with open(path, encoding="utf-8") as stream: observation = json.load(stream)
    cube = np.asarray(observation["estimated_cube_center_base_m"], dtype=float)
    board = np.asarray(observation["board_center_base_m"], dtype=float)
    normal = np.asarray(observation["board_normal_base"], dtype=float); normal /= np.linalg.norm(normal)
    x_axis = np.asarray(observation["board_x_axis_base"], dtype=float)
    x_axis -= normal * float(x_axis @ normal); x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    board_rotation = np.column_stack((x_axis, y_axis, normal))
    # Centre the thin board primitive on its measured plane rather than putting
    # its top face at the plane; collision geometry then occupies both sides by
    # half its known thickness.
    board_center = board - normal * args.board_thickness_m / 2.0
    scene = PlanningScene(); scene.is_diff = True
    scene.world.collision_objects = [
        box("detected_cube_50mm", args.frame, cube,
            [args.cube_edge_m] * 3),
        box("checkerboard", args.frame, board_center,
            [args.board_size_m[0], args.board_size_m[1], args.board_thickness_m],
            board_rotation),
    ]
    rclpy.init(); node = Node("publish_auto_pick_scene")
    publisher = node.create_publisher(PlanningScene, "/planning_scene", 10)
    deadline = node.get_clock().now().nanoseconds + int(args.seconds * 1e9)
    while node.get_clock().now().nanoseconds < deadline:
        publisher.publish(scene); rclpy.spin_once(node, timeout_sec=0.1)
    print("published PlanningScene: detected_cube_50mm + checkerboard in", args.frame)
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
