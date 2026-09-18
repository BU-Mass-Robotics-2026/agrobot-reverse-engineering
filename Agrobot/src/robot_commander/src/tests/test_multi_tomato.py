#!/usr/bin/env python3
"""
Inject fake tomato pick targets directly to commander, bypassing tomato_picker.
Centroids are specified in linear_rail_link frame (the robot base frame).
Use this when the camera is not connected.

Usage:
    python3 src/robot_commander/src/test_multi_tomato.py
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
import numpy as np

# ---------------------------------------------------------------------------
# Tomato centroids in linear_rail_link frame (meters).
# x = along linear rail, y = forward from base, z = up from base
# Radius is the tomato sphere radius in meters.
# ---------------------------------------------------------------------------
TOMATOES = [
    {"x": 0.0, "y": 1.0, "z": 0.5, "radius": 0.04},
    {"x": 0.5, "y": 1.0, "z": 0.8, "radius": 0.04},
    {"x": 1.0, "y": 1.0, "z": 1.0, "radius": 0.04},
]

APPROACH_STANDOFF = 0.05  # 5 cm before tomato surface
RETRACT_STANDOFF  = 0.15  # 15 cm back after grasp


def approach_vector(pos):
    mag = math.sqrt(pos["x"]**2 + pos["y"]**2 + pos["z"]**2)
    return {"dx": pos["x"] / mag, "dy": pos["y"] / mag, "dz": pos["z"] / mag}


def get_waypoints(centroid, vec, radius):
    approach = {
        "x": centroid["x"] - vec["dx"] * (radius + APPROACH_STANDOFF),
        "y": centroid["y"] - vec["dy"] * (radius + APPROACH_STANDOFF),
        "z": centroid["z"] - vec["dz"] * (radius + APPROACH_STANDOFF),
    }
    retract = {
        "x": centroid["x"] - vec["dx"] * (radius + RETRACT_STANDOFF),
        "y": centroid["y"] - vec["dy"] * (radius + RETRACT_STANDOFF),
        "z": centroid["z"] - vec["dz"] * (radius + RETRACT_STANDOFF),
    }
    return [approach, centroid, retract]


def get_quaternion(vec):
    target  = np.array([vec["dx"], vec["dy"], vec["dz"]])
    current = np.array([0.0, 0.0, 1.0])
    rotation, _ = R.align_vectors([target], [current])
    return rotation.as_quat()  # [x, y, z, w]


class MultiTomatoTester(Node):
    def __init__(self):
        super().__init__('multi_tomato_tester')
        self.pick_pub   = self.create_publisher(PoseArray, '/agrobot/pick_targets', 10)
        self.safety_pub = self.create_publisher(Bool,      '/agrobot/safe_to_pick', 10)

    def run(self):
        self.get_logger().info('Waiting 2 s for subscribers...')
        time.sleep(2.0)

        safe_msg = Bool()
        safe_msg.data = True
        self.safety_pub.publish(safe_msg)
        self.get_logger().info('Published safe_to_pick = True')
        time.sleep(0.2)

        pick_msg = PoseArray()
        pick_msg.header.frame_id = 'linear_rail_link'
        pick_msg.header.stamp    = self.get_clock().now().to_msg()

        for t in TOMATOES:
            vec       = approach_vector(t)
            waypoints = get_waypoints(t, vec, t["radius"])
            quat      = get_quaternion(vec)

            for wp in waypoints:
                pose = Pose()
                pose.position.x    = wp["x"]
                pose.position.y    = wp["y"]
                pose.position.z    = wp["z"]
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                pick_msg.poses.append(pose)

        self.pick_pub.publish(pick_msg)
        self.get_logger().info(
            f'Published {len(TOMATOES)} tomatoes ({len(pick_msg.poses)} poses) '
            f'to /agrobot/pick_targets'
        )


def main():
    rclpy.init()
    node = MultiTomatoTester()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()