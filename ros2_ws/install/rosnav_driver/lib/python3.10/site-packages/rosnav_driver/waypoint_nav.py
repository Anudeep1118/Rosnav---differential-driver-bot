#!/usr/bin/env python3
"""
RosNav — waypoint_nav ROS2 node
Drives the robot through a list of (x,y) waypoints using odometry.

Waypoints loaded from /tmp/rosnav_waypoints.json
Format: [{"x":1.0,"y":0.0}, {"x":1.0,"y":1.0}]
"""

import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WP_FILE = "/tmp/rosnav_waypoints.json"

# Controller tuning
GOAL_TOL    = 0.07   # meters — waypoint reached distance
MAX_LIN     = 0.30   # m/s
MAX_ANG     = 1.0    # rad/s
K_LIN       = 0.5    # proportional gain for distance
K_ANG       = 0.9    # proportional gain for heading error


class WaypointNav(Node):
    def __init__(self):
        super().__init__("waypoint_nav")

        self.cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10)

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.wp_idx = 0
        self.active = True

        self.waypoints = self._load_waypoints()
        if not self.waypoints:
            self.get_logger().error(
                f"No waypoints found in {WP_FILE}. "
                "Send waypoints via the web UI first.")
            self.active = False
        else:
            self.get_logger().info(
                f"Loaded {len(self.waypoints)} waypoints")

        self.create_timer(0.1, self._navigate)

    def _load_waypoints(self):
        try:
            with open(WP_FILE) as f:
                data = json.load(f)
            wps = []
            for item in data:
                if isinstance(item, dict):
                    wps.append((float(item["x"]), float(item["y"])))
                elif isinstance(item, (list, tuple)) and len(item) >= 2:
                    wps.append((float(item[0]), float(item[1])))
            return wps
        except FileNotFoundError:
            self.get_logger().warn(f"{WP_FILE} not found")
            return []
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return []

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        self.yaw = math.atan2(siny, cosy)

    def _navigate(self):
        twist = Twist()

        if not self.active or self.wp_idx >= len(self.waypoints):
            self.cmd_pub.publish(twist)
            return

        gx, gy = self.waypoints[self.wp_idx]
        dx  = gx - self.x
        dy  = gy - self.y
        dist  = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        err_a = math.atan2(
            math.sin(angle - self.yaw),
            math.cos(angle - self.yaw))

        if dist < GOAL_TOL:
            self.get_logger().info(
                f"Reached WP {self.wp_idx + 1}/{len(self.waypoints)} "
                f"({gx:.2f}, {gy:.2f})")
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints reached — stopping.")
                self.active = False
        elif abs(err_a) > 0.20:
            # Rotate in place to face goal
            twist.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * err_a))
        else:
            # Drive toward goal
            twist.linear.x  = min(MAX_LIN, K_LIN * dist)
            twist.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * err_a))

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = WaypointNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
