#!/usr/bin/env python3
"""
RosNav — serial_bridge ROS2 node
STM32 <-> ROS2 bridge for 2-wheel robot

Subscribes : /cmd_vel  (geometry_msgs/Twist)
Publishes  : /odom     (nav_msgs/Odometry)
             /imu      (sensor_msgs/Imu)
             /tf       odom -> base_link
"""

import os
import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import serial

# ── CONFIG — override with env vars ──────────────────────
SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE   = int(os.environ.get("BAUD_RATE",  "115200"))
WHEEL_BASE  = float(os.environ.get("WHEEL_BASE", "0.15"))    # meters
WHEEL_RAD   = float(os.environ.get("WHEEL_RAD",  "0.0325"))  # meters
ENC_PPR     = int(os.environ.get("ENC_PPR",     "360"))      # pulses/rev


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub  = self.create_publisher(Imu,      "/imu",  10)
        self.tf_bc    = TransformBroadcaster(self)

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_cb, 10)

        # Robot pose
        self.x       = 0.0
        self.y       = 0.0
        self.yaw     = 0.0
        self.prev_el = None
        self.prev_er = None

        # Open serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
            self.get_logger().info(
                f"Serial bridge started  port={SERIAL_PORT}  baud={BAUD_RATE}")
        except serial.SerialException as e:
            self.get_logger().error(
                f"Cannot open {SERIAL_PORT}: {e}\n"
                "Fix: sudo chmod 666 /dev/ttyUSB0   OR   sudo usermod -aG dialout $USER")
            raise

        # Poll at 20 Hz
        self.create_timer(0.05, self._read_serial)
        self.get_logger().info(
            f"wheel_base={WHEEL_BASE}m  wheel_r={WHEEL_RAD}m  enc_ppr={ENC_PPR}")

    # /cmd_vel -> STM32 JSON ─────────────────────────────
    def _cmd_cb(self, msg: Twist):
        payload = json.dumps({
            "v": round(float(msg.linear.x),  3),
            "w": round(float(msg.angular.z), 3),
        }) + "\n"
        try:
            self.ser.write(payload.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial write error: {e}")

    # STM32 JSON -> ROS2 ─────────────────────────────────
    def _read_serial(self):
        if not self.ser.in_waiting:
            return
        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("{"):
                return
            data = json.loads(line)
            self._update_odom(data.get("el", 0), data.get("er", 0))
            self._pub_imu(
                data.get("ax", 0.0),
                data.get("ay", 0.0),
                data.get("gz", 0.0),
            )
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    # Odometry calculation ───────────────────────────────
    def _update_odom(self, el: int, er: int):
        if self.prev_el is None:
            self.prev_el, self.prev_er = el, er
            return

        dist_per_tick = 2.0 * math.pi * WHEEL_RAD / ENC_PPR
        dL  = (el - self.prev_el) * dist_per_tick
        dR  = (er - self.prev_er) * dist_per_tick
        self.prev_el, self.prev_er = el, er

        dc  = (dL + dR) / 2.0
        dth = (dR - dL) / WHEEL_BASE

        self.x   += dc * math.cos(self.yaw + dth / 2.0)
        self.y   += dc * math.sin(self.yaw + dth / 2.0)
        self.yaw += dth

        now = self.get_clock().now().to_msg()
        qz  = math.sin(self.yaw / 2.0)
        qw  = math.cos(self.yaw / 2.0)

        # Odometry message
        odom = Odometry()
        odom.header.stamp            = now
        odom.header.frame_id         = "odom"
        odom.child_frame_id          = "base_link"
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0]      = 0.001
        odom.pose.covariance[7]      = 0.001
        odom.pose.covariance[35]     = 0.001
        self.odom_pub.publish(odom)

        # TF broadcast
        tf = TransformStamped()
        tf.header.stamp            = now
        tf.header.frame_id         = "odom"
        tf.child_frame_id          = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self.tf_bc.sendTransform(tf)

    # IMU publish ────────────────────────────────────────
    def _pub_imu(self, ax_g: float, ay_g: float, gz_degs: float):
        imu = Imu()
        imu.header.stamp                     = self.get_clock().now().to_msg()
        imu.header.frame_id                  = "base_link"
        imu.linear_acceleration.x           = ax_g * 9.80665
        imu.linear_acceleration.y           = ay_g * 9.80665
        imu.angular_velocity.z              = math.radians(gz_degs)
        imu.linear_acceleration_covariance[0]  = 0.01
        imu.linear_acceleration_covariance[4]  = 0.01
        imu.angular_velocity_covariance[8]     = 0.01
        self.imu_pub.publish(imu)

    def destroy_node(self):
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
