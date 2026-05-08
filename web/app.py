#!/usr/bin/env python3
"""
RosNav Web Server  —  FIXED VERSION
Flask + SocketIO — controls robot via browser

FIXES vs original:
  1. ros2_pub() uses persistent ROS2 publisher thread — no more subprocess per cmd
  2. Flask no longer opens serial port (serial_bridge owns it exclusively)
  3. Teleop mode guard relaxed — commands work in teleop and idle modes
  4. SERIAL_PORT default corrected to /dev/ttyACM0
  5. Speed limits raised: linear ±1.0 m/s, angular ±2.5 rad/s
"""

import os
import json
import math
import time
import threading
import subprocess
import logging

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

# ── CONFIG ───────────────────────────────────────────────
SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyACM0")
BAUD_RATE   = int(os.environ.get("BAUD_RATE", "115200"))
ROS_SETUP   = "/opt/ros/humble/setup.bash"
_THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
WS_SETUP    = os.path.join(_THIS_DIR, "..", "ros2_ws", "install", "setup.bash")
WP_FILE     = "/tmp/rosnav_waypoints.json"

MAX_LINEAR  = 1.0
MAX_ANGULAR = 2.5

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("rosnav")

app = Flask(__name__)
app.config["SECRET_KEY"] = "rosnav-2024"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

state = {
    "mode": "idle", "connected": False, "waypoints": [], "wp_index": 0,
    "odom": {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "vz": 0.0},
    "imu":  {"ax": 0.0, "ay": 0.0, "gz": 0.0},
    "battery": 100.0, "cmd_vel": {"linear": 0.0, "angular": 0.0}, "last_cmd": 0.0,
}
lock = threading.Lock()
start_time = time.time()

WHEEL_RAD  = float(os.environ.get("WHEEL_RAD",  "0.0325"))
WHEEL_BASE = float(os.environ.get("WHEEL_BASE", "0.15"))
ENC_PPR    = int(os.environ.get("ENC_PPR",     "360"))

_ros2_cmd   = {"linear": 0.0, "angular": 0.0}
_ros2_lock  = threading.Lock()
_ros2_ready = False

def ros2_publisher_thread():
    global _ros2_ready
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        rclpy.init()
        node = Node("rosnav_web_publisher")
        pub  = node.create_publisher(Twist, "/cmd_vel", 10)
        _ros2_ready = True
        log.info("ROS2 publisher node started")
        while rclpy.ok():
            with _ros2_lock:
                lin = _ros2_cmd["linear"]
                ang = _ros2_cmd["angular"]
            msg = Twist()
            msg.linear.x  = float(lin)
            msg.angular.z = float(ang)
            pub.publish(msg)
            time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        log.error(f"ROS2 publisher thread failed: {e}")

def ros2_pub(linear, angular):
    with _ros2_lock:
        _ros2_cmd["linear"]  = linear
        _ros2_cmd["angular"] = angular
    with lock:
        state["cmd_vel"] = {"linear": linear, "angular": angular}
        state["last_cmd"] = time.time()

def ros2_run(node_name):
    cmd = (f"source {ROS_SETUP} 2>/dev/null; source {WS_SETUP} 2>/dev/null; "
           f"ros2 run rosnav_driver {node_name}")
    subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def ros2_topic_listener():
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        for _ in range(30):
            if _ros2_ready: break
            time.sleep(0.5)
        node = Node("rosnav_web_listener")

        def odom_cb(msg):
            with lock:
                state["odom"]["x"]  = round(msg.pose.pose.position.x, 4)
                state["odom"]["y"]  = round(msg.pose.pose.position.y, 4)
                q = msg.pose.pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
                state["odom"]["yaw"] = round(math.atan2(siny, cosy), 4)
                state["odom"]["vx"]  = round(msg.twist.twist.linear.x, 4)
                state["odom"]["vz"]  = round(msg.twist.twist.angular.z, 4)
                state["connected"] = True

        def imu_cb(msg):
            with lock:
                state["imu"]["ax"] = round(msg.linear_acceleration.x / 9.80665, 4)
                state["imu"]["ay"] = round(msg.linear_acceleration.y / 9.80665, 4)
                state["imu"]["gz"] = round(math.degrees(msg.angular_velocity.z), 4)

        node.create_subscription(Odometry, "/odom", odom_cb, 10)
        node.create_subscription(Imu,      "/imu",  imu_cb,  10)
        log.info("ROS2 topic listener started (/odom, /imu)")
        import rclpy.executors
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except Exception as e:
        log.error(f"ROS2 topic listener failed: {e}")

def telemetry_loop():
    while True:
        socketio.emit("telemetry", build_telemetry())
        time.sleep(0.1)

def build_telemetry():
    with lock:
        return {
            "odom": state["odom"].copy(), "imu": state["imu"].copy(),
            "battery": state["battery"], "mode": state["mode"],
            "connected": state["connected"], "wp_index": state["wp_index"],
            "uptime": int(time.time() - start_time), "cmd_vel": state["cmd_vel"].copy(),
        }

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/status")
def api_status():
    return jsonify(build_telemetry())

@app.route("/api/mode", methods=["POST"])
def api_mode():
    data = request.get_json(force=True)
    mode = data.get("mode", "idle")
    with lock:
        state["mode"] = mode
    if mode == "idle":
        ros2_pub(0.0, 0.0)
    elif mode == "teleop":
        ros2_pub(0.0, 0.0)
    elif mode == "waypoint":
        with lock:
            wps = state["waypoints"]
        with open(WP_FILE, "w") as f:
            json.dump(wps, f)
        ros2_pub(0.0, 0.0)
        ros2_run("waypoint_nav")
    socketio.emit("mode_change", {"mode": mode})
    log.info(f"Mode → {mode}")
    return jsonify({"status": "ok", "mode": mode})

@app.route("/api/waypoints", methods=["GET"])
def api_wp_get():
    with lock:
        return jsonify({"waypoints": state["waypoints"]})

@app.route("/api/waypoints", methods=["POST"])
def api_wp_set():
    data = request.get_json(force=True)
    clean = []
    for wp in data.get("waypoints", []):
        try:
            clean.append({"x": float(wp["x"]), "y": float(wp["y"])})
        except Exception:
            pass
    with lock:
        state["waypoints"] = clean
        state["wp_index"]  = 0
    log.info(f"Waypoints set: {len(clean)} points")
    return jsonify({"status": "ok", "count": len(clean)})

@app.route("/api/waypoints", methods=["DELETE"])
def api_wp_del():
    with lock:
        state["waypoints"] = []
        state["wp_index"]  = 0
    return jsonify({"status": "ok"})

@app.route("/api/odom/reset", methods=["POST"])
def api_odom_reset():
    with lock:
        state["odom"] = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "vz": 0.0}
    return jsonify({"status": "ok"})

@app.route("/api/serial/status")
def api_serial_status():
    return jsonify({"port": SERIAL_PORT, "exists": os.path.exists(SERIAL_PORT), "connected": state["connected"]})

@socketio.on("connect")
def on_connect():
    log.info(f"Browser connected: {request.sid}")
    emit("telemetry", build_telemetry())

@socketio.on("disconnect")
def on_disconnect():
    log.info(f"Browser disconnected: {request.sid}")

@socketio.on("teleop")
def on_teleop(data):
    with lock:
        current_mode = state["mode"]
    if current_mode not in ("teleop", "idle"):
        return
    linear  = max(-MAX_LINEAR,  min(MAX_LINEAR,  float(data.get("linear",  0.0))))
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, float(data.get("angular", 0.0))))
    ros2_pub(linear, angular)

@socketio.on("emergency_stop")
def on_estop(_=None):
    with lock:
        state["mode"] = "idle"
    ros2_pub(0.0, 0.0)
    socketio.emit("mode_change", {"mode": "idle"})
    log.warning("EMERGENCY STOP")

@socketio.on("request_telemetry")
def on_req_telem(_=None):
    emit("telemetry", build_telemetry())

def start_threads():
    threading.Thread(target=ros2_publisher_thread, daemon=True).start()
    threading.Thread(target=ros2_topic_listener,   daemon=True).start()
    threading.Thread(target=telemetry_loop,         daemon=True).start()

if __name__ == "__main__":
    log.info("=" * 50)
    log.info(f"  RosNav Web Server")
    log.info(f"  Serial : {SERIAL_PORT} @ {BAUD_RATE}  (owned by serial_bridge)")
    log.info(f"  Max speed: linear={MAX_LINEAR} m/s  angular={MAX_ANGULAR} rad/s")
    log.info(f"  Web UI : http://0.0.0.0:5000")
    log.info("=" * 50)
    start_threads()
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)
