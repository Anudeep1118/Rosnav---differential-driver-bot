# RosNav

<!-- Badges -->
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red?logo=raspberry-pi)
![MCU](https://img.shields.io/badge/MCU-STM32F103C6-03234B?logo=stmicroelectronics)
![Firmware](https://img.shields.io/badge/Firmware-v2.1-orange)
![License](https://img.shields.io/badge/License-MIT-green)
![Python](https://img.shields.io/badge/Python-3.10%2B-yellow?logo=python)

> Autonomous ground robot navigation stack — ROS2 Humble on Raspberry Pi 4, with an STM32F103 handling real-time motor control, quadrature encoder counting, and MPU6050 IMU over a JSON serial protocol. Includes a live web dashboard with telemetry, teleop, and waypoint mission control.

---

## Overview

RosNav is a compact, deployable differential-drive navigation system. The Raspberry Pi runs the ROS2 stack and Nav2 planner while the STM32 ("Blue Pill") handles everything time-critical: PWM generation, encoder interrupts at 20 Hz, IMU reads over I2C, and a 1-second watchdog that cuts motors if the Pi goes silent.

Communication between the two boards is a lightweight JSON-over-UART protocol at 115200 baud:

```
Pi → STM32 :  {"v": 0.30, "w": 0.50}
STM32 → Pi :  {"el": 120, "er": 118, "ax": 0.01, "ay": -0.02, "gz": 1.5}
```

The `serial_bridge` ROS2 node translates these packets into `/odom`, `/imu`, and `/tf` for the rest of the stack. A Flask + SocketIO web server provides a live dashboard and accepts teleop commands or waypoint missions from any browser on the same network.

**Hardware:**
- Raspberry Pi 4 (Ubuntu 22.04 + ROS2 Humble)
- STM32F103C6 (Blue Pill, 32KB flash)
- L298N dual motor driver
- Quadrature encoders (360 PPR)
- MPU6050 IMU (I2C — SDA→PB7, SCL→PB6)

---

## Features

- 🤖 **ROS2 Humble** navigation with Nav2 and `robot_localization`
- ⚡ **STM32 firmware v2.1** — hard real-time PWM, encoder ISRs, 20 Hz telemetry, 1-second safety watchdog
- 📡 **JSON serial bridge** — bidirectional ROS2 ↔ STM32 over UART at 115200 baud
- 🗺️ **Proportional waypoint navigation** — odometry-based point-to-point with heading correction
- 📊 **Live web dashboard** — real-time odometry, IMU, velocity, uptime, teleop joystick, waypoint manager
- 🔁 **Systemd services** — `rosnav-bridge` and `rosnav-web` auto-start on boot
- 🔧 **Hardware checker** — validates serial port, MPU6050, Python packages, and ROS2 before first launch
- ⚙️ **Env-variable calibration** — tune wheel geometry and serial config without touching source

---

## Architecture

```
┌────────────────────────────────────────────────────────┐
│                    Raspberry Pi 4                       │
│                                                        │
│  ┌─────────────────┐     ┌──────────────────────────┐  │
│  │  serial_bridge   │◄───►│      ROS2 / Nav2         │  │
│  │  (ROS2 node)     │     │  /odom  /imu  /cmd_vel   │  │
│  │                  │     └────────────┬─────────────┘  │
│  │  Pub: /odom      │                  │                 │
│  │       /imu       │     ┌────────────▼─────────────┐  │
│  │       /tf        │     │  Flask + SocketIO         │  │
│  │  Sub: /cmd_vel   │     │  Web dashboard (port 5000)│  │
│  └────────┬─────────┘     │  Teleop / Waypoints / HUD │  │
│           │ UART JSON      └──────────────────────────┘  │
│           │ 115200 baud                                  │
└───────────┼────────────────────────────────────────────┘
            │
   ┌────────▼──────────────┐
   │     STM32F103C6        │
   │                       │
   │  Motor PWM  PA0, PA1  │
   │  Dir pins   PB0/PB1   │
   │             PA8/PA9   │
   │  Enc ISR    PA6, PB9  │
   │  MPU6050    PB6/PB7   │
   │  Watchdog   1000 ms   │
   │  Rate       20 Hz     │
   └────────┬──────────────┘
            │
   ┌────────▼──────────────┐
   │  L298N + Motors        │
   │  + Quadrature Encoders │
   └───────────────────────┘
```

The waypoint navigator runs as a separate `waypoint_nav` ROS2 node. When waypoint mode is activated from the web UI, the Flask server writes the mission to `/tmp/rosnav_waypoints.json` and spawns the node. It uses a proportional controller — rotate in place to face the goal, then drive with heading correction, advance when within 7 cm.

---

## Wiring Reference

| Signal | STM32 Pin | Notes |
|---|---|---|
| L298N ENA (left speed) | PA0 | PWM |
| L298N IN1 / IN2 | PB0 / PB1 | Left direction |
| L298N ENB (right speed) | PA1 | PWM |
| L298N IN3 / IN4 | PA8 / PA9 | Right direction |
| Left encoder A | PA6 | Interrupt |
| Left encoder B | PA7 | Quadrature sense |
| Right encoder A | PB9 | Interrupt |
| Right encoder B | PB8 | Quadrature sense |
| MPU6050 SDA | PB7 | 4.7 kΩ pull-up to 3.3 V required |
| MPU6050 SCL | PB6 | 4.7 kΩ pull-up to 3.3 V required |
| MPU6050 AD0 | GND | Sets I2C address to 0x68 |

All grounds (STM32, L298N, encoders, MPU6050) must share a common GND.

**PWM dead-zone:** `MIN_PWM` defaults to 140 (out of 255). Raise toward 160 if motors stall at low speed; lower toward 100 if startup is jerky.

---

## Project Structure

```
rosnav/
├── install.sh                        # One-time setup script
├── start.sh                          # Manual launch
├── requirements.txt                  # Python dependencies
├── firmware/
│   └── rosnav_firmware/
│       └── rosnav_firmware.ino       # STM32 firmware v2.1 — flash via Arduino IDE
├── ros2_ws/
│   └── src/
│       └── rosnav_driver/
│           ├── package.xml
│           ├── setup.py
│           └── rosnav_driver/
│               ├── serial_bridge.py  # JSON bridge, odometry, IMU pub, TF broadcast
│               └── waypoint_nav.py  # Proportional waypoint navigation node
├── web/
│   ├── app.py                        # Flask + SocketIO server (port 5000)
│   └── templates/
│       └── index.html               # Web dashboard UI
└── scripts/
    └── check_hardware.py            # Pre-flight hardware check
```

---

## Installation & Deployment

### Prerequisites

- Raspberry Pi 4 running **Ubuntu 22.04 Server**
- Arduino IDE on your PC for flashing the STM32
- STM32F103C6 connected via USB (CH340, CP2102, or ST-Link adapter)
- Pi and your PC on the same network

---

### Step 1 — Copy files to the Pi

From your development machine:

```bash
scp -r rosnav/ pi@<raspberry-pi-ip>:~/rosnav
```

Or clone directly on the Pi if the repo is hosted remotely.

---

### Step 2 — Flash STM32 firmware

1. Open **Arduino IDE** on your PC
2. Go to **File → Preferences** and add the board manager URL:
   ```
   https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
   ```
3. Open **Tools → Board Manager**, search `STM32`, install **STM32 MCU based boards**
4. Open `firmware/rosnav_firmware/rosnav_firmware.ino`
5. Edit the robot parameters near the top to match your build:
   ```cpp
   #define WHEEL_BASE    0.15f   // center-to-center wheel distance (meters)
   #define MAX_SPEED_MS  0.57f   // max wheel speed — adjust for your motors
   #define MIN_PWM       140     // raise if motors stall; lower if start is jerky
   ```
6. Select **Tools → Board → STM32 → Generic STM32F1 series → Generic STM32F103C6**
7. Upload via USB

On successful boot the STM32 sends `{"status":"READY","imu":true}` (or `false` if MPU6050 wasn't found). You can verify this in the Arduino Serial Monitor at 115200 baud before moving to the Pi side.

---

### Step 3 — Install software on the Pi

SSH in — Ubuntu 22.04 required:

```bash
ssh pi@<raspberry-pi-ip>
cd ~/rosnav
chmod +x install.sh start.sh
./install.sh
```

`install.sh` runs the full setup in order:

1. Installs system packages and enables I2C / UART in `/boot/firmware/config.txt`
2. Removes `console=serial0` from `cmdline.txt` so UART is free for STM32
3. Adds udev rules for CH340, CP2102, and ST-Link → STM32 reliably appears as `/dev/rosnav_serial`
4. Installs **ROS2 Humble** from the official APT repository
5. Installs **Nav2**, `robot_localization`, and TF2 tools
6. Installs Python packages: Flask 3.0, flask-socketio 5.3, pyserial 3.5, eventlet
7. Builds the `rosnav_driver` workspace with `colcon --symlink-install`
8. Appends ROS2 source lines and env vars to `~/.bashrc`
9. Registers and enables `rosnav-bridge.service` and `rosnav-web.service`

When it finishes:

```bash
sudo reboot
```

---

### Step 4 — Verify hardware

After reboot, run the pre-flight check:

```bash
cd ~/rosnav
python3 scripts/check_hardware.py
```

This checks: Python packages ✓, serial port accessible ✓, STM32 responding with live JSON ✓, MPU6050 at I2C 0x68 ✓, ROS2 sourced and `rosnav_driver` package present ✓.

The script exits non-zero if any hard failures are found. Fix everything marked `[FAIL]` before continuing.

---

### Step 5 — Launch

**Auto-start (default after install):**

Services start on every boot. Check status with:

```bash
sudo systemctl status rosnav-bridge.service
sudo systemctl status rosnav-web.service
```

**Manual launch (single script):**

```bash
cd ~/rosnav
./start.sh
```

**Manual launch (separate terminals — useful for debugging):**

```bash
# Terminal 1 — serial bridge
source ~/.bashrc
ros2 run rosnav_driver serial_bridge

# Terminal 2 — web server
cd ~/rosnav/web
python3 app.py

# Terminal 3 — topic monitor
ros2 topic echo /odom
ros2 topic echo /imu
ros2 topic echo /cmd_vel
```

---

### Step 6 — Open the web dashboard

Get the Pi's IP:

```bash
hostname -I
```

Open in any browser on the same network:

```
http://<raspberry-pi-ip>:5000
```

The dashboard shows live odometry (x, y, yaw), IMU readings, current velocity, uptime, connection status, and operating mode. Drive with the teleop joystick, switch to waypoint mode, or hit emergency stop. The web server publishes `/cmd_vel` from a persistent ROS2 publisher thread running in the background — it never touches the serial port directly (that's owned exclusively by `serial_bridge`).

---

## Calibration

Override defaults without editing code:

| Variable | Default | Description |
|---|---|---|
| `SERIAL_PORT` | `/dev/rosnav_serial` | Serial device path |
| `BAUD_RATE` | `115200` | Must match firmware `SERIAL_BAUD` |
| `WHEEL_BASE` | `0.15` | Center-to-center wheel distance (m) |
| `WHEEL_RAD` | `0.0325` | Wheel radius (m) |
| `ENC_PPR` | `360` | Encoder pulses per revolution |

Example:

```bash
export WHEEL_BASE=0.18
export WHEEL_RAD=0.035
./start.sh
```

`WHEEL_BASE` is the most impactful parameter for straight-line accuracy. If the robot drifts in arcs when commanded forward, measure the actual center-to-center wheel distance carefully and set this exactly. The odometry in `serial_bridge.py` uses the standard differential-drive model: `dc = (dL + dR) / 2`, `dth = (dR - dL) / WHEEL_BASE`.

---

## Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| `/dev/rosnav_serial` not found | STM32 USB not detected. Run `ls /dev/tty*` before and after plugging in. Check if your adapter (CH340 / CP2102 / ST-Link) matches one of the three in the udev rule |
| Permission denied on serial port | `sudo chmod 666 /dev/ttyUSB0`, or re-login after `sudo usermod -aG dialout $USER` |
| STM32 not sending data | Firmware not flashed, or wrong baud rate. Connect via Arduino Serial Monitor at 115200 — you should see JSON lines immediately |
| Motors don't respond | Verify ENA/IN1/IN2 wiring to L298N and that motor driver shares GND with STM32 |
| Motors start but immediately stop | Watchdog kicking in — `serial_bridge` isn't sending heartbeats. Check the bridge node is running and UART connection is healthy |
| Encoder values stuck at 0 | Check encoder VCC and that signal wires reach the interrupt-capable pins (PA6, PB9) |
| MPU6050 not detected | Add 4.7 kΩ pull-ups on SDA/SCL to 3.3 V; confirm AD0 → GND; verify I2C is enabled in `config.txt` |
| Serial data looks corrupted | Baud mismatch — confirm `SERIAL_BAUD 115200` in firmware and `BAUD_RATE=115200` on the Pi |
| Web UI won't load | Confirm `python3 app.py` is running and port 5000 is reachable. Run `hostname -I` for the correct IP |
| Robot curves instead of going straight | `WHEEL_BASE` is off — measure the actual center-to-center distance and set it exactly |
| `waypoint_nav` crashes on start | `/tmp/rosnav_waypoints.json` is missing or malformed — set waypoints via the web UI before switching to waypoint mode |
| ROS2 nodes crash immediately | STM32 not powered, or serial port not present. Run `check_hardware.py` first |

---

## Roadmap

- [ ] SLAM with RPLIDAR A1 + slam_toolbox
- [ ] Map persistence and multi-mission waypoint planning
- [ ] PID velocity controller on STM32 (currently open-loop with proportional scaling)
- [ ] PID tuning interface in the web dashboard
- [ ] Pi Camera v2 feed with basic obstacle detection
- [ ] RViz2 configuration for desktop visualization
- [ ] Docker container for the ROS2 workspace
- [ ] Unit tests for JSON serial protocol and odometry math

---

## Contributing

Pull requests are welcome. If you're adapting this for different hardware (different MCU, motor driver, encoder PPR, or chassis geometry), opening an issue with your config first helps avoid duplicate effort.

For bigger changes — new ROS2 nodes, reworking the serial protocol, replacing Nav2 — please open an issue before submitting a PR so we can align on direction.

```bash
git checkout -b feature/your-feature-name
# make changes, then open a PR against main
```

Keep commits focused and include a short description of what changed and why.

---

## License

MIT — see [LICENSE](LICENSE) for details.

---

## Acknowledgements

- [ROS2 Humble](https://docs.ros.org/en/humble/) — navigation middleware
- [Nav2](https://nav2.ros.org/) — autonomous navigation stack
- [robot\_localization](https://docs.ros.org/en/humble/p/robot_localization/) — EKF for sensor fusion
- [STM32duino](https://github.com/stm32duino) — Arduino core for STM32F1
- [Flask](https://flask.palletsprojects.com/) + [Flask-SocketIO](https://flask-socketio.readthedocs.io/) — web dashboard backend

---

## Authors

**D Anudeep**, **K Vinay**  — STM32 firmware, hardware bring-up, ROS2 serial bridge, navigation stack, web dashboard, deployment and install tooling

