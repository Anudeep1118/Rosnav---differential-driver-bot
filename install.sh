#!/bin/bash
# ============================================================
#  RosNav — Master Install Script
#  Run ONCE on Raspberry Pi 4 with Ubuntu 22.04
#
#  Usage:
#    cd ~/rosnav
#    chmod +x install.sh
#    ./install.sh
# ============================================================
set -e

CYAN='\033[0;36m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $1"; }
ok()    { echo -e "${GREEN}[ OK ]${NC}  $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
err()   { echo -e "${RED}[ERR ]${NC}  $1"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
info "RosNav install starting from: $SCRIPT_DIR"

# ── 1. System packages ──────────────────────────────────
info "Updating system packages..."
sudo apt update -y
sudo apt install -y \
    curl wget git \
    python3-pip python3-venv \
    software-properties-common \
    build-essential \
    i2c-tools \
    udev
ok "System packages done"

# ── 2. Enable UART and I2C on Raspberry Pi ─────────────
info "Enabling UART and I2C..."
CONFIG=/boot/firmware/config.txt
if [ ! -f "$CONFIG" ]; then
    CONFIG=/boot/config.txt
fi

grep -qF "enable_uart=1"      "$CONFIG" || echo "enable_uart=1"      | sudo tee -a "$CONFIG"
grep -qF "dtparam=i2c_arm=on" "$CONFIG" || echo "dtparam=i2c_arm=on" | sudo tee -a "$CONFIG"
ok "UART and I2C enabled in $CONFIG"

# Disable serial console so UART is free for STM32
CMDLINE=/boot/firmware/cmdline.txt
if [ ! -f "$CMDLINE" ]; then CMDLINE=/boot/cmdline.txt; fi
if grep -q "console=serial0" "$CMDLINE"; then
    sudo sed -i 's/console=serial0,[0-9]* //' "$CMDLINE"
    ok "Serial console disabled — UART freed for STM32"
fi

# ── 3. udev rule for STM32 USB serial ──────────────────
info "Adding udev rules for STM32..."
sudo tee /etc/udev/rules.d/99-rosnav-serial.rules > /dev/null <<'UDEV'
# CH340 (common STM32 USB-Serial chip)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="rosnav_serial", MODE="0666"
# CP2102
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", SYMLINK+="rosnav_serial", MODE="0666"
# ST-Link / STM32 native USB
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", SYMLINK+="rosnav_serial", MODE="0666"
UDEV
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
ok "udev rules installed — STM32 will appear as /dev/rosnav_serial"

# ── 4. ROS2 Humble ──────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    info "Installing ROS2 Humble..."

    sudo rm -f /etc/apt/sources.list.d/ros2.list

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    ARCH=$(dpkg --print-architecture)
    CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME")
    echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
    ok "ROS2 Humble installed"
else
    ok "ROS2 already installed — skipping"
fi

# ── 5. ROS2 Nav2 + localization ────────────────────────
info "Installing Nav2 and TF2 packages..."
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros
ok "Nav2 packages installed"

# ── 6. Python packages ──────────────────────────────────
info "Installing Python packages..."
pip3 install --break-system-packages \
    flask==3.0.0 \
    flask-socketio==5.3.6 \
    pyserial==3.5 \
    eventlet==0.35.1
ok "Python packages installed"

# ── 7. Build ROS2 workspace ─────────────────────────────
info "Building ROS2 workspace..."
cd "$SCRIPT_DIR/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
ok "ROS2 workspace built"
cd "$SCRIPT_DIR"

# ── 8. Configure .bashrc ─────────────────────────────────
info "Updating ~/.bashrc..."
BASHRC="$HOME/.bashrc"

add_line() {
    grep -qxF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

add_line "source /opt/ros/humble/setup.bash"
add_line "source $SCRIPT_DIR/ros2_ws/install/setup.bash"
add_line "export SERIAL_PORT=/dev/rosnav_serial"
add_line "export ROSNAV_DIR=$SCRIPT_DIR"
ok ".bashrc updated"

# ── 9. Systemd services ──────────────────────────────────
info "Installing systemd services..."

sudo tee /etc/systemd/system/rosnav-bridge.service > /dev/null <<BRIDGE
[Unit]
Description=RosNav Serial Bridge
After=network.target

[Service]
User=$USER
WorkingDirectory=$SCRIPT_DIR/ros2_ws
Environment=SERIAL_PORT=/dev/rosnav_serial
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source $SCRIPT_DIR/ros2_ws/install/setup.bash && ros2 run rosnav_driver serial_bridge'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
BRIDGE

sudo tee /etc/systemd/system/rosnav-web.service > /dev/null <<WEB
[Unit]
Description=RosNav Web Server
After=network.target rosnav-bridge.service

[Service]
User=$USER
WorkingDirectory=$SCRIPT_DIR/web
Environment=SERIAL_PORT=/dev/rosnav_serial
ExecStart=/usr/bin/python3 $SCRIPT_DIR/web/app.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
WEB

sudo systemctl daemon-reload
sudo systemctl enable rosnav-bridge.service rosnav-web.service
ok "Systemd services installed and enabled"

# ── Done ─────────────────────────────────────────────────
PI_IP=$(hostname -I | awk '{print $1}')
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║     RosNav Install Complete!                 ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════╝${NC}"
echo ""
echo "  Next steps:"
echo "  1. Flash firmware/rosnav_firmware.ino to STM32 via Arduino IDE"
echo "  2. sudo reboot"
echo "  3. After reboot, services start automatically"
echo "  4. Open: http://${PI_IP}:5000"
echo ""
echo "  Or launch manually (no reboot needed):"
echo "    source ~/.bashrc && ./start.sh"
echo ""
