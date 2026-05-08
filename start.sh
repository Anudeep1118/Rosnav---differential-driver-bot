#!/bin/bash
# ============================================================
#  RosNav — Manual Launch Script
#  Usage: ./start.sh
# ============================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CYAN='\033[0;36m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}"
echo "  ██████╗  ██████╗ ███████╗███╗   ██╗ █████╗ ██╗   ██╗"
echo "  ██╔══██╗██╔═══██╗██╔════╝████╗  ██║██╔══██╗██║   ██║"
echo "  ██████╔╝██║   ██║███████╗██╔██╗ ██║███████║██║   ██║"
echo "  ██╔══██╗██║   ██║╚════██║██║╚██╗██║██╔══██║╚██╗ ██╔╝"
echo "  ██║  ██║╚██████╔╝███████║██║ ╚████║██║  ██║ ╚████╔╝ "
echo "  ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝  ╚═══╝╚═╝  ╚═╝  ╚═══╝ "
echo -e "${NC}"

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}[ERR] ROS2 Humble not found. Run ./install.sh first.${NC}"
    exit 1
fi

if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
else
    echo -e "${YELLOW}[WARN] Workspace not built — building now...${NC}"
    cd "$SCRIPT_DIR/ros2_ws"
    colcon build --symlink-install
    source install/setup.bash
    cd "$SCRIPT_DIR"
fi

# Auto-detect serial port
if [ -e /dev/rosnav_serial ]; then
    export SERIAL_PORT=/dev/rosnav_serial
    echo -e "${GREEN}[ OK ] STM32 found at /dev/rosnav_serial${NC}"
elif [ -e /dev/ttyUSB0 ]; then
    export SERIAL_PORT=/dev/ttyUSB0
    echo -e "${YELLOW}[WARN] Using /dev/ttyUSB0${NC}"
elif [ -e /dev/ttyACM0 ]; then
    export SERIAL_PORT=/dev/ttyACM0
    echo -e "${YELLOW}[WARN] Using /dev/ttyACM0${NC}"
else
    echo -e "${YELLOW}[WARN] No serial port found — running without hardware${NC}"
    export SERIAL_PORT=/dev/ttyUSB0
fi

PI_IP=$(hostname -I | awk '{print $1}')
echo -e "${CYAN}[INFO] Serial port : $SERIAL_PORT${NC}"
echo -e "${CYAN}[INFO] Pi IP       : $PI_IP${NC}"
echo ""

# Kill existing instances
pkill -f "serial_bridge" 2>/dev/null || true
pkill -f "waypoint_nav"  2>/dev/null || true
pkill -f "app.py"        2>/dev/null || true
sleep 1

# Start ROS2 serial bridge
echo -e "${CYAN}[INFO] Starting ROS2 serial bridge...${NC}"
ros2 run rosnav_driver serial_bridge &
BRIDGE_PID=$!
sleep 2

# Start web server
echo -e "${CYAN}[INFO] Starting web server on port 5000...${NC}"
cd "$SCRIPT_DIR/web"
python3 app.py &
WEB_PID=$!
sleep 1

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  RosNav is RUNNING                               ║${NC}"
echo -e "${GREEN}║  Web UI  →  http://${PI_IP}:5000         ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════╝${NC}"
echo ""
echo "  Press Ctrl+C to stop all services"
echo ""

trap "echo 'Stopping RosNav...'; kill $BRIDGE_PID $WEB_PID 2>/dev/null; exit 0" INT TERM
wait
