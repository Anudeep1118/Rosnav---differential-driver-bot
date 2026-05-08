#!/usr/bin/env python3
"""
RosNav — Hardware Check
Run before starting the system to verify all connections.
Usage: python3 scripts/check_hardware.py
"""
import sys, os, glob, time, subprocess

G='\033[0;32m'; R='\033[0;31m'; Y='\033[1;33m'; C='\033[0;36m'; N='\033[0m'
ok   = lambda m: print(f"{G}  [PASS]{N} {m}")
fail = lambda m: print(f"{R}  [FAIL]{N} {m}")
warn = lambda m: print(f"{Y}  [WARN]{N} {m}")
info = lambda m: print(f"{C}  [INFO]{N} {m}")

print(f"\n{C}{'═'*44}{N}")
print(f"{C}  RosNav Hardware Check{N}")
print(f"{C}{'═'*44}{N}\n")
errors = 0

# 1. Python packages
print("1. Python packages")
for pkg in ['serial','flask','flask_socketio']:
    try:
        __import__(pkg); ok(f"{pkg}")
    except ImportError:
        fail(f"{pkg} missing — pip3 install {pkg}"); errors+=1

# 2. Serial port
print("\n2. Serial port (STM32)")
ports = (glob.glob('/dev/rosnav_serial') +
         glob.glob('/dev/ttyUSB*') +
         glob.glob('/dev/ttyACM*'))

if not ports:
    fail("No serial ports found — plug in STM32 USB"); errors+=1
else:
    for p in ports:
        if os.access(p, os.R_OK|os.W_OK):
            ok(f"{p} — accessible")
        else:
            fail(f"{p} — permission denied")
            print(f"       Fix: sudo chmod 666 {p}")
            errors+=1

    try:
        import serial
        port=ports[0]
        ser=serial.Serial(port,115200,timeout=1.5)
        ok(f"Opened {port}")
        info("Waiting for STM32 response...")
        time.sleep(2)
        if ser.in_waiting>0:
            line=ser.readline().decode('utf-8',errors='ignore').strip()
            ok(f"STM32 says: {line}")
        else:
            warn("No data — is firmware flashed? Check USB cable")
        ser.close()
    except Exception as e:
        fail(f"Serial open error: {e}"); errors+=1

# 3. I2C / MPU6050
print("\n3. I2C — MPU6050")
try:
    import smbus2
    bus=smbus2.SMBus(1)
    try:
        who=bus.read_byte_data(0x68,0x75)
        if who==0x68: ok("MPU6050 at 0x68 — WHO_AM_I OK")
        else: warn(f"Device at 0x68 returned 0x{who:02X} (expected 0x68)")
    except OSError:
        fail("MPU6050 not found at 0x68")
        print("       Check: SDA→PB9, SCL→PB8, VCC→3.3V, GND→GND")
        print("       Check: 4.7kΩ pull-ups on SDA and SCL to 3.3V")
        print("       Check: AD0 pin → GND")
        errors+=1
    bus.close()
except ImportError:
    warn("smbus2 not installed — pip3 install smbus2")
except Exception as e:
    warn(f"I2C error: {e}")

# 4. ROS2
print("\n4. ROS2")
r=subprocess.run(['which','ros2'],capture_output=True,text=True)
if r.returncode==0:
    ok(f"ros2: {r.stdout.strip()}")
else:
    fail("ros2 not in PATH — run: source /opt/ros/humble/setup.bash"); errors+=1

r2=subprocess.run(
    ['bash','-c','source /opt/ros/humble/setup.bash 2>/dev/null && ros2 pkg list'],
    capture_output=True, text=True)
if 'rosnav_driver' in r2.stdout:
    ok("rosnav_driver package found")
else:
    warn("rosnav_driver not found — cd ros2_ws && colcon build")

# 5. Network
print("\n5. Network")
import socket
try:
    ip=socket.gethostbyname(socket.gethostname())
    ok(f"IP: {ip}")
    info(f"Web UI will be at: http://{ip}:5000")
except Exception as e:
    warn(f"IP lookup failed: {e}")

# Summary
print(f"\n{C}{'═'*44}{N}")
if errors==0:
    print(f"{G}  ALL CHECKS PASSED — run ./start.sh{N}")
else:
    print(f"{R}  {errors} ISSUE(S) — fix above before launching{N}")
print(f"{C}{'═'*44}{N}\n")
sys.exit(errors)
