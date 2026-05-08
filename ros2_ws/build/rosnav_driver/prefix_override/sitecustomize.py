import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anudeep/rosnav/ros2_ws/install/rosnav_driver'
