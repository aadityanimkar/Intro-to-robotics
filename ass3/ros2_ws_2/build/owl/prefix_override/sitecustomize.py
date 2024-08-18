import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aaditya/Intro-to-robotics/ass3/ros2_ws_2/install/owl'
