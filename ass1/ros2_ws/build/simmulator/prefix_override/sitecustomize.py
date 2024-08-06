import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aaditya/Intro-to-robotics/ass1/ros2_ws/install/simmulator'
