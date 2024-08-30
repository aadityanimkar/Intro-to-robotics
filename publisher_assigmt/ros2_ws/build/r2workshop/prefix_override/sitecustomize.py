import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aaditya/Intro-to-robotics/publisher_assigmt/ros2_ws/install/r2workshop'
