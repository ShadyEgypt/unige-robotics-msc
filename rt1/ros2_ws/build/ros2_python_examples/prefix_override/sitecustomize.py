import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shady/Documents/unige-robotics-msc/rt1/ros2_ws/install/ros2_python_examples'
