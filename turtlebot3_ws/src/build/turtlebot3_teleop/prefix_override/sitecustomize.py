import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/njoy/rover-simulation/turtlebot3_ws/src/install/turtlebot3_teleop'
