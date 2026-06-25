import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/njoy/ROS/rover_ws/src/install/py_pubsub'
