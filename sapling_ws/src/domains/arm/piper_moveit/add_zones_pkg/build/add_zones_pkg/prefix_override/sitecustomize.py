import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leo/piper_ws/src/src/piper_moveit/add_zones_pkg/install/add_zones_pkg'
