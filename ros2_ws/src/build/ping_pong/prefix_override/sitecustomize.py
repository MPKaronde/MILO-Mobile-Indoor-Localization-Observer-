import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/milo/project_parts/MILO-Mobile-Indoor-Localization-Observer-/ros2_ws/src/install/ping_pong'
