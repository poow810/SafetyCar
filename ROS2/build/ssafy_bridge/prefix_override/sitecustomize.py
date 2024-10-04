import sys
if sys.prefix == 'c:\\python37':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = 'C:\\Users\\SSAFY\\Desktop\\git_clone\\S11P21B209\\ROS2\\install\\ssafy_bridge'
