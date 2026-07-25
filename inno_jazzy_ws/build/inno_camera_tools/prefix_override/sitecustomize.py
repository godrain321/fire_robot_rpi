import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/seeno04/inno_jazzy_ws/install/inno_camera_tools'
