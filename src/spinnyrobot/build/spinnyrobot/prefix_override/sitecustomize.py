import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/Desktop/Week 2 Training/TR-Autonomy-2/src/spinnyrobot/install/spinnyrobot'
