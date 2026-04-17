import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/youssef_negm_24/Desktop/ELSA2/ws/install/campus_vision'
