import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aly/Desktop/ELSA/ELSA/install/campus_vision'
