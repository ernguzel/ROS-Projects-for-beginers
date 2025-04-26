import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eren/ern/projects/ROS-Projects-for-beginers/otonom_sarj/install/otonom_sarj'
