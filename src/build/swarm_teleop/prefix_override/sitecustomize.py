import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/janhavi/SwarmRobotics/src/install/swarm_teleop'
