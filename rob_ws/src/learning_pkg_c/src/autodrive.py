import time

import rcply
from rcply.node import Node
from geometry_msgs.msg import Twist

class AutoDrive(Node):
    def __init__(self):
        super().__init__("auto_drive")