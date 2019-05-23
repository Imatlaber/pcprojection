uint8 SYMBOL_TYPE_UNKNOWN = 0
uint8 SYMBOL_TYPE_ZEBRA = 1
uint8 SYMBOL_TYPE_LEFT = 2
uint8 SYMBOL_TYPE_FORWARD = 3
uint8 SYMBOL_TYPE_RIGHT = 4
uint8 SYMBOL_TYPE_REVERSE = 5
uint8 SYMBOL_TYPE_RIGHTFORWARD = 6

#project coordinate system: Z-upward, Y-North True, X-East
geometry_msgs/Point[]    proj_hull

#relative to the camera, in camera coordinate system: Z-forward, Y-Downward, X-Rightward
geometry_msgs/Point[]    camera_hull

uint8 type
