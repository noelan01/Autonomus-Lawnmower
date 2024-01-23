# State "enum"
uint8 MOWERAPP_MODE_AUTO = 0
uint8 MOWERAPP_MODE_MANUAL = 1
uint8 MOWERAPP_MODE_HOME = 2
uint8 MOWERAPP_MODE_DEMO = 3
uint8 MOWERAPP_MODE_POI = 4

std_msgs/Header     header
uint8               mode
string              description
