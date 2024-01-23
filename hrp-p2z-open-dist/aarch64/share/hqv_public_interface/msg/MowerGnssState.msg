#state constants
uint8 IGNSS_STATE_STARTUP=0
uint8 IGNSS_STATE_OK=1
uint8 IGNSS_STATE_HALTED=2
uint8 IGNSS_STATE_ERROR=3
std_msgs/Header   header
uint8             state
