#status constants
uint8 ICODRIVER_COMMAND_NONE         = 0
uint8 ICODRIVER_COMMAND_ABORT        = 1
uint8 ICODRIVER_COMMAND_ROTATE       = 2
uint8 ICODRIVER_COMMAND_TURN         = 3
uint8 ICODRIVER_COMMAND_MOVE_TO      = 4
uint8 ICODRIVER_COMMAND_REDUCE_SPEED = 5
std_msgs/Header   header
uint8             command
