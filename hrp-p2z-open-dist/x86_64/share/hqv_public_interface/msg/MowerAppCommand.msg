#Subscription constants
uint8 START=0
uint8 PAUSE=1
uint8 ABORT_START=2
uint8 CONFIRM_ERROR=3

#Subscription Request
std_msgs/Header     header
uint8 command
