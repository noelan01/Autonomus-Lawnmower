#Subscription constants
uint8 FEEDBACK_NORMAL = 0             # Nominal driving
uint8 FEEDBACK_FULL_STOP = 1          # Stop in place
uint8 FEEDBACK_TURN_IN_PLACE = 2      # Perform turn in place
uint8 FEEDBACK_ADAPT_MAX_VELOCITY = 3 # Adapt max velocity
uint8 FEEDBACK_CHANGE_HEADING = 4     # Change heading
uint8 FEEDBACK_DRIVE_STRAIGHT = 5     # Drive straight
uint8 FEEDBACK_DRIVE_TO = 6           # Drive to an {x,y} coordinate
uint8 FEEDBACK_CALIBRATION_DONE = 7   # Calibration done
uint8 FEEDBACK_RTK_ANOMALY = 8        # RTK anomaly
uint8 FEEDBACK_UNKNOWN = 9

#std_msgs/Header     header
uint8 feedback
int32 feedback_data_1
int32 feedback_data_2
