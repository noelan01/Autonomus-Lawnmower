# State "enum"
uint8 STATE_RUNNING = 0      # Perception engine is running
uint8 STATE_IDLE = 1         # Perception engine is dile
uint8 STATE_STARTED = 2      # Perception engine is started
uint8 STATE_CONNECTED = 3    # Perception engine is connected
uint8 STATE_READY = 4        # Perception engine is ready
uint8 STATE_CALIBRATING = 5  # Perception engine is calibrating
uint8 STATE_FILTER_WARM_UP = 6  # Perception engine filter is warming up
uint8 STATE_WAITING_FOR_GLOBAL_POS = 7  # Perception engine waiting for global coordinates
uint8 STATE_UNKNOWN = 8

#std_msgs/Header     header
uint8 state
