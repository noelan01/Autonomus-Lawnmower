# Message type "enum"
#uint8 MESSAGE_TYPE_SET_ACTIVE_MAP = 0
uint8 MESSAGE_TYPE_GET_ACTIVE_MAP = 1
#uint8 MESSAGE_TYPE_CREATE_MAP = 2
#uint8 MESSAGE_TYPE_DELETE_MAP = 3
uint8 MESSAGE_TYPE_LIST_MAPS = 4
#uint8 MESSAGE_TYPE_DELETE_MAPS = 5

uint8 message_type      # See above
uint32 map_id           # Only used for MESSAGE_TYPE = 0,2,3

