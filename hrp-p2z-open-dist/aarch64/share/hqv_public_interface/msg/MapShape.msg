#Subscription constants
int8 UNDEFIED_AREA=-1
int8 WORKING_AREA=0
int8 TRANSPORT_AREA=1
uint8 FORBIDDEN_AREA=2
uint8 HIDDEN_AREA=3
int8 TRANSPORT_PATH=4
int8 POINTS_OF_INTEREST_PATH=5
int8 CHARGING_STATION_PATH=6

geometry_msgs/Polygon polygon
char[64] id
int8 type
uint32 stroke_color
uint32 fill_color
bool hidden
