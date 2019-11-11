uint8 MODE_IDLE=0
uint8 MODE_LOCALISING=1
uint8 MODE_MAPPING=2
uint8 mode

uint8 LOST = 0
uint8 LOCALISED = 1
uint8 PAUSED = 2
uint8 localisation_status

bool map_loaded

# map to odom frame
geometry_msgs/TransformStamped localisation

uint8 number_of_global_constraints
