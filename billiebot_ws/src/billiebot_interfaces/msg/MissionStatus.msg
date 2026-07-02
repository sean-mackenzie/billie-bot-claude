# Current mission state
std_msgs/Header header

# Mode constants
uint8 IDLE=0
uint8 PATROL=1
uint8 INVESTIGATE=2
uint8 TRACK_OBSERVE=3
uint8 RETURN=4
uint8 SAFE=5

uint8 mode

# Current target waypoint name (if patrolling)
string current_waypoint

# Dog state summary
uint8 dog_state
float32 dog_confidence

# Battery
float32 battery_voltage

# Navigation status
bool nav_active
uint8 recovery_count

# E-stop engaged
bool estopped
