# Fused dog behavioral state estimate
std_msgs/Header header

# State constants
uint8 NOT_FOUND=0
uint8 SLEEPING=1
uint8 RESTING=2
uint8 ACTIVE=3
uint8 BARKING=4
uint8 EATING=5

# Current state
uint8 state
float32 confidence

# Position in map frame (valid when state != NOT_FOUND)
geometry_msgs/Point position
string room

# Context vector for future Behavior AI (SYS-EXT-3)
float32[] context

# Stress proxy: heuristic from bark rate + retreat motion (SYS-EXT-4)
float32 stress_proxy

# Duration in current state (seconds)
float64 state_duration
