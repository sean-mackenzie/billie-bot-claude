# Classified audio event from YAMNet + ReSpeaker
std_msgs/Header header

# Event type constants
uint8 BARK=0
uint8 WHINE=1
uint8 HOWL=2
uint8 LOUD_NOISE=3
uint8 SILENCE=4

uint8 event_type
float32 confidence

# Direction of arrival in degrees [0, 360) from robot front
float32 doa_deg

# Raw YAMNet class label
string yamnet_label

# Energy level (dB)
float32 energy_db
