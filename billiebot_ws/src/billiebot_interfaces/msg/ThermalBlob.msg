# Warm body detection from MLX90640 thermal camera
std_msgs/Header header

# Blob centroid in thermal image coordinates (32x24)
float32 cx
float32 cy

# Blob area in pixels
int32 area

# Temperature statistics (Celsius)
float32 max_temp
float32 mean_temp

# Whether this blob likely represents the dog
bool is_dog_candidate
