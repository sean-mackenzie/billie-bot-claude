# 3D dog detection from OAK-D Lite stereo depth
std_msgs/Header header

# 2D bounding box in image coordinates
int32 bbox_x
int32 bbox_y
int32 bbox_w
int32 bbox_h

# Detection confidence [0.0, 1.0]
float32 confidence

# 3D position in camera optical frame
geometry_msgs/Point position

# Depth in metres
float32 depth

# COCO class label (should be "dog")
string label
