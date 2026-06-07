std_msgs/Header header

uint64  id
int32   robot_id

uint8 POSE     = 0
uint8 LANDMARK = 1
uint8 type

geometry_msgs/PoseWithCovariance pose      # used when type == POSE
geometry_msgs/Point              position  # used when type == LANDMARK
