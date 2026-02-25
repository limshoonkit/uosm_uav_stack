# uosm_msgs/msg/TrunkObservation.msg
std_msgs/Header header          # Timestamp

uint32 landmark_id              # The unique ID of the observed trunk
geometry_msgs/Point position    # Measured position [x, y] of the trunk center relative to base_link
float64[4] covariance           # 2x2 covariance matrix for the position measurement [Cxx, Cxy, Cxy, Cyy]