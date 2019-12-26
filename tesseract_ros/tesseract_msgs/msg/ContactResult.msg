time stamp
float64 distance
byte[2] type_id
string[2] link_names
uint64[2] shape_id # Link's child shape index
uint64[2] subshape_id # If mesh or octomap this will be the voxel/triangle index
geometry_msgs/Pose[2] transform # Transform of link_names at time step 0
geometry_msgs/Vector3[2] nearest_points
geometry_msgs/Vector3[2] nearest_points_local
geometry_msgs/Vector3 normal
geometry_msgs/Pose[2] cc_transform # Transform of link_names at time step 1
float64[2] cc_time # The time at contact during continuous collision checking
byte[2] cc_type #CCType_None = 0, CCType_Time0 = 1, CCType_Time1 = 2, CCType_Between = 3
