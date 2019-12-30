uint8 CCType_None=0
uint8 CCType_Time0=1
uint8 CCType_Time1=2
uint8 CCType_Between=3

time stamp
float64 distance
uint8[2] type_id
string[2] link_names # The two links that are in contact
uint64[2] shape_id # The two shapes that are in contact. Each link can be made up of multiple shapes
uint64[2] subshape_id # If mesh or octomap this will be the voxel/triangle index
geometry_msgs/Vector3[2] nearest_points # The nearest point on both links in world coordinates
geometry_msgs/Vector3[2] nearest_points_local # The nearest point on both links in local(link) coordinates
geometry_msgs/Pose[2] transform # Transform of link_names at time step 0
geometry_msgs/Vector3 normal # The normal vector to move the two objects out of contact in world coordinates
float64[2] cc_time # The time at contact during continuous collision checking
uint8[2] cc_type #CCType_None = 0, CCType_Time0 = 1, CCType_Time1 = 2, CCType_Between = 3
geometry_msgs/Pose[2] cc_transform # Transform of link_names at time step 1
