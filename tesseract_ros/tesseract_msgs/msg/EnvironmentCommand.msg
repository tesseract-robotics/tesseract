# Possible Commands
uint8 ADD=0
uint8 MOVE_LINK=1
uint8 MOVE_JOINT=2
uint8 REMOVE_LINK=3
uint8 REMOVE_JOINT=4
uint8 CHANGE_LINK_ORIGIN=5
uint8 CHANGE_JOINT_ORIGIN=6
uint8 CHANGE_LINK_COLLISION_ENABLED = 7
uint8 CHANGE_LINK_VISIBILITY=8
uint8 ADD_ALLOWED_COLLISION=9
uint8 REMOVE_ALLOWED_COLLISION=10
uint8 REMOVE_ALLOWED_COLLISION_LINK=11
uint8 UPDATE_JOINT_STATE=12

# Specifies the type of command. Dictates which of the fields below are used
uint8 command

# ADD - Adds a link and the associated joint to the environment
tesseract_msgs/Link add_link
tesseract_msgs/Joint add_joint

# MOVE_LINK - Changes the joint associated with the link
tesseract_msgs/Joint move_link_joint

# MOVE_JOINT - Changes the parent link associated with the joint
string move_joint_name
string move_joint_parent_link

# REMOVE_LINK - Removes the link from the environment
string remove_link

# REMOVE_JOINT - Removes the joint from the environment
string remove_joint

# CHANGE_LINK_ORIGIN - Changes the origin of the link
string change_link_origin_name
geometry_msgs/Pose change_link_origin_pose

# CHANGE_JOINT_ORIGIN - Changes the origin of the joint
string change_joint_origin_name
geometry_msgs/Pose change_joint_origin_pose

# CHANGE_LINK_COLLISION_ENABLED - Enables/Disables collisions for the link
string change_link_collision_enabled_name
bool change_link_collision_enabled_value

# CHANGE_LINK_VISIBILITY - Enables/Disables visibility for the link
string change_link_visibility_name
bool change_link_visibility_value

# ADD_ALLOWED_COLLISION - Adds an allowable collision to the allowable collision matrix
tesseract_msgs/AllowedCollisionEntry add_allowed_collision

# REMOVE_ALLOWED_COLLISION - Removes an allowable collision from the allowable collision matrix
tesseract_msgs/AllowedCollisionEntry remove_allowed_collision

# REMOVE_ALLOWED_COLLISION_LINK
string remove_allowed_collision_link

# UPDATE_JOINT_STATE
sensor_msgs/JointState joint_state
