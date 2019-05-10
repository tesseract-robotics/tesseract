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

uint8 command

# ADD
tesseract_msgs/Link add_link
tesseract_msgs/Joint add_joint

# MOVE_LINK
tesseract_msgs/Joint move_link_joint

# MOVE_JOINT
string move_joint_name
string move_joint_parent_link

# REMOVE_LINK
string remove_link

# REMOVE_JOINT
string remove_joint

# CHANTE_LINK_ORIGIN
string change_link_orgin_name
geometry_msgs/Pose change_link_orgin_pose

# CHANGE_JOINT_ORIGIN
string change_joint_orgin_name
geometry_msgs/Pose change_joint_orgin_pose

# CHANGE_LINK_COLLISION_ENABLED
string change_link_collision_enabled_name
bool change_link_collision_enabled_value

# CHANGE_LINK_VISIBILITY
string change_link_visibility_name
bool change_link_visibility_value

# ADD_ALLOWED_COLLISION
tesseract_msgs/AllowedCollisionEntry add_allowed_collision

# REMOVE_ALLOWED_COLLISION
tesseract_msgs/AllowedCollisionEntry remove_allowed_collision

# REMOVE_ALLOWED_COLLISION_LINK
string remove_allowed_collision_link

# UPDATE_JOINT_STATE
sensor_msgs/JointState joint_state
