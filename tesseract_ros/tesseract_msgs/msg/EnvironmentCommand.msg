# Possible Commands
uint8 ADD=0
uint8 MOVE_LINK=1
uint8 MOVE_JOINT=2
uint8 UPDATE_LINK=3
uint8 UPDATE_JOINT=4
uint8 REMOVE_LINK=5
uint8 REMOVE_JOINT=6
uint8 UPDATE_COLLISION=7
uint8 UPDATE_ALLOWED_COLLISION=8
uint8 UPDATE_JOINT_STATE=9

uint8 command

# ADD
tesseract_msgs/Link add_link
tesseract_msgs/Joint add_joint

# MOVE_LINK
tesseract_msgs/Joint move_link_joint

# MOVE_JOINT
string move_joint_name
string move_joint_parent_link

# UPDATE_LINK
tesseract_msgs/Link update_link

# UPDATE_JOINT
tesseract_msgs/Joint update_joint

# REMOVE_LINK
string remove_link

# REMOVE_JOINT
string remove_joint

# UPDATE_COLLISION
string update_collision_link
bool enabled

# UPDATE_ALLOWED_COLLISION
tesseract_msgs/AllowedCollisionEntry[] allowed_collisions

# UPDATE_JOINT_STATE
sensor_msgs/JointState joint_state
