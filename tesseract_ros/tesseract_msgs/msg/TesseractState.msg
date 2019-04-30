# Environment Name
string name
uint64 id # This is used to notify subscribers if the state has changed.

# This message contains information about the state, i.e. the positions of its joints and links
sensor_msgs/JointState joint_state

# Joints that may have multiple DOF are specified here
sensor_msgs/MultiDOFJointState multi_dof_joint_state
