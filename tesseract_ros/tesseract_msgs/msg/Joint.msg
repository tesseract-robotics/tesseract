uint8 UNKNOWN=0
uint8 REVOLUTE=1
uint8 CONTINUOUS=2
uint8 PRISMATIC=3
uint8 FLOATING=4
uint8 PLANAR=5
uint8 FIXED=6

# Required Parameters
string name
uint8 type # (UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED)

# \brief     type_       meaning of axis_
# ------------------------------------------------------
#            UNKNOWN     unknown type
#            REVOLUTE    rotation axis
#            PRISMATIC   translation axis
#            FLOATING    N/A
#            PLANAR      plane normal axis
#            FIXED       N/
float64[3] axis
string child_link_name
string parent_link_name
geometry_msgs/Pose parent_to_joint_origin_transform
tesseract_msgs/JointLimits limits

# Optional Parameters
tesseract_msgs/JointDynamics dynamics
tesseract_msgs/JointSafety safety
tesseract_msgs/JointCalibration calibration
tesseract_msgs/JointMimic mimic
