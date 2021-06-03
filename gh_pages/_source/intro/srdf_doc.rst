SRDF
====

The SRDF or Semantic Robot Description Format complement the URDF and specifies joint groups, default robot configurations, additional collision checking information, and additional transforms that may be needed to completely specify the robot's pose. The recommended way to generate a SRDF is using the Tesseract Setup Assistant.

Groups
^^^^^^
Groups (also referred to as kinematics groups) define collections of links and joints that are used for planning. Groups can be specified in several ways:

Collection of Joints
""""""""""""""""""""
A group can be specified as a collection of joints. All the child links of each joint are automatically included in the group.

Collection of Links
"""""""""""""""""""
A group can also be specified as a collection of links. All the parent joints of the links are also included in the group.

Serial Chain
""""""""""""
A serial chain is specified using the base link and the tip link. The tip link in a chain is the child link of the last joint in the chain. The base link in a chain is the parent link for the first joint in the chain.

Collection of Sub-Groups
""""""""""""""""""""""""
A group can also be a collection of groups. E.g., you can define left_arm and right_arm as two groups and then define a new group called both_arms that includes these two groups.

End-Effectors
^^^^^^^^^^^^^
Certain groups in a robot can be given a special designation as an end-effector. An end-effector is typically connected to another group (like an arm) through a fixed joint. Note that when specifying groups that are end-effectors, it's important to make sure that there are no common links between the end-effector and the parent group it is connected to.

Self-Collisions
^^^^^^^^^^^^^^^
The Default Self-Collision Matrix Generator (part of Setup Assistant) searches for pairs of links on the robot that can safely be disabled from collision checking, decreasing motion planning processing time. These pairs of links are disabled when they are always in collision, never in collision, in collision in the robot's default position or when the links are adjacent to each other on the kinematic chain. The sampling density specifies how many random robot positions to check for self collision. Higher densities require more computation time while lower densities have a higher possibility of disabling pairs that should not be disabled. The default value is 10,000 collision checks. Collision checking is done in parallel to decrease processing time.

Robot Poses
^^^^^^^^^^^
The SRDF can also store fixed configurations of the robot. A typical example of the SRDF in this case is in defining a HOME position for a manipulator. The configuration is stored with a string id, which can be used to recover the configuration later.

SRDF Documentation
^^^^^^^^^^^^^^^^^^
For information about the syntax for the SRDF, read more details on the `ROS SRDF Wiki page <http://www.ros.org/wiki/srdf>`_.
