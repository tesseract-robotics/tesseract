URDF
====

Tesseract is compatible with URDFs (Universal Robot Description Format), the native format for describing robots in ROS. In this tutorial, you will find resources for the URDF and important tips.

.. attention:: Tesseract has extended capabilities for URDFs. For more information, see `Tesseract URDF <tesseract_urdf_doc.html>`_.

URDF Resources
^^^^^^^^^^^^^^

* `URDF ROS Wiki Page <http://www.ros.org/wiki/urdf>`_ - The URDF ROS Wiki page is the source of most information about the URDF.
* `URDF Tutorials <http://www.ros.org/wiki/urdf/Tutorials>`_ - Tutorials for working with the URDF.
* `SOLIDWORKS URDF Plugin <http://www.ros.org/wiki/sw_urdf_exporter>`_ - A plugin that lets you generate a URDF directly from a SOLIDWORKS model.

Important Tips
^^^^^^^^^^^^^^
This section contains a set of tips on making sure that the URDF that you generate can be used with Tesseract. Make sure you go through all these tips before starting to use Tesseract with your robot.

Special Characters in Joint Names
"""""""""""""""""""""""""""""""""
Joint names should not contain any of the following special characters: -, [, ], (, )

Safety Limits
"""""""""""""
Some URDFs have safety limits set in addition to the joint limits of the robot. Here's an example of the safety controller specified for the Panda robot head pan joint: ::

   <safety_controller k_position="100" k_velocity="1.5" soft_lower_limit="-2.857" soft_upper_limit="2.857"/>

The "soft_lower_limit" field and the "soft_upper_limit" field specify the joint position limits for this joint. Tesseract will compare these limits to the hard limits for the joint specified in the URDF and choose the limits that are more conservative.

.. note:: If the "soft_lower_limit" and the "soft_upper_limit" in the safety_controller are set to 0.0, your joint will be unable to move. Tesseract relies on you to specify the correct robot model.

Collision Checking
""""""""""""""""""
Tesseract uses the meshes specified in the URDF for collision checking. The URDF allows you to specify two sets of meshes separately for visualization and collision checking. In general, the visualization meshes can be detailed and pretty, but the collision meshes should be much less detailed. The number of triangles in a mesh affects the amount of time it takes to collision check a robot link. The number of triangles in the whole robot should be on the order of a few thousand.

Test your URDF
""""""""""""""
It is very important to test your URDF out and make sure things are ok. The ROS URDF packages provide a check_urdf tool. To verify your URDF using the check_urdf tool, follow the instructions `here <http://wiki.ros.org/urdf#Verification>`_.

URDF Examples
^^^^^^^^^^^^^
There are lots of URDFs available for robots using ROS.

* `URDF Examples <http://www.ros.org/wiki/urdf/Examples>`_ - A list of URDFs from the ROS community.
