**********************
Tesseract Setup Wizard
**********************

Overview
========

The Tesseract Setup Wizard is a GUI that is designed to help you generate a Semantic Robot Description Format file (SRDF). This
file defines your robot's:

* allowed collision matrix
* kinematics groups
* user defined joint states
* user defined tool center points (TCPs)
* OPW kinematics parameters


Getting Started
===============

The Tesseract Setup Wizard is part of the Tesseract Ignition snap package.

- You can install this package by using the following command: ::

   sudo snap install tesseract-ignition


Step 1: Start
=============

.. note:: If you are using ROS, make sure you source your workspace before launching the Tesseract Setup Wizard

* Launch the Tesseract Setup Wizard with the following command: ::

   tesseract-ignition.tesseract-setup-wizard

.. note:: The Tesseract Setup Wizard contains a toolbar with pages for each component of the SRDF file.
          To navigate to different pages, you must click on the dark grey background of the tool bar, hold,
          and drag you mouse left or right.


Step 2: Load URDF and SRDF
==========================

.. image:: /_static/tesseract_setup_wizard_startup.png

To use the Tesseract Setup Wizard and create/edit a SRDF file, we'll first need to
load in your URDF file. If you'd like to edit an existing SRDF, you may also load
that in as well.

* To load in your URDF file:

  1. Click the **BROWSE** button to the right of the *URDF File* field.

  2. Find and select your URDF file.

* If you would like to edit an existing SRDF file:

  1. Click the **BROWSE** button to the right of the *SRDF File* field.

  2. Find and select you SRDF file.

* Once you have selected your URDF file (required) and SRDF file (not required):

  1. Click **LOAD** to load your model into the setup wizard.

  2. At this point, you should see your model appear in the workspace to the left of
     the toolbar.

  3. If your model did not appear, check the terminal to see error messages that will
     help you debug the issue.

* Click, hold, and drag left to move to the next page.


Step 3: Generate/Edit Allowed Collison Matrix
=============================================

.. image:: /_static/tesseract_setup_wizard_collision_matrix.png

The allowed collision matrix specifies which links don't need to be checked for collisions. Links that are adjacent
to each other or are never in collision should be ignored by the Tesseract Collision Manager.

.. note:: If you loaded in an existing SRDF file, the table should already be populated with your
          allowed collision matrix.

* To generate an allowed collision matrix:

  1. Click **GENERATE**. After a couple seconds you should see the table poplulate with your default collision matrix.

  2. Inspect your collision matrix and ensure that there are no link pairs that should not be disabled for collision checking.

  3. If you would like to remove a link pair (enabling it for collision checking) select the row in the table corresponding to
     the link pair, and click **REMOVE**.

* Click, hold, and drag left to move to the next page.


Step 4: Create/Edit Kinematics Groups
=====================================

.. image:: /_static/tesseract_setup_wizard_kinematics_groups.png

A kinematics group specifies a group of links that belong to a specific part of your robot. Usually, you'll want to specify
the chain of links that represent the arm and will be used for motion planning.

* A kinematics group can be specified in three ways:

  1. By selecting a chain of links
  2. By selecting each joint in the group
  3. By selecting each link in the group

* First, enter a name for your kinematics group in the *Group Name* field.

Method 1: Select Chain of Links
-------------------------------

Specifying a chain is the most common way to specify kinematics group.

* To do this:

  1. Click the **CHAIN** tab.

  2. Click the *Base Link* drop down box.

  3. Select the link that represents the first link in your chain (usually this is named something like *base_link*).

4. Click the *Tip Link* drop down box.

5. Select the link that represents the end of your chain (usually this is named something link *tool0* or *tcp*).

6. Click **ADD GROUP** to add the chain to your list of kinematics groups.

Method 2: Select All Links
--------------------------

You can also specify a kinematics group by specifying each joint in the group. To do this:

1. Click the **JOINTS** tab.

2. Click the *Joint Names* drop down box.

3. Select a joint to add to the group.

4. Click **ADD** to add the joint to the group.

5. Repeat steps 1-4 for each joint.

6. If you need to remove a joint, select the joint in the list and click **REMOVE**

7. Once you have added all joints to your group, click **ADD GROUP**.

Method 3: Select All Joints
---------------------------

You can also specify a kinematics group by specifying each link in the group. To do this:

1. Click the **LINKS** tab.

2. Click the *Link Names* drop down box.

3. Select a link to add to the group.

4. Click **ADD** to add the link to the group.

5. Repeat the last three steps for each link.

6. If you need to remove a link, select the link in the list and click **REMOVE**

7. Once you have added all links for your group, click **ADD GROUP**.

Removing a Group
----------------

To remove a group, select the group in the table and click **REMOVE** (at the bottom of the tool bar).

* Click, hold, and drag left to move to the next page.


Step 5: Create/Edit Joint States
================================

.. image:: /_static/tesseract_setup_wizard_joint_states.png

The *User Defined Joint States* page allows you to define different poses for your robot. For
example, it is often useful to create a joint state called *Home* which contains the joint values
for the starting/ending state of your robot.

* To define a joint state:

  1. Enter a name for your joint state in the *Joint State Name* field.

  2. Select the kinematic group that you would like to use from the *Group Name* drop down box.

  3. After selecting a group name, a value field for each joint should appear bellow
     the *Group Name* drop down box. For each joint, select the value you'd like to set
     for the robot's position.

  4. Once each joint value is set, click **ADD STATE**.

* To remove a joint state:

  1. Select the joint state from the table.

  2. Click **REMOVE**.

* Click, hold, and drag left to move to the next page.


Step 6: Create/Edit TCPs
========================

.. image:: /_static/tesseract_setup_wizard_user_defined_tcps.png

The User Defined TCPs page allows you to define Tool Center Points for your kinematic
groups.

* To define a TCP:

  1. Enter a name for your TCP in the *TCP Name* field.

  2. Select the kinematic group that you would like to use from the *Group Name* drop down box.

  3. In the *Position* fields enter the *X*, *Y*, and *Z* positions of the TCP in reference to the
     last link in your kinematics group.

  4. In the *Orientation* fields enter the Roll (*R*), Pitch (*P*), and Yaw (*Y*) of the TCP in
     reference to the last link in your kinematics group.

  5. Click **ADD TCP**.

* To remove a TCP:

  1. Select the TCP from the table

  2. Click **REMOVE**.

* Click, hold, and drag left to move to the next page.


Step 7: Setting OPW Parameters
==============================

.. image:: /_static/tesseract_setup_wizard_opw_kinematics.png

OPW is an effecient inverse kinematics solver for robots with parallel bases and spherical wrists. This algorithm
requires 7 measurements from the robot's specification sheet to be defined here. These values will be stored
in the SRDF and used by the OPW solver.

* To define your robot's OPW parameters:

1. Use the following diagram to determine each parameter

.. image:: /_static/tesseract_setup_wizard_opw_diagram.png

2. Enter each value in it's respective field.

* For more details on the OPW algorithm, visit the `opw_kinematics github repository <https://github.com/Jmeyer1292/opw_kinematics>`_.

* Click, hold, and drag left to move to the next page.


Step 8: Saving the SRDF File
============================

.. image:: /_static/tesseract_setup_wizard_save_srdf.png

Settings for the allowed collision matrix, kinematics groups, joint states, tcp values, and OPW
parameters are all stored in a Semantic Robot Description Format file (SRDF).

* To save your SRDF file:

  1. Scroll back to the left most page where you originally loaded your URDF file.

  2. Click **SAVE** and select a file and location to save the SRDF to.
