*****************************
Tesseract Scene Graph Package
*****************************

Background
==========
This package contains the scene graph and parsers. The scene graph is used to manage the connectivity of the environment. The scene graph inherits from boost graph so you are able to leverage boost graph utilities for searching.

Scene Graph (Tree)
------------------

.. image:: ../_static/tesseract_scene_graph_tree.png
   :alt: Scene Graph (Tree)

Scene Graph (Acyclic)
---------------------

.. image:: ../_static/tesseract_scene_graph_graph.png
   :alt: Scene Graph (Acyclic)

Features
========

#. Links - Get, Add, Remove, Modify, Show/Hide, and Enable/Disable Collision
#. Joints - Get, Add, Remove, Move and Modify
#. Allowed Collision Matrix - Get, Add, Remove
#. Graph Functions

   * Get Inbound/Outbound Joints for Link
   * Check if acyclic
   * Check if tree
   * Get Adjacent/InvAdjacent Links for Joint

#. Utility Functions

   * Save to Graph to Graph Description Language (DOT)
   * Get shortest path between two Links

#. Parsers

   * URDF Parser
   * SRDF Parser
   * KDL Parser
   * Mesh Parser

Examples
========

#. :ref:`Building A Scene Graph <ex1>`
#. :ref:`Create Scene Graph from URDF <ex2>`
#. :ref:`Parse SRDF adding ACM to Scene Graph <ex3>`
#. :ref:`Parse Mesh <ex4>`

.. _ex1:

Building A Scene Graph
======================

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp


Example Explanation
-------------------

Create Scene Graph
^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:1: Create scene graph
   :end-before: // documentation:end:1: Create scene graph

Add Links
^^^^^^^^^

Create the links. The links are able to be configured see Link documentation.

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
  :language: c++
  :start-after: // documentation:start:2: Create links
  :end-before: // documentation:end:2: Create links

Add the links to the scene graph

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:3: Add links
   :end-before: // documentation:end:3: Add links

Add Joints
^^^^^^^^^^

Create the joints. The links are able to be configured see Joint documentation.

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:4: Create joints
   :end-before: // documentation:end:4: Create joints

Add the joints to the scene graph_acyclic_tree_example

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:5: Add joints
   :end-before: // documentation:end:5: Add joints

Inspect Scene Graph
^^^^^^^^^^^^^^^^^^^

Get the adjacent links for **link_3** and print to terminal

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:6: Check getAdjacentLinkNames Method
   :end-before: // documentation:end:6: Check getAdjacentLinkNames Method

Get the inverse adjacent links for **link_3** and print to terminal

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:7: Check getInvAdjacentLinkNames Method
   :end-before: // documentation:end:7: Check getInvAdjacentLinkNames Method

Get child link names for link **link_3** and print to terminal

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:8: Check getLinkChildrenNames
   :end-before: // documentation:end:8: Check getLinkChildrenNames

Get child link names for joint **joint_1** and print to terminal

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:9: Check getJointChildrenNames
   :end-before: // documentation:end:9: Check getJointChildrenNames

Save the graph to a file for visualization

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:10: Save Graph
   :end-before: // documentation:end:10: Save Graph

Test if the graph is Acyclic and print to terminal

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:11: Test if the graph is Acyclic
   :end-before: // documentation:end:11: Test if the graph is Acyclic

Test if the graph is a tree and print to terminal

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:12: Test if the graph is Tree
   :end-before: // documentation:end:12: Test if the graph is Tree

Detect Unused Links
^^^^^^^^^^^^^^^^^^^

First add a link but do not create joint and check if it is a tree. It should return false because the link is not associated with a joint.

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:13: Test for unused links
   :end-before: // documentation:end:13: Test for unused links

Remove link and check if it is a tree. It should return true.

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:14: Remove unused link
   :end-before: // documentation:end:14: Remove unused link

Create Acyclic Graph
^^^^^^^^^^^^^^^^^^^^

Add joint connecting **link_5** and **link_4** to create an Acyclic graph_acyclic_tree_example

.. image:: ../_static/tesseract_scene_graph_graph.png

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:15: Add new joint
   :end-before: // documentation:end:15: Add new joint

Save the Acyclic graph

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:16: Save new graph
   :end-before: // documentation:end:16: Save new graph

Test to confirm it is acyclic, should return true.

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:17: Test again if the graph is Acyclic
   :end-before: // documentation:end:17: Test again if the graph is Acyclic

Test if it is a tree, should return false.

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:18: Test again if the graph is Tree
   :end-before: // documentation:end:18: Test again if the graph is Tree

Get Shortest Path
^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:19: Get Shortest Path
   :end-before: // documentation:end:19: Get Shortest Path


.. _ex2:

Create Scene Graph from URDF
============================

.. literalinclude:: ../../tesseract_urdf/examples/load_urdf_example.cpp

Example Explanation
-------------------

Create Resource Locator
^^^^^^^^^^^^^^^^^^^^^^^

Because this is ROS agnostic you need to provide a resource locator for interpreting **package:/**.

.. literalinclude:: ../../tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:1: Define a resource locator function
   :end-before: documentation:end:1: Define a resource locator function

Load URDF
^^^^^^^^^

Get the file path to the urdf file

.. literalinclude:: ../../tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:2: Get the urdf file path
   :end-before: // documentation:end:2: Get the urdf file path

Create scene graph from urdf

.. literalinclude:: ../../tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:3: Create scene graph
   :end-before: // documentation:end:3: Create scene graph

Print information about the scene graph to the terminal

.. literalinclude:: ../../tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:4: Print information
   :end-before: // documentation:end:4: Print information

Save the graph to a file.

.. literalinclude:: ../../tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:5: Save graph
   :end-before: // documentation:end:5: Save graph

.. _ex3:

Parse SRDF adding Allowed Collision Matrix to Graph
===================================================

.. literalinclude:: ../../tesseract_scene_graph/examples/parse_srdf_example.cpp

Example Explanation
-------------------

Create Scene Graph
^^^^^^^^^^^^^^^^^^
.. literalinclude:: ../../tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:1: Create scene graph
   :end-before: // documentation:end:1: Create scene graph


Load SRDF
^^^^^^^^^^^^^^^^^^

Get the file path to the SRDF file

.. literalinclude:: ../../tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:2: Get the srdf file path
   :end-before: // documentation:end:2: Get the srdf file path

Parse SRDF

.. literalinclude:: ../../tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:3: Parse the srdf
   :end-before: // documentation:end:3: Parse the srdf

Add Allowed Collision Matrix to Scene Graph

.. literalinclude:: ../../tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:4: Add allowed collision matrix to scene graph
   :end-before: // documentation:end:4: Add allowed collision matrix to scene graph

Methods for getting Allowed Collision Matrix from Scene Graph

.. literalinclude:: ../../tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:5: Get info about allowed collision matrix
   :end-before: // documentation:end:5: Get info about allowed collision matrix

.. _ex4:

Parse Mesh from file
====================

.. literalinclude:: ../../tesseract_geometry/examples/parse_mesh_example.cpp

Example Explanation
-------------------

Parse Mesh from File
^^^^^^^^^^^^^^^^^^^^

Mesh files can contain multiple meshes. This is a critical difference between MoveIt! which merges all shapes in to a single triangle list for collision checking. By keeping each mesh independent, each will have its own bounding box and if you want to convert to a convex hull you will get a closer representation of the geometry.

.. literalinclude:: ../../tesseract_geometry/examples/parse_mesh_example.cpp
   :language: c++
   :start-after: // documentation:start:1: Create meshes
   :end-before: // documentation:end:1: Create meshes

Print Mesh Information to Terminal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_geometry/examples/parse_mesh_example.cpp
   :language: c++
   :start-after: // documentation:start:2: Print mesh information
   :end-before: // documentation:end:2: Print mesh information
