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

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp


Example Explanation
-------------------

Create Scene Graph
^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Create scene graph
   :end-before: // Create links

Add Links
^^^^^^^^^

Create the links. The links are able to be configured see Link documentation.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
  :language: c++
  :start-after: // Create links
  :end-before: // Add links

Add the links to the scene graph

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Add links
   :end-before: // Create joints

Add Joints
^^^^^^^^^^

Create the joints. The links are able to be configured see Joint documentation.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Create joints
   :end-before: // Add joints

Add the joints to the scene graph_acyclic_tree_example

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Add joints
   :end-before: // Check getAdjacentLinkNames Method

Inspect Scene Graph
^^^^^^^^^^^^^^^^^^^

Get the adjacent links for **link_3** and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Check getAdjacentLinkNames Method
   :end-before: // Check getInvAdjacentLinkNames Method

Get the inverse adjacent links for **link_3** and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Check getInvAdjacentLinkNames Method
   :end-before: // Check getLinkChildrenNames

Get child link names for link **link_3** and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Check getLinkChildrenNames
   :end-before: // Check getJointChildrenNames

Get child link names for joint **joint_1** and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Check getJointChildrenNames
   :end-before: // Save Graph

Save the graph to a file for visualization

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Save Graph
   :end-before: // Test if the graph is Acyclic

Test if the graph is Acyclic and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Test if the graph is Acyclic
   :end-before: // Test if the graph is Tree

Test if the graph is a tree and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Test if the graph is Tree
   :end-before: // Test for unused links

Detect Unused Links
^^^^^^^^^^^^^^^^^^^

First add a link but do not create joint and check if it is a tree. It should return false because the link is not associated with a joint.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Test for unused links
   :end-before: // Remove unused link

Remove link and check if it is a tree. It should return true.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Remove unused link
   :end-before: // Add new joint

Create Acyclic Graph
^^^^^^^^^^^^^^^^^^^^

Add joint connecting **link_5** and **link_4** to create an Acyclic graph_acyclic_tree_example

.. image:: ../_static/tesseract_scene_graph_graph.png

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Add new joint
   :end-before: // Save new graph

Save the Acyclic graph

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Save new graph
   :end-before: // Test again if the graph is Acyclic

Test to confirm it is acyclic, should return true.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Test again if the graph is Acyclic
   :end-before: // Test again if the graph is Tree

Test if it is a tree, should return false.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Test again if the graph is Tree
   :end-before: // Get Shortest Path

Get Shortest Path
^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // Get Shortest Path
   :end-before: }

.. _ex2:

Create Scene Graph from URDF
============================

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/load_urdf_example.cpp

Example Explanation
-------------------

Create Resource Locator
^^^^^^^^^^^^^^^^^^^^^^^

Because this is ROS agnostic you need to provide a resource locator for interpreting **package:/**.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // Define a resource locator function
   :end-before: int main(int /*argc*/, char** /*argv*/)

Load URDF
^^^^^^^^^

Get the file path to the urdf file

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // Get the urdf file path
   :end-before: // Create scene graph

Create scene graph from urdf

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // Create scene graph
   :end-before: // Print information

Print information about the scene graph to the terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // Print information
   :end-before: // Save graph

Save the graph to a file.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // Save graph
   :end-before: }

.. _ex3:

Parse SRDF adding Allowed Collision Matrix to Graph
===================================================

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp

Example Explanation
-------------------

Create Resource Locator
^^^^^^^^^^^^^^^^^^^^^^^

Because this is ROS agnostic you need to provide a resource locator for interpreting **package:/**.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // Define a resource locator function
   :end-before: int main(int /*argc*/, char** /*argv*/)

Load URDF and SRDF
^^^^^^^^^^^^^^^^^^

Get the file path to the URDF and SRDF file

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // Get the urdf and srdf file paths
   :end-before: // Create scene graph

Create Scene Graph from URDF

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // Create scene graph
   :end-before: // Parse the srdf

Parse SRDF

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // Parse the srdf
   :end-before: // Add allowed collision matrix to scene graph

Add Allowed Collision Matrix to Scene Graph

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // Add allowed collision matrix to scene graph
   :end-before: // Get info about allowed collision matrix

Methods for getting Allowed Collision Matrix from Scene Graph

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // Get info about allowed collision matrix
   :end-before: const AllowedCollisionMatrix::AllowedCollisionEntries& acm_entries = acm->getAllAllowedCollisions();

.. _ex4:

Parse Mesh from file
====================

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_mesh_example.cpp

Example Explanation
-------------------

Parse Mesh from File
^^^^^^^^^^^^^^^^^^^^

Mesh files can contain multiple meshes. This is a critical difference between MoveIt which merges all shapes in to a single triangle list for collision checking. By keeping each mesh independent, each will have its own bounding box and if you want to convert to a convex hull you will get a closer representation of the geometry.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_mesh_example.cpp
   :language: c++
   :start-after: // Create meshes
   :end-before: // Print mesh information

Print Mesh Information to Terminal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_mesh_example.cpp
   :language: c++
   :start-after: // Print mesh information
   :end-before: }
