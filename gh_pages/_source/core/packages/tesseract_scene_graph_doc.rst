*****************************
Tesseract Scene Graph Package
*****************************

Background
==========
This package contains the scene graph and parsers. The scene graph is used to manage the connectivity of the environment. The scene graph inherits from boost graph so you are able to leverage boost graph utilities for searching.

Scene Graph (Tree)
------------------

.. image:: /_static/tesseract_scene_graph_tree.png
   :alt: Scene Graph (Tree)

Scene Graph (Acyclic)
---------------------

.. image:: /_static/tesseract_scene_graph_graph.png
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

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++

You can find this example `https://github.com/ros-industrial-consortium/tesseract/blob/master/tesseract_scene_graph/examples/build_scene_graph_example.cpp <https://github.com/ros-industrial-consortium/tesseract/blob/master/tesseract_scene_graph/examples/build_scene_graph_example.cpp>`_

Example Explanation
-------------------

Create Scene Graph
^^^^^^^^^^^^^^^^^^

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:1:
   :end-before: // documentation:end:1:

Add Links
^^^^^^^^^

Create the links. The links are able to be configured see Link documentation.

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
  :language: c++
  :start-after: // documentation:start:2:
  :end-before: // documentation:end:2:

Add the links to the scene graph

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:3:
   :end-before: // documentation:end:3:

Add Joints
^^^^^^^^^^

Create the joints. The links are able to be configured see Joint documentation.

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:4:
   :end-before: // documentation:end:4:

Add the joints to the scene graph_acyclic_tree_example

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:5:
   :end-before: // documentation:end:5:

Inspect Scene Graph
^^^^^^^^^^^^^^^^^^^

Get the adjacent links for **link_3** and print to terminal

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:6:
   :end-before: // documentation:end:6:

Get the inverse adjacent links for **link_3** and print to terminal

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:7:
   :end-before: // documentation:end:7:

Get child link names for link **link_3** and print to terminal

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:8:
   :end-before: // documentation:end:8:

Get child link names for joint **joint_1** and print to terminal

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:9:
   :end-before: // documentation:end:9:

Save the graph to a file for visualization

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:10:
   :end-before: // documentation:end:10:

Test if the graph is Acyclic and print to terminal

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:11:
   :end-before: // documentation:end:11:

Test if the graph is a tree and print to terminal

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:12:
   :end-before: // documentation:end:12:

Detect Unused Links
^^^^^^^^^^^^^^^^^^^

First add a link but do not create joint and check if it is a tree. It should return false because the link is not associated with a joint.

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:13:
   :end-before: // documentation:end:13:

Remove link and check if it is a tree. It should return true.

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:14:
   :end-before: // documentation:end:14:

Create Acyclic Graph
^^^^^^^^^^^^^^^^^^^^

Add joint connecting **link_5** and **link_4** to create an Acyclic graph_acyclic_tree_example

.. image:: /_static/tesseract_scene_graph_graph.png

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:15:
   :end-before: // documentation:end:15:

Save the Acyclic graph

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:16:
   :end-before: // documentation:end:16:

Test to confirm it is acyclic, should return true.

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:17:
   :end-before: // documentation:end:17:

Test if it is a tree, should return false.

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:18:
   :end-before: // documentation:end:18:

Get Shortest Path
^^^^^^^^^^^^^^^^^

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :start-after: // documentation:start:19:
   :end-before: // documentation:end:19:

Running the Example
-------------------

Build the Tesseract Workspace: ::

  catkin build

Navigate to the build folder containing the executable: ::

  cd build/tesseract_scene_graph/examples

Run the executable: ::

  ./tesseract_scene_graph_build_graph_example

.. _ex2:

Create Scene Graph from URDF
============================

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++

You can find this example `https://github.com/ros-industrial-consortium/tesseract/blob/master/tesseract_urdf/examples/load_urdf_example.cpp <https://github.com/ros-industrial-consortium/tesseract/blob/master/tesseract_urdf/examples/load_urdf_example.cpp>`_

Example Explanation
-------------------

Create Resource Locator
^^^^^^^^^^^^^^^^^^^^^^^

Because this is ROS agnostic you need to provide a resource locator for interpreting **package:/**.

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:1:
   :end-before: documentation:end:1:

Load URDF
^^^^^^^^^

Get the file path to the urdf file

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:2:
   :end-before: // documentation:end:2:

Create scene graph from urdf

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:3:
   :end-before: // documentation:end:3:

Print information about the scene graph to the terminal

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:4:
   :end-before: // documentation:end:4:

Save the graph to a file.

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_urdf/examples/load_urdf_example.cpp
   :language: c++
   :start-after: // documentation:start:5:
   :end-before: // documentation:end:5:

Running the Example
-------------------

Build the Tesseract Workspace: ::

  catkin build

Navigate to the build folder containing the executable: ::

  cd build/tesseract_urdf/examples

Run the executable: ::

  ./tesseract_urdf_load_urdf_example

.. _ex3:

Parse SRDF adding Allowed Collision Matrix to Graph
===================================================

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_srdf/examples/parse_srdf_example.cpp
   :language: c++

You can find this example at `https://github.com/ros-industrial-consortium/tesseract/blob/master/tesseract_srdf/examples/parse_srdf_example.cpp <https://github.com/ros-industrial-consortium/tesseract/blob/master/tesseract_srdf/examples/parse_srdf_example.cpp>`_

Example Explanation
-------------------

Create Scene Graph
^^^^^^^^^^^^^^^^^^
.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_srdf/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:1:
   :end-before: // documentation:end:1:


Load SRDF
^^^^^^^^^^^^^^^^^^

Get the file path to the SRDF file

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_srdf/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:2:
   :end-before: // documentation:end:2:

Parse SRDF

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_srdf/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:3:
   :end-before: // documentation:end:3:

Add Allowed Collision Matrix to Scene Graph

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_srdf/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:4:
   :end-before: // documentation:end:4:

Methods for getting Allowed Collision Matrix from Scene Graph

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_srdf/examples/parse_srdf_example.cpp
   :language: c++
   :start-after: // documentation:start:5:
   :end-before: // documentation:end:5:

.. _ex4:

Parse Mesh from file
====================

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_geometry/examples/parse_mesh_example.cpp

Example Explanation
-------------------

Parse Mesh from File
^^^^^^^^^^^^^^^^^^^^

Mesh files can contain multiple meshes. This is a critical difference between MoveIt! which merges all shapes in to a single triangle list for collision checking. By keeping each mesh independent, each will have its own bounding box and if you want to convert to a convex hull you will get a closer representation of the geometry.

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_geometry/examples/parse_mesh_example.cpp
   :language: c++
   :start-after: // documentation:start:1:
   :end-before: // documentation:end:1:

Print Mesh Information to Terminal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. rli:: https://raw.githubusercontent.com/steviedale/tesseract/update/tesseract_documentation/tesseract_geometry/examples/parse_mesh_example.cpp
   :language: c++
   :start-after: // documentation:start:2:
   :end-before: // documentation:end:2:

Running the Example
-------------------

Build the Tesseract Workspace: ::

  catkin build

Navigate to the build folder containing the executable: ::

  cd build/tesseract_geometry/examples

Run the executable: ::

  ./tesseract_geometry_parse_mesh_example
