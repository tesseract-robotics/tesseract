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
   :lines: 23

Add Links
^^^^^^^^^

Create the links. The links are able to be configured see Link documentation.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
  :language: c++
  :lines: 25-29

Add the links to the scene graph

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 31-35

Add Joints
^^^^^^^^^^

Create the joints. The links are able to be configured see Joint documentation.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 37-59

Add the joints to the scene graph_acyclic_tree_example

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 61-64

Inspect Scene Graph
^^^^^^^^^^^^^^^^^^^

Get the adjacent links for **link_3** and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 67-69

Get the inverse adjacent links for **link_3** and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 72-74

Get child link names for link **link_3** and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 77-79

Get child link names for joint **joint_1** and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 82-84

Save the graph to a file for visualization

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 87

Test if the graph is Acyclic and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 90-91

Test if the graph is a tree and print to terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 94-95

Detect Unused Links
^^^^^^^^^^^^^^^^^^^

First add a link but do not create joint and check if it is a tree. It should return false because the link is not associated with a joint.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 98-101

Remove link and check if it is a tree. It should return true.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 103-105

Create Acyclic Graph
^^^^^^^^^^^^^^^^^^^^

Add joint connecting **link_5** and **link_4** to create an Acyclic graph_acyclic_tree_example

.. image:: ../_static/tesseract_scene_graph_graph.png

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 107-112

Save the Acyclic graph

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 115

Test to confirm it is acyclic, should return true.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 118-119

Test if it is a tree, should return false.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 122-123

Get Shortest Path
^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 126-127

.. _ex2:
Create Scene Graph from URDF
============================

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/load_urdf_example.cpp

Example Explanation
-------------------

Create Resource Locator
^^^^^^^^^^^^^^^^^^^^^^^

Because this is ROS agnostic you need to provide a resource locator for interpreting **package:/**.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 20-42

Load URDF
^^^^^^^^^

Get the file path to the urdf file

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 49

Create scene graph from urdf

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 51-52

Print information about the scene graph to the terminal

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 54-57

Save the graph to a file.

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/build_scene_graph_example.cpp
   :language: c++
   :lines: 60

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
   :lines: 22-47

Load URDF and SRDF
^^^^^^^^^^^^^^^^^^

Get the file path to the URDF and SRDF file

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :lines: 51-52

Create Scene Graph from URDF

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :lines: 54-55

Parse SRDF

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :lines: 57-59

Add Allowed Collision Matrix to Scene Graph

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :lines: 61

Methods for getting Allowed Collision Matrix from Scene Graph

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_srdf_example.cpp
   :language: c++
   :lines: 63-64

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
   :lines: 10-11

Print Mesh Information to Terminal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_scene_graph/examples/parse_mesh_example.cpp
   :language: c++
   :lines: 13-17
