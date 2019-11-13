**********************
Tesseract URDF Package
**********************

Background
==========
This package contains urdf parser used by Tesseract. It supports additional shape and features not supported by urdfdom. This wiki only contains additional items and for more information please refer to http://wiki.ros.org/urdf/XML.

Features
========

#. New Shapes

   * Capsule
   * Cone
   * Mesh
   * Convex Mesh
   * SDF Mesh
   * Octomap

#. Origin

   * Quaternion

#. URDF Version

   * The original implementation of Tesseract interpreted mesh tags different than what is called version 2. It originally converted mesh geometry types to convex hull because there was no way to distinguish different types of meshes. Now in version 2 it supports the shape types (mesh, convex_mesh, sdf_mesh, etc.), therefore in version 2 the mesh tag is now interpreted as a detailed mesh and is no longer converted to a convex hull. To get the same behavior using version 2 change the tag to convex_mesh and set convert equal to true. For backwards compatibility any URDF without a version is assumed version 1 and mesh tags will be converted to convex hulls.

Change URDF Version
===================

.. code-block:: xml

   <robot name="kuka_iiwa" version="2">
   </robot>


Defining New Shapes
===================

Create Capsule
--------------

.. code-block:: xml

   <capsule radius="1" length="2"/>

The total height is the **length + 2 * radius**, so the length is just the height between the center of each sphere of the capsule caps.

Create Cone
--------------

.. code-block:: xml

   <cone radius="1" length="2"/>

The cone is like the cylinder. It is around z-axis and centered at the origin.

Create Convex Mesh
------------------

.. code-block:: xml

   <convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" convert="false"/>

This will create a convex hull shape type. This shape is more efficient than a regular mesh for collision checking. Also it provides an accurate penetration distance where in the case of mesh type you only get the penetration of one triangle into another.

.. list-table::
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Required/Optional
     - Description
   * - filename
     - Required
     - If convert is false (default) the mesh must be a convex hull represented by a polygon mesh. If it is triangulated such that multiple triangles represent the same surface you will get undefined behavior from collision checking.
   * - scale
     - Optional
     - Scales the mesh axis aligned bounding box. Default scale = [1, 1, 1].
   * - convert
     - Optional
     - If true the mesh is converted to a convex hull. Default convert = false.

Create SDF Mesh
---------------

.. code-block:: xml

   <sdf_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" />

This will create a signed distance field shape type, which only affects collision shapes. This shape is more efficient than a regular mesh for collision checking, but not as efficient as convex hull.

.. list-table::
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Required/Optional
     - Description
   * - filename
     - Required
     - A path to a convex or non-convex mesh.
   * - scale
     - Optional
     - Scales the mesh axis aligned bounding box. Default scale = [1, 1, 1].

Create Octree/Octomap
---------------------

There are two methods for creating an octomap collision object. The first is to provide and octree file (.bt | .ot) and the second option is to provide a point cloud file (.pcd) with a resolution.

.. code-block:: xml

   <octomap shape_type="box" prune="false" >
     <octree filename="package://tesseract_support/meshes/box_2m.bt"/>
   </octomap>

   <octomap shape_type="box" prune="false" >
     <point_cloud filename="package://tesseract_support/meshes/box_2m.pcd" resolution="0.1"/>
   </octomap>


This will create an octomap shape type. Each occupied cell is represented by either a box, shere outside, or sphere inside shape.

.. list-table:: Octomap Element
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Required/Optional
     - Description
   * - shape_type
     - Required
     - Currently three shape types (box, sphere_inside, sphere_outside).
   * - prune
     - Optional
     - This executes the octree toMaxLikelihood() the prune() method prior to creating shape which will combine adjacent occupied cell into larget cells resulting in fewer shapes.

.. list-table:: Octree Element
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Required/Optional
     - Description
   * - filename
     - Required
     - A path to a binary or ascii octree file.

.. list-table:: Point Cloud Element
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Required/Optional
     - Description
   * - filename
     - Required
     - A path to a PCL point clound file.
   * - resolution
     - Required
     - The resolution of the octree populated by the provided point cloud

Create Origin
-------------

.. code-block:: xml

   <origin xyz="0 0 0" rpy="0 0 0" wxyz="1 0 0 0"/>;

This allows the ability to use a quaternion instead of roll, pitch and yaw values. It is acceptable to have both to allow backwards compatability with other parsers, but the quaternion will take preference over rpy.

.. list-table::
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Required/Optional
     - Description
   * - wxyz
     - Optional
     - A Quaternion = [w, x, y, z]. It will be normalized on creation.
