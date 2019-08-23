**************************
Tesseract Geometry Package
**************************

Background
==========
This package contains geometries used by Tesseract

Features
========

#. Primitive Shapes

   * Box
   * Cone
   * Cylinder
   * Plane
   * Sphere

#. Mesh
#. Convex Mesh
#. SDF Mesh
#. Octree

Creating Geometry Shapes
========================

.. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp


Example Explanation
-------------------

#. Create a box.

   .. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp
      :language: c++
      :start-after: // Primitive Shapes
      :end-before: auto cone = std::make_shared<tesseract_geometry::Cone>(1, 1);

#. Create a cone.

   .. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp
      :language: c++
      :start-after: auto box = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
      :end-before: auto cylinder = std::make_shared<tesseract_geometry::Cylinder>(1, 1);

#. Create a cylinder.

   .. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp
      :language: c++
      :start-after: auto cone = std::make_shared<tesseract_geometry::Cone>(1, 1);
      :end-before: auto plane = std::make_shared<tesseract_geometry::Plane>(1, 1, 1, 1);

#. Create a plane.

   .. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp
      :language: c++
      :start-after: auto cylinder = std::make_shared<tesseract_geometry::Cylinder>(1, 1);
      :end-before: auto sphere = std::make_shared<tesseract_geometry::Sphere>(1);

#. Create a sphere.

   .. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp
      :language: c++
      :start-after: auto plane = std::make_shared<tesseract_geometry::Plane>(1, 1, 1, 1);
      :end-before: // Manually create mesh

#. Create a mesh.

   .. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp
      :language: c++
      :start-after: // Manually create mesh
      :end-before: // Manually create signed distance field mesh

   .. Note::

      This shows how to create a mesh provided vertices and faces. You may also use utilities in tesseract_scene_graph mesh parser to load meshes from file.

#. Create a signed distance field mesh.

   .. Note::

      This should be the same as a mesh, but when interperated as the collision object it will be encoded as a signed distance field.

   .. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp
      :language: c++
      :start-after: // Manually create signed distance field mesh
      :end-before: // Manually create convex mesh

   .. Note::

      This shows how to create a SDF mesh provided vertices and faces. You may also use utilities in tesseract_scene_graph mesh parser to load meshes from file.

#. Create a convex mesh.

   .. Warning::

      This expects the data to already represent a convex mesh. If yours does not load as a mesh and then use tesseract utility to convert to a convex mesh.

   .. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp
      :language: c++
      :start-after: // Manually create convex mesh
      :end-before: // Create an octree

   .. Note::

      This shows how to create a convex mesh provided vertices and faces. You may also use utilities in tesseract_scene_graph mesh parser to load meshes from file.

#. Create an octree.

   .. literalinclude:: ../../tesseract/tesseract_geometry/examples/create_geometries_example.cpp
      :language: c++
      :start-after: // Create an octree
      :end-before: }

   .. Note::

      It is benificial to prune the octree prior to creating the tesseract octree shap to simplify

   Octree support multiple shape types to represent a cell in the octree.

   * BOX **tesseract_geometry::Octree::SubType::BOX**
   * SPHERE_INSIDE **tesseract_geometry::Octree::SubType::SPHERE_INSIDE**
   * SPHERE_OUTSIDE **tesseract_geometry::Octree::SubType::SPHERE_OUTSIDE**
