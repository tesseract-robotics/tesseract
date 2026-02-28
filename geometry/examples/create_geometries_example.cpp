/**
 * @page create_geometries_example Tesseract Geometry Creation Example
 *
 * @section create_geometries_overview Overview
 *
 * This example demonstrates how to create and instantiate various geometry types
 * in Tesseract. Geometries are the building blocks for collision objects, which are
 * used in motion planning, simulation, and collision detection. Understanding the
 * different geometry types and when to use each one is fundamental to working with
 * Tesseract.
 *
 * @section create_geometries_concepts Key Concepts
 *
 * - **Primitive Shapes**: Simple, analytically-defined shapes (Box, Sphere, Cylinder, etc.)
 *   that are computationally efficient for collision checking.
 *
 * - **Mesh Geometries**: Triangle-based representations of arbitrary 3D shapes, loaded
 *   from files or constructed manually. More flexible but slower than primitives.
 *
 * - **Signed Distance Field (SDF) Mesh**: A specialized mesh that stores per-vertex or
 *   per-cell distance information. Useful for faster proximity queries and haptic feedback.
 *
 * - **Convex Mesh**: A mesh guaranteed to be convex (all interior angles < 180°). More
 *   efficient for collision checking than arbitrary meshes while maintaining flexibility.
 *
 * - **Octree**: A hierarchical spatial data structure that represents volumetric regions,
 *   useful for voxel-based collision detection and occupancy information.
 *
 * @section create_geometries_when_to_use When to Use Each Type
 *
 * | Geometry Type | Speed | Flexibility | Use Case |
 * |---------------|-------|-------------|----------|
 * | Primitive | Very Fast | Low | Robot links, obstacles with simple shapes |
 * | Mesh | Slow | Very High | Complex geometry, CAD models |
 * | SDF Mesh | Medium | High | Smooth contact, force feedback |
 * | Convex Mesh | Fast | Medium | Complex convex objects, gripper fingers |
 * | Octree | Medium | Medium | Point clouds, voxel grids, occupancy maps |
 *
 * @section create_geometries_primitives Creating Primitive Shapes
 *
 * Primitive shapes are created with a single constructor call. They are analytically
 * defined and are the fastest for collision checking:
 *
 * **Box** (width × height × depth):
 * @snippet create_geometries_example.cpp create_geometries_box
 *
 * **Sphere** (radius):
 * @snippet create_geometries_example.cpp create_geometries_sphere
 *
 * **Cylinder** (radius × height):
 * @snippet create_geometries_example.cpp create_geometries_cylinder
 *
 * **Cone** (radius × height):
 * @snippet create_geometries_example.cpp create_geometries_cone
 *
 * **Capsule** (radius × height):
 * @snippet create_geometries_example.cpp create_geometries_capsule
 *
 * **Plane** (a·x + b·y + c·z + d = 0):
 * @snippet create_geometries_example.cpp create_geometries_plane
 *
 * @section create_geometries_from_file Loading Meshes from Files
 *
 * The most practical way to use meshes is to load them from CAD files (DAE, STL, OBJ, etc.).
 * Tesseract provides `createMeshFromPath()` which automatically parses the file format and
 * returns a vector of meshes (some files may contain multiple mesh objects):
 *
 * @snippet create_geometries_example.cpp create_geometries_load_mesh
 *
 * You can then inspect the loaded meshes and use them in collision checking:
 *
 * @snippet create_geometries_example.cpp create_geometries_mesh_info
 *
 * @section create_geometries_mesh Creating Mesh Geometries Manually
 *
 * Mesh geometries require you to provide vertex positions and face indices (triangles).
 * Vertices are 3D points, and faces are groups of 3 indices defining triangles:
 *
 * **Standard Mesh** - Use for complex geometry loaded from CAD files:
 * @snippet create_geometries_example.cpp create_geometries_mesh
 *
 * **Signed Distance Field (SDF) Mesh** - Use when you need distance information at vertices:
 * @snippet create_geometries_example.cpp create_geometries_sdf_mesh
 *
 * **Convex Mesh** - Use when your geometry should be convex (more efficient):
 * @snippet create_geometries_example.cpp create_geometries_convex_mesh
 *
 * @section create_geometries_octree Creating Octrees
 *
 * An octree represents volumetric space hierarchically. It's useful when working with
 * point clouds, occupancy grids, or voxel-based representations:
 *
 * @snippet create_geometries_example.cpp create_geometries_octree
 *
 * @section create_geometries_typical_workflow Typical Workflow
 *
 * 1. **Choose the geometry type** based on the shape and performance requirements
 * 2. **Create the geometry** with appropriate parameters (see sections above)
 * 3. **Add to collision manager** - Pass the geometry to `addCollisionObject()` (see box_box_example)
 * 4. **Use in collision checks** - Query distances, penetration, contacts
 * 5. **Use in planning** - Feed collision results to motion planners
 *
 * @section create_geometries_tips Tips and Best Practices
 *
 * - **Prefer primitives**: Always use primitives when possible (faster and simpler)
 * - **Convex hulls for complex shapes**: If you have a complex mesh, compute its convex hull
 *   for better performance
 * - **Scale appropriately**: Geometry parameters should match your robot's dimensions in meters
 * - **Mesh resolution**: Complex meshes are slower; balance between fidelity and performance
 * - **Octree resolution**: Choose appropriate octree depth based on your precision needs
 * - **Shared ownership**: Use `std::shared_ptr` for memory management (shown in all examples)
 *
 * @section create_geometries_integration Integrating with Collision Checking
 *
 * After creating geometries, use them with the collision manager:
 * @code
 * BulletDiscreteBVHManager checker;
 * checker.addCollisionObject("my_object", 0, {box}, {Eigen::Isometry3d::Identity()});
 * @endcode
 *
 * See the @ref box_box_example "box_box_example" page for complete collision checking examples.
 *
 * @section create_geometries_summary Summary
 *
 * This example demonstrates creating all major geometry types in Tesseract:
 *
 * - Common primitives: Box, Sphere, Cylinder, Cone, Capsule, Plane
 * - Mesh loading from files (practical for real-world usage)
 * - Mesh variants: Standard, SDF, and Convex meshes
 * - Volumetric representation: Octrees
 *
 * Choose the simplest geometry that fits your use case for optimal performance.
 *
 * @section create_geometries_full_code Full source code
 *
 * For reference, here is the complete source of this example:
 *
 * @snippet create_geometries_example.cpp create_geometries_full_source
 */

//! [create_geometries_full_source]
#include <console_bridge/console.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/mesh_parser.h>
#include <tesseract/common/resource_locator.h>
#include <octomap/OcTree.h>
#include <iostream>

using namespace tesseract::geometry;

int main(int /*argc*/, char** /*argv*/)
{
  //! [create_geometries_load_mesh]
  tesseract::common::GeneralResourceLocator locator;
  std::string mesh_file = "package://tesseract/support/meshes/sphere_p25m.dae";
  std::vector<Mesh::Ptr> loaded_meshes = createMeshFromPath<Mesh>(locator.locateResource(mesh_file)->getFilePath());
  //! [create_geometries_load_mesh]

  //! [create_geometries_mesh_info]
  CONSOLE_BRIDGE_logInform("Number of meshes loaded: %zu", loaded_meshes.size());
  for (size_t i = 0; i < loaded_meshes.size(); ++i)
  {
    CONSOLE_BRIDGE_logInform("Mesh #%zu - Triangles: %zu, Vertices: %zu",
                             i + 1,
                             loaded_meshes[i]->getFaceCount(),
                             loaded_meshes[i]->getVertexCount());
  }
  //! [create_geometries_mesh_info]

  //! [create_geometries_box]
  auto box = std::make_shared<tesseract::geometry::Box>(1, 1, 1);
  //! [create_geometries_box]

  //! [create_geometries_cone]
  auto cone = std::make_shared<tesseract::geometry::Cone>(1, 1);
  //! [create_geometries_cone]

  //! [create_geometries_capsule]
  auto capsule = std::make_shared<tesseract::geometry::Capsule>(1, 1);
  //! [create_geometries_capsule]

  //! [create_geometries_cylinder]
  auto cylinder = std::make_shared<tesseract::geometry::Cylinder>(1, 1);
  //! [create_geometries_cylinder]

  //! [create_geometries_plane]
  auto plane = std::make_shared<tesseract::geometry::Plane>(1, 1, 1, 1);
  //! [create_geometries_plane]

  //! [create_geometries_sphere]
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(1);
  //! [create_geometries_sphere]

  //! [create_geometries_mesh]
  // Manually create mesh (advanced: usually you load from file instead)
  std::shared_ptr<const tesseract::common::VectorVector3d> mesh_vertices =
      std::make_shared<const tesseract::common::VectorVector3d>();
  std::shared_ptr<const Eigen::VectorXi> mesh_faces = std::make_shared<const Eigen::VectorXi>();
  // Next fill out vertices and triangles
  auto mesh = std::make_shared<tesseract::geometry::Mesh>(mesh_vertices, mesh_faces);
  //! [create_geometries_mesh]

  //! [create_geometries_sdf_mesh]
  // Manually create signed distance field mesh
  std::shared_ptr<const tesseract::common::VectorVector3d> sdf_vertices =
      std::make_shared<const tesseract::common::VectorVector3d>();
  std::shared_ptr<const Eigen::VectorXi> sdf_faces = std::make_shared<const Eigen::VectorXi>();
  // Next fill out vertices and triangles
  auto sdf_mesh = std::make_shared<tesseract::geometry::SDFMesh>(sdf_vertices, sdf_faces);
  //! [create_geometries_sdf_mesh]

  //! [create_geometries_convex_mesh]
  // Manually create convex mesh
  std::shared_ptr<const tesseract::common::VectorVector3d> convex_vertices =
      std::make_shared<const tesseract::common::VectorVector3d>();
  std::shared_ptr<const Eigen::VectorXi> convex_faces = std::make_shared<const Eigen::VectorXi>();
  // Next fill out vertices and triangles
  auto convex_mesh = std::make_shared<tesseract::geometry::ConvexMesh>(convex_vertices, convex_faces);
  //! [create_geometries_convex_mesh]

  //! [create_geometries_octree]
  // Create an octree
  std::shared_ptr<const octomap::OcTree> octree;
  auto octree_t = std::make_shared<tesseract::geometry::Octree>(octree, tesseract::geometry::OctreeSubType::BOX);
  //! [create_geometries_octree]
}
//! [create_geometries_full_source]
