/**
 * @page box_box_example Tesseract Collision Checker Example
 *
 * @section box_box_example_overview Overview
 *
 * This example demonstrates how to use Tesseract's collision checking capabilities
 * to detect when objects are in contact or approaching each other. Collision checking
 * is essential for motion planning, robot simulation, and ensuring safe trajectories.
 *
 * The example uses the Bullet discrete BVH (Bounding Volume Hierarchy) collision
 * manager, which efficiently handles proximity queries between rigid bodies.
 *
 * @section box_box_example_concepts Key Concepts
 *
 * - **Collision Manager**: The core object that manages collision objects and performs
 *   queries. A collision object consists of one or more collision shapes (primitives
 *   or meshes) at specified poses.
 *
 * - **Collision Margin (Contact Distance)**: The distance threshold at which objects
 *   are considered "in contact". This is useful for safety buffers—you might want to
 *   plan trajectories that maintain a 5cm buffer from obstacles.
 *
 * - **Active Objects**: Only active objects participate in collision tests. This lets
 *   you enable/disable objects dynamically without removing them from the manager.
 *
 * - **Contact Test Type**:
 *   - `ContactTestType::CLOSEST`: Find the single closest pair of points between objects
 *   - `ContactTestType::FIRST`: Stop at the first overlap detected (faster)
 *   - Other types support multiple contacts and filtered queries
 *
 * - **Contact Results**: Include distance (negative = penetrating), nearest points on
 *   each object, and a normal vector pointing from object A to object B.
 *
 * @section box_box_example_workflow Typical Workflow
 *
 * It demonstrates how to:
 *
 * - Create a Bullet-based discrete BVH collision manager
 * - Add primitive box and convex-hull collision shapes with unique identifiers
 * - Configure which objects participate in queries (active objects)
 * - Set collision margins for safety buffers
 * - Transform objects in 3D space
 * - Query for closest distances and interpret the results
 * - Use results to make planning or control decisions
 *
 * @section box_box_example_manager Creating the collision manager
 *
 * First, create the Bullet discrete BVH collision manager. This object owns the
 * underlying Bullet broadphase data structures and provides the main API for
 * adding collision objects and performing contact tests.
 *
 * @snippet box_box_example.cpp collision_example_create_manager
 *
 * @section box_box_example_add_primitive Adding collision objects
 *
 * Add a unit box (1 x 1 x 1) as collision object "box_link":
 *
 * @snippet box_box_example.cpp collision_example_add_box
 *
 * **Note**: The last parameter (default `true`) controls whether the object is initially
 * active. We add a second, thinner box (0.1 x 1 x 1) with `enabled=false` so it doesn't
 * participate in collision queries until we activate it later:
 *
 * @snippet box_box_example.cpp collision_example_add_thin_box
 *
 * @section box_box_example_convex_hull Adding convex-hull objects
 *
 * For complex mesh geometry, Tesseract can compute a convex hull approximation, which
 * is more efficient for collision checking than the full mesh. Here, we load a PLY
 * mesh file and convert it to a convex hull:
 *
 * @snippet box_box_example.cpp collision_example_add_convex_hull
 *
 * Then add it to the collision manager as a collision object:
 *
 * @snippet box_box_example.cpp collision_example_add_convex_hull_collision
 *
 * @section box_box_example_setup Configuring query parameters
 *
 * Before querying, you must choose which objects should be queried. This reduces
 * unnecessary computation:
 *
 * @snippet box_box_example.cpp collision_example_set_active_objects
 *
 * Set a collision margin. A 0.1m margin means objects are considered "in contact"
 * when they are within 0.1m of each other:
 *
 * @snippet box_box_example.cpp collision_example_set_margin_data
 *
 * @section box_box_example_initial_poses Setting object transforms
 *
 * Update the 3D poses of collision objects. This doesn't modify the objects themselves—it
 * just tells the collision manager where they are located:
 *
 * @snippet box_box_example.cpp collision_example_set_transforms_initial
 *
 * @section box_box_example_first_check First query: Objects in contact
 *
 * Perform a closest-distance query with `ContactTestType::CLOSEST` to find the single
 * closest point pair. The results include:
 * - **distance**: Negative if overlapping, positive if separated
 * - **nearest_points**: The closest point on each object
 * - **normal**: Direction from object A to object B (useful for pushing objects apart)
 *
 * @snippet box_box_example.cpp collision_example_first_check
 *
 * @section box_box_example_move_outside Moving objects apart
 *
 * Move one object further away so it exceeds the contact distance threshold:
 *
 * @snippet box_box_example.cpp collision_example_move_outside_contact_distance
 *
 * Query again. Now the result will be empty (no contacts within the margin):
 *
 * @snippet box_box_example.cpp collision_example_second_check
 *
 * @section box_box_example_increase_margin Adjusting the margin
 *
 * Increase the margin to 0.25m and query again to see how objects now register as
 * "in contact" at greater distances. This is useful for conservative safe planning:
 *
 * @snippet box_box_example.cpp collision_example_change_default_margin
 * @snippet box_box_example.cpp collision_example_third_check
 *
 * @section box_box_example_interpreting Interpreting Results
 *
 * The `ContactResultVector` returned from queries provides:
 * - `link_names[0]` and `link_names[1]`: Names of the two objects
 * - `distance`: Separation distance (negative = penetration)
 * - `nearest_points[0]` and `nearest_points[1]`: 3D coordinates of closest points
 * - `normal`: Unit vector pointing from object 1 toward object 2
 *
 * Use the normal vector to determine which direction to move an object to resolve collisions.
 *
 * @section box_box_example_common_patterns Common Patterns
 *
 * **Safety-checked trajectory**: For each waypoint in a trajectory, set transforms and
 * check collisions. If any check returns contacts, reject or modify the trajectory.
 *
 * **Collision-free motion planning**: Feed collision checks to a planner algorithm
 * (e.g., RRT, PRM) to generate feasible paths.
 *
 * **Gripper/object approach**: Use decreasing margin values to gradually approach
 * objects and stop at a safe distance.
 *
 * @section box_box_example_summary Summary
 *
 * The typical workflow with `BulletDiscreteBVHManager` is:
 *
 * 1. Create a manager and register collision objects with unique names
 * 2. Choose which objects are active with `setActiveCollisionObjects()`
 * 3. Set margins and transforms
 * 4. Call `contactTest()` and examine the results
 * 5. Use distance/normal information to make decisions (plan, control, reject waypoints)
 *
 * @section box_box_example_troubleshooting Troubleshooting Tips
 *
 * - **Empty results**: Check that at least 2 objects are active and configured
 * - **Unexpected distances**: Verify object poses are in the correct coordinate frame
 * - **Performance issues**: Consider disabling objects you don't need, or using simpler
 *   shapes (primitives instead of meshes)
 *
 * @section box_box_example_full_code Full source code
 *
 * For reference, here is the complete source of this example:
 *
 * @snippet box_box_example.cpp box_box_example_full_source
 */

//! [box_box_example_full_source]
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/convex_hull_utils.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/ply_io.h>

using namespace tesseract::collision;
using namespace tesseract::geometry;

std::string toString(const Eigen::MatrixXd& a)
{
  std::stringstream ss;
  ss << a;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  //! [collision_example_create_manager]
  BulletDiscreteBVHManager checker;
  //! [collision_example_create_manager]

  //! [collision_example_add_box]
  CollisionShapePtr box = std::make_shared<Box>(1, 1, 1);
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses);
  //! [collision_example_add_box]

  //! [collision_example_add_thin_box]
  CollisionShapePtr thin_box = std::make_shared<Box>(0.1, 1, 1);
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract::common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, false);
  //! [collision_example_add_thin_box]

  //! [collision_example_add_convex_hull]
  tesseract::common::GeneralResourceLocator locator;
  CollisionShapePtr second_box;

  auto mesh_vertices = std::make_shared<tesseract::common::VectorVector3d>();
  auto mesh_faces = std::make_shared<Eigen::VectorXi>();
  tesseract::common::loadSimplePlyFile(
      locator.locateResource("package://tesseract/support/meshes/box_2m.ply")->getFilePath(),
      *mesh_vertices,
      *mesh_faces,
      true);

  auto mesh = std::make_shared<tesseract::geometry::Mesh>(mesh_vertices, mesh_faces);
  second_box = makeConvexMesh(*mesh);
  //! [collision_example_add_convex_hull]

  //! [collision_example_add_convex_hull_collision]
  Eigen::Isometry3d second_box_pose;
  second_box_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract::common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(second_box);
  obj3_poses.push_back(second_box_pose);

  checker.addCollisionObject("second_box_link", 0, obj3_shapes, obj3_poses);
  //! [collision_example_add_convex_hull_collision]

  CONSOLE_BRIDGE_logInform("Test when object is inside another");

  //! [collision_example_set_active_objects]
  checker.setActiveCollisionObjects({ "box_link", "second_box_link" });
  //! [collision_example_set_active_objects]

  //! [collision_example_set_margin_data]
  checker.setCollisionMarginData(CollisionMarginData(0.1));
  //! [collision_example_set_margin_data]

  //! [collision_example_set_transforms_initial]
  tesseract::common::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["box_link"].translation()(0) = 0.2;
  location["box_link"].translation()(1) = 0.1;
  location["second_box_link"] = Eigen::Isometry3d::Identity();

  checker.setCollisionObjectsTransform(location);
  //! [collision_example_set_transforms_initial]

  //! [collision_example_first_check]
  ContactResultMap result;
  ContactRequest request(ContactTestType::CLOSEST);
  checker.contactTest(result, request);

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());
  CONSOLE_BRIDGE_logInform("Distance: %f", result_vector[0].distance);
  CONSOLE_BRIDGE_logInform("Link %s nearest point: %s",
                           result_vector[0].link_names[0].c_str(),
                           toString(result_vector[0].nearest_points[0]).c_str());
  CONSOLE_BRIDGE_logInform("Link %s nearest point: %s",
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].nearest_points[1]).c_str());
  CONSOLE_BRIDGE_logInform("Direction to move Link %s out of collision with Link %s: %s",
                           result_vector[0].link_names[0].c_str(),
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].normal).c_str());
  //! [collision_example_first_check]

  CONSOLE_BRIDGE_logInform("Test object is out side the contact distance");

  //! [collision_example_move_outside_contact_distance]
  location["box_link"].translation() = Eigen::Vector3d(1.60, 0, 0);
  checker.setCollisionObjectsTransform(location);
  //! [collision_example_move_outside_contact_distance]

  //! [collision_example_second_check]
  result.clear();
  result_vector.clear();

  checker.contactTest(result, request);
  result.flattenMoveResults(result_vector);
  CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());
  //! [collision_example_second_check]

  //! [collision_example_change_default_margin]
  checker.setDefaultCollisionMargin(0.25);
  //! [collision_example_change_default_margin]

  //! [collision_example_third_check]
  CONSOLE_BRIDGE_logInform("Test object inside the contact distance");
  result.clear();
  result_vector.clear();

  checker.contactTest(result, request);
  result.flattenMoveResults(result_vector);

  CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());
  CONSOLE_BRIDGE_logInform("Distance: %f", result_vector[0].distance);
  CONSOLE_BRIDGE_logInform("Link %s nearest point: %s",
                           result_vector[0].link_names[0].c_str(),
                           toString(result_vector[0].nearest_points[0]).c_str());
  CONSOLE_BRIDGE_logInform("Link %s nearest point: %s",
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].nearest_points[1]).c_str());
  CONSOLE_BRIDGE_logInform("Direction to move Link %s further from Link %s: %s",
                           result_vector[0].link_names[0].c_str(),
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].normal).c_str());
  //! [collision_example_third_check]
}
//! [box_box_example_full_source]
