/**
 * @page box_box_example Tesseract Collision Checker Example
 *
 * This example shows how to use Tesseract collision checker
 * to perform discrete collision and distance queries between simple shapes.
 *
 * It demonstrates how to:
 *
 * - Create a Bullet-based discrete BVH collision manager
 * - Add primitive box and convex-hull collision shapes
 * - Configure active collision objects and collision margins
 * - Set object transforms
 * - Query for closest distances and interpret the results
 *
 * @section box_box_example_manager Creating the collision manager
 *
 * First, create the Bullet discrete BVH collision manager. This object owns the
 * underlying Bullet broadphase data structures and provides the main API for
 * adding collision objects and performing contact tests.
 *
 * @snippet box_box_example.cpp collision_example_create_manager
 *
 * @section box_box_example_add_primitive Adding primitive boxes
 *
 * Add a unit box (1 x 1 x 1) as a collision object:
 *
 * @snippet box_box_example.cpp collision_example_add_box
 *
 * We also add a second, thinner box (0.1 x 1 x 1), but leave it disabled so it
 * does not participate in collision queries:
 *
 * @snippet box_box_example.cpp collision_example_add_thin_box
 *
 * @section box_box_example_convex_hull Adding a convex-hull object
 *
 * Next, we load a mesh from a PLY file, convert it to a convex hull, and add it
 * as another collision object:
 *
 * @snippet box_box_example.cpp collision_example_add_convex_hull
 * @snippet box_box_example.cpp collision_example_add_convex_hull_collision
 *
 * @section box_box_example_setup Setting active objects and margin
 *
 * Before running a collision check, choose which objects should be active and
 * configure the collision margin:
 *
 * @snippet box_box_example.cpp collision_example_set_active_objects
 * @snippet box_box_example.cpp collision_example_set_margin_data
 *
 * @section box_box_example_initial_poses Setting initial transforms
 *
 * Set the initial transforms for the active objects:
 *
 * @snippet box_box_example.cpp collision_example_set_transforms_initial
 *
 * @section box_box_example_first_check First collision/distance check
 *
 * Perform a closest-distance query and inspect the resulting distance, nearest
 * points, and normal:
 *
 * @snippet box_box_example.cpp collision_example_first_check
 *
 * @section box_box_example_move_outside Moving out of contact distance
 *
 * Move one object further away so it lies outside the current contact distance
 * threshold and repeat the check:
 *
 * @snippet box_box_example.cpp collision_example_move_outside_contact_distance
 * @snippet box_box_example.cpp collision_example_second_check
 *
 * @section box_box_example_increase_margin Increasing the contact distance
 *
 * Increase the default collision margin and run another check to see how the
 * results change:
 *
 * @snippet box_box_example.cpp collision_example_change_default_margin
 * @snippet box_box_example.cpp collision_example_third_check
 *
 * @section box_box_example_summary Summary
 *
 * This example demonstrates the core workflow when using
 * `BulletDiscreteBVHManager`:
 *
 * - Create a manager and register your collision objects (primitive and/or convex hull)
 * - Choose which objects are active for a given query
 * - Set collision margins and object transforms
 * - Call `contactTest` and interpret the results to make decisions in your
 *   planner or controller
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
