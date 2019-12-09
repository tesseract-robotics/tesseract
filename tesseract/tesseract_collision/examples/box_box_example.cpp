#include <console_bridge/console.h>
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

std::string toString(const Eigen::MatrixXd& a)
{
  std::stringstream ss;
  ss << a;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  // Create Collision Manager
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;

  // Add box to checker
  CollisionShapePtr box(new Box(1, 1, 1));
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses);

  // Add thin box to checker which is disabled
  CollisionShapePtr thin_box(new Box(0.1, 1, 1));
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, false);

  // Add second box to checker, but convert to convex hull mesh.
  CollisionShapePtr second_box;

  tesseract_common::VectorVector3d mesh_vertices;
  Eigen::VectorXi mesh_faces;
  loadSimplePlyFile(std::string(DATA_DIR) + "/box_2m.ply", mesh_vertices, mesh_faces);

  // This is required because convex hull cannot have multiple faces on the same plane.
  std::shared_ptr<tesseract_common::VectorVector3d> ch_verticies(new tesseract_common::VectorVector3d());
  std::shared_ptr<Eigen::VectorXi> ch_faces(new Eigen::VectorXi());
  int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
  second_box.reset(new ConvexMesh(ch_verticies, ch_faces, ch_num_faces));

  Eigen::Isometry3d second_box_pose;
  second_box_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(second_box);
  obj3_poses.push_back(second_box_pose);

  checker.addCollisionObject("second_box_link", 0, obj3_shapes, obj3_poses);

  CONSOLE_BRIDGE_logInform("Test when object is inside another");
  checker.setActiveCollisionObjects({ "box_link", "second_box_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["box_link"].translation()(0) = 0.2;
  location["box_link"].translation()(1) = 0.1;
  location["second_box_link"] = Eigen::Isometry3d::Identity();

  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::CLOSEST);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

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

  CONSOLE_BRIDGE_logInform("Test object is out side the contact distance");
  location["box_link"].translation() = Eigen::Vector3d(1.60, 0, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  checker.setCollisionObjectsTransform(location);

  // Check for collision after moving object
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());

  CONSOLE_BRIDGE_logInform("Test object inside the contact distance");
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  // Set higher contact distance threshold
  checker.setContactDistanceThreshold(0.25);

  // Check for contact with new threshold
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

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
}
