#ifndef TESSERACT_COLLISION_COLLISION_COMPOUND_COMPOUND_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_COMPOUND_COMPOUND_UNIT_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision::test_suite
{
namespace detail
{
template <class T>
inline void addCollisionObjects(T& checker)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/box_2m.bt";
  auto ot = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap = std::make_shared<tesseract_geometry::Octree>(ot, tesseract_geometry::Octree::BOX);
  Eigen::Isometry3d octomap_pose;
  octomap_pose.setIdentity();
  octomap_pose.translation() = Eigen::Vector3d(1.1, 0, 0);

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);

  checker.addCollisionObject("octomap1_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////////////////////////
  // Add second octomap
  /////////////////////////////////////////////////////////////////
  auto ot_b = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap_b =
      std::make_shared<tesseract_geometry::Octree>(ot_b, tesseract_geometry::Octree::BOX);
  Eigen::Isometry3d octomap_pose_b;
  octomap_pose_b.setIdentity();
  octomap_pose_b.translation() = Eigen::Vector3d(-1.1, 0, 0);

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(dense_octomap_b);
  obj2_poses.push_back(octomap_pose_b);

  checker.addCollisionObject("octomap2_link", 0, obj2_shapes, obj2_poses);

  EXPECT_TRUE(checker.getCollisionObjects().size() == 2);
  const auto& co = checker.getCollisionObjects();
  for (std::size_t i = 0; i < co.size(); ++i)
  {
    EXPECT_TRUE(checker.getCollisionObjectGeometries(co[i]).size() == 1);
    EXPECT_TRUE(checker.getCollisionObjectGeometriesTransforms(co[i]).size() == 1);
    const auto& cgt = checker.getCollisionObjectGeometriesTransforms(co[i]);
    if (i == 0)
    {
      EXPECT_TRUE(cgt[0].isApprox(octomap_pose, 1e-5));
    }
    else
    {
      EXPECT_TRUE(cgt[0].isApprox(octomap_pose_b, 1e-5));
    }
  }
}

inline void runTestCompound(DiscreteContactManager& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "octomap1_link", "octomap2_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.25));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.25, 1e-5);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["octomap1_link"] = Eigen::Isometry3d::Identity();
  location["octomap2_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  ContactResultVector result_vector;
  flattenMoveResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  for (const auto& cr : result_vector)
  {
    EXPECT_NEAR(cr.distance, 0.20, 0.001);
  }
}

inline void runTestCompound(ContinuousContactManager& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "octomap1_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.25));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.25, 1e-5);

  // Set the collision object transforms
  Eigen::Isometry3d start_pos, end_pos;
  start_pos = Eigen::Isometry3d::Identity();
  end_pos = Eigen::Isometry3d::Identity();
  start_pos.translation() = Eigen::Vector3d(0, -2.0, 0);
  end_pos.translation() = Eigen::Vector3d(0, 2.0, 0);
  checker.setCollisionObjectsTransform("octomap1_link", start_pos, end_pos);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  ContactResultVector result_vector;
  flattenMoveResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  for (const auto& cr : result_vector)
  {
    EXPECT_NEAR(cr.distance, 0.20, 0.001);
  }
}
}  // namespace detail

inline void runTest(ContinuousContactManager& checker)
{
  detail::addCollisionObjects<ContinuousContactManager>(checker);
  detail::runTestCompound(checker);

  ContinuousContactManager::Ptr cloned_checker = checker.clone();
  detail::runTestCompound(*cloned_checker);
}

inline void runTest(DiscreteContactManager& checker)
{
  detail::addCollisionObjects<DiscreteContactManager>(checker);
  detail::runTestCompound(checker);

  DiscreteContactManager::Ptr cloned_checker = checker.clone();
  detail::runTestCompound(*cloned_checker);
}
}  // namespace tesseract_collision::test_suite
#endif  // TESSERACT_COLLISION_COLLISION_COMPOUND_COMPOUND_UNIT_HPP
