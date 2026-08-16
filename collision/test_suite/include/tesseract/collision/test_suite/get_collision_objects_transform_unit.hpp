#ifndef TESSERACT_COLLISION_GET_COLLISION_OBJECTS_TRANSFORM_UNIT_HPP
#define TESSERACT_COLLISION_GET_COLLISION_OBJECTS_TRANSFORM_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/geometry/geometries.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
inline void addTwoSpheres(DiscreteContactManager& checker)
{
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.25);

  CollisionShapesConst shapes{ sphere };
  tesseract::common::VectorIsometry3d shape_poses{ Eigen::Isometry3d::Identity() };

  checker.addCollisionObject("sphere_a", 0, shapes, shape_poses);
  checker.addCollisionObject("sphere_b", 0, shapes, shape_poses);
}

inline void addTwoSpheres(ContinuousContactManager& checker)
{
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.25);

  CollisionShapesConst shapes{ sphere };
  tesseract::common::VectorIsometry3d shape_poses{ Eigen::Isometry3d::Identity() };

  checker.addCollisionObject("sphere_a", 0, shapes, shape_poses);
  checker.addCollisionObject("sphere_b", 0, shapes, shape_poses);
}
}  // namespace detail

inline void runDiscreteGetCollisionObjectsTransformUnit(DiscreteContactManager& checker)
{
  detail::addTwoSpheres(checker);

  const tesseract::common::LinkId id_a("sphere_a");
  const tesseract::common::LinkId id_b("sphere_b");

  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  pose_a.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  pose_a.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  checker.setCollisionObjectsTransform(id_a, pose_a);

  EXPECT_TRUE(checker.getCollisionObjectsTransform(id_a).isApprox(pose_a, 1e-9));
  // sphere_b was never moved — getter should see the identity it was added with.
  EXPECT_TRUE(checker.getCollisionObjectsTransform(id_b).isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}

inline void runContinuousGetCollisionObjectsTransformUnit(ContinuousContactManager& checker)
{
  detail::addTwoSpheres(checker);

  const tesseract::common::LinkId id_a("sphere_a");
  const tesseract::common::LinkId id_b("sphere_b");

  // Mark both as active so the cast-pose overload's KinematicFilter precondition is satisfied.
  checker.setActiveCollisionObjects(std::unordered_set<tesseract::common::LinkId>{ id_a, id_b });

  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  pose_a.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  pose_a.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();

  // Discrete (single-pose) overload — getter must round-trip the pose.
  checker.setCollisionObjectsTransform(id_a, pose_a);
  EXPECT_TRUE(checker.getCollisionObjectsTransform(id_a).isApprox(pose_a, 1e-9));

  // Cast (pose1, pose2) overload — getter must return pose1, not pose2.
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(-1.0, 0.5, 0.25);
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(2.0, -0.5, 0.75);

  checker.setCollisionObjectsTransform(id_b, pose1, pose2);
  EXPECT_TRUE(checker.getCollisionObjectsTransform(id_b).isApprox(pose1, 1e-9));
  EXPECT_FALSE(checker.getCollisionObjectsTransform(id_b).isApprox(pose2, 1e-9));
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_GET_COLLISION_OBJECTS_TRANSFORM_UNIT_HPP
