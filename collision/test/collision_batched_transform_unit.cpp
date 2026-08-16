#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <memory>
#include <stdexcept>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/geometry/impl/box.h>

using namespace tesseract::collision;

namespace
{
template <typename ManagerType>
void addBox(ManagerType& checker, const tesseract::common::LinkId& id)
{
  CollisionShapePtr box = std::make_shared<tesseract::geometry::Box>(1, 1, 1);
  CollisionShapesConst shapes{ box };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject(id, 0, shapes, poses);
}

/** @brief Move two boxes into contact with a single one-pose parallel-array call and confirm the broadphase sees it */
template <typename ManagerType>
void runOnePoseArraySetterTest()
{
  ManagerType checker;
  addBox(checker, tesseract::common::LinkId("box_a"));
  addBox(checker, tesseract::common::LinkId("box_b"));
  checker.setActiveCollisionObjects({ tesseract::common::LinkId("box_a"), tesseract::common::LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);

  const std::vector<tesseract::common::LinkId> ids{ "box_a", "box_b" };
  Eigen::Isometry3d far_a{ Eigen::Isometry3d::Identity() };
  far_a.translation() = Eigen::Vector3d(-10, 0, 0);
  Eigen::Isometry3d far_b{ Eigen::Isometry3d::Identity() };
  far_b.translation() = Eigen::Vector3d(10, 0, 0);
  checker.setCollisionObjectsTransform(ids, tesseract::common::VectorIsometry3d{ far_a, far_b });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty());

  Eigen::Isometry3d near_a{ Eigen::Isometry3d::Identity() };
  near_a.translation() = Eigen::Vector3d(-0.25, 0, 0);
  Eigen::Isometry3d near_b{ Eigen::Isometry3d::Identity() };
  near_b.translation() = Eigen::Vector3d(0.25, 0, 0);
  checker.setCollisionObjectsTransform(ids, tesseract::common::VectorIsometry3d{ near_a, near_b });

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());

  EXPECT_TRUE(checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_a")).isApprox(near_a, 1e-6));
  EXPECT_TRUE(checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_b")).isApprox(near_b, 1e-6));
}
}  // namespace

TEST(TesseractCollisionBatchedTransformUnit, FCLDiscreteArraySetter)  // NOLINT
{
  runOnePoseArraySetterTest<FCLDiscreteBVHManager>();
}

TEST(TesseractCollisionBatchedTransformUnit, BulletDiscreteBVHArraySetter)  // NOLINT
{
  runOnePoseArraySetterTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBatchedTransformUnit, BulletDiscreteSimpleArraySetter)  // NOLINT
{
  runOnePoseArraySetterTest<BulletDiscreteSimpleManager>();
}

// The continuous managers inherit the one-pose array setter from the base class, which loops the single-object
// setter; a zero-length sweep must report the same overlap a discrete check would.
TEST(TesseractCollisionBatchedTransformUnit, BulletCastBVHArraySetter)  // NOLINT
{
  runOnePoseArraySetterTest<BulletCastBVHManager>();
}

// The two-pose array setter also comes from the continuous base class. A sweep set through it must produce the
// same contact as setting the sweep one object at a time.
TEST(TesseractCollisionBatchedTransformUnit, BulletCastBVHTwoPoseArraySetter)  // NOLINT
{
  BulletCastBVHManager checker;
  addBox(checker, tesseract::common::LinkId("box_a"));
  addBox(checker, tesseract::common::LinkId("box_b"));
  checker.setActiveCollisionObjects({ tesseract::common::LinkId("box_a") });
  checker.setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d start{ Eigen::Isometry3d::Identity() };
  start.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d end{ Eigen::Isometry3d::Identity() };
  end.translation() = Eigen::Vector3d(5, 0, 0);

  const std::vector<tesseract::common::LinkId> ids{ "box_a" };
  checker.setCollisionObjectsTransform(
      ids, tesseract::common::VectorIsometry3d{ start }, tesseract::common::VectorIsometry3d{ end });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());  // the swept box passes through box_b at the origin
}

TEST(TesseractCollisionBatchedTransformUnit, FCLArraySetterSkipsUnknownIds)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  addBox(checker, tesseract::common::LinkId("box_a"));

  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  const std::vector<tesseract::common::LinkId> ids{ "box_a", "does_not_exist" };

  EXPECT_NO_THROW(checker.setCollisionObjectsTransform(  // NOLINT
      ids,
      tesseract::common::VectorIsometry3d{ pose, Eigen::Isometry3d::Identity() }));
  EXPECT_TRUE(checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_a")).isApprox(pose, 1e-6));
}

TEST(TesseractCollisionBatchedTransformUnit, FCLArraySetterSizeMismatchThrows)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  const std::vector<tesseract::common::LinkId> ids{ "box_a", "box_b" };
  const tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  EXPECT_THROW(checker.setCollisionObjectsTransform(ids, poses), std::runtime_error);  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
