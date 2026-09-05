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

namespace
{
Eigen::Isometry3d translated(double x)
{
  Eigen::Isometry3d tf{ Eigen::Isometry3d::Identity() };
  tf.translation() = Eigen::Vector3d(x, 0, 0);
  return tf;
}
}  // namespace

// Re-setting an identical transform must be a no-op that is invisible from outside: same stored pose, same contacts.
TEST(TesseractCollisionBatchedTransformUnit, BulletDiscreteRedundantSetIsInvisible)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  addBox(checker, tesseract::common::LinkId("box_a"));
  addBox(checker, tesseract::common::LinkId("box_b"));
  checker.setActiveCollisionObjects({ tesseract::common::LinkId("box_a"), tesseract::common::LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);

  checker.setCollisionObjectsTransform(tesseract::common::LinkId("box_a"), translated(-0.25));
  checker.setCollisionObjectsTransform(tesseract::common::LinkId("box_b"), translated(0.25));

  ContactResultMap before;
  checker.contactTest(before, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(before.empty());

  // The guarded path: same value again, three times.
  for (int i = 0; i < 3; ++i)
    checker.setCollisionObjectsTransform(tesseract::common::LinkId("box_a"), translated(-0.25));

  ContactResultMap after;
  checker.contactTest(after, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(before.count(), after.count());
  EXPECT_TRUE(
      checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_a")).isApprox(translated(-0.25), 1e-9));

  // A genuinely new transform must still take effect, through the broadphase and not just the stored pose.
  checker.setCollisionObjectsTransform(tesseract::common::LinkId("box_a"), translated(-10));
  ContactResultMap moved;
  checker.contactTest(moved, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(moved.empty());
}

// The map and array setters delegate to the guarded single-object setter, so both must show the same behaviour.
// Each call below has to move the boxes for the following assertion to hold, so a setter that never reaches the
// guarded single-object form fails the test.
TEST(TesseractCollisionBatchedTransformUnit, BulletDiscreteGuardReachedThroughMapAndArray)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  addBox(checker, tesseract::common::LinkId("box_a"));
  addBox(checker, tesseract::common::LinkId("box_b"));
  checker.setActiveCollisionObjects({ tesseract::common::LinkId("box_a"), tesseract::common::LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);

  // Park the boxes apart, so the boxes only touch again if a later setter actually moves them.
  const tesseract::common::LinkIdTransformMap apart{ { tesseract::common::LinkId("box_a"), translated(-10) },
                                                     { tesseract::common::LinkId("box_b"), translated(10) } };
  checker.setCollisionObjectsTransform(apart);

  ContactResultMap parked;
  checker.contactTest(parked, ContactRequest(ContactTestType::ALL));
  ASSERT_TRUE(parked.empty());

  const tesseract::common::LinkIdTransformMap in_contact{ { tesseract::common::LinkId("box_a"), translated(-0.25) },
                                                          { tesseract::common::LinkId("box_b"), translated(0.25) } };

  checker.setCollisionObjectsTransform(in_contact);
  ContactResultMap through_map;
  checker.contactTest(through_map, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(through_map.empty());
  EXPECT_TRUE(
      checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_a")).isApprox(translated(-0.25), 1e-9));

  // The redundant map hits the unchanged-transform guard, which must leave both the pose and the contacts alone.
  checker.setCollisionObjectsTransform(in_contact);
  ContactResultMap redundant;
  checker.contactTest(redundant, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(through_map.count(), redundant.count());

  // The array overload has to carry a genuine move all the way to the broadphase.
  checker.setCollisionObjectsTransform(std::vector<tesseract::common::LinkId>{ "box_a", "box_b" },
                                       tesseract::common::VectorIsometry3d{ translated(-10), translated(10) });
  ContactResultMap through_array;
  checker.contactTest(through_array, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(through_array.empty());
  EXPECT_TRUE(checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_a")).isApprox(translated(-10), 1e-9));
}

// The two-pose setter is deliberately unguarded: an update whose pose1 is unchanged and whose pose2 is not must
// still rebuild the swept hull. Guarding it on pose1 makes this fail, because the sweep never happens and nothing
// crosses box_b.
TEST(TesseractCollisionBatchedTransformUnit, BulletCastUnchangedPose1StillSweeps)  // NOLINT
{
  BulletCastBVHManager checker;
  addBox(checker, tesseract::common::LinkId("box_a"));
  addBox(checker, tesseract::common::LinkId("box_b"));
  checker.setActiveCollisionObjects({ tesseract::common::LinkId("box_a") });
  checker.setDefaultCollisionMargin(0.0);

  // First sweep: a short move that stays clear of box_b at the origin.
  checker.setCollisionObjectsTransform(tesseract::common::LinkId("box_a"), translated(-5), translated(-4));
  ContactResultMap clear_result;
  checker.contactTest(clear_result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(clear_result.empty());

  // Second sweep: same pose1, a pose2 that crosses the origin.
  checker.setCollisionObjectsTransform(tesseract::common::LinkId("box_a"), translated(-5), translated(5));
  ContactResultMap swept_result;
  checker.contactTest(swept_result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(swept_result.empty());
}

// The cast manager's one-pose setter is guarded, and for a kinematic link the broadphase proxy lives on the cast
// cow rather than the regular one. A redundant one-pose set on a static link must remain invisible.
TEST(TesseractCollisionBatchedTransformUnit, BulletCastRedundantOnePoseSetIsInvisible)  // NOLINT
{
  BulletCastBVHManager checker;
  addBox(checker, tesseract::common::LinkId("box_a"));
  addBox(checker, tesseract::common::LinkId("box_b"));
  checker.setActiveCollisionObjects({ tesseract::common::LinkId("box_a") });  // box_b is static
  checker.setDefaultCollisionMargin(0.0);

  checker.setCollisionObjectsTransform(tesseract::common::LinkId("box_b"), translated(0.25));
  checker.setCollisionObjectsTransform(tesseract::common::LinkId("box_b"), translated(0.25));  // guarded
  checker.setCollisionObjectsTransform(tesseract::common::LinkId("box_a"), translated(-0.25), translated(-0.25));

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());
  EXPECT_TRUE(
      checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_b")).isApprox(translated(0.25), 1e-9));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
