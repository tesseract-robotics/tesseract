#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/geometry/impl/box.h>

using namespace tesseract::collision;
using tesseract::common::LinkId;

namespace
{
Eigen::Isometry3d at(double x)
{
  Eigen::Isometry3d tf{ Eigen::Isometry3d::Identity() };
  tf.translation() = Eigen::Vector3d(x, 0, 0);
  return tf;
}

/** @brief Register one unit box at @p pose through the single-object entry point */
template <typename ManagerType>
void addBox(ManagerType& checker, const LinkId& id, const Eigen::Isometry3d& pose, double edge = 1.0)
{
  CollisionShapesConst shapes{ std::make_shared<tesseract::geometry::Box>(edge, edge, edge) };
  tesseract::common::VectorIsometry3d poses{ pose };
  checker.addCollisionObject(id, 0, shapes, poses, true);
}
}  // namespace

// Disabling through the batch form must be indistinguishable from disabling one at a time.
template <typename ManagerType>
void runBulkEnabledEquivalenceTest()
{
  ManagerType checker;
  addBox(checker, LinkId("a"), at(-0.25));
  addBox(checker, LinkId("b"), at(0.25));
  // One active, one static: that is the pair a cast broadphase is built to report. Making both links active
  // removes the only pair under test.
  checker.setActiveCollisionObjects({ LinkId("a") });
  checker.setDefaultCollisionMargin(0.0);

  ContactResultMap before;
  checker.contactTest(before, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(before.empty());

  checker.setCollisionObjectsEnabled({ { LinkId("b"), false } });
  EXPECT_FALSE(checker.isCollisionObjectEnabled(LinkId("b")));
  EXPECT_TRUE(checker.isCollisionObjectEnabled(LinkId("a")));

  ContactResultMap disabled;
  checker.contactTest(disabled, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(disabled.empty()) << "disabled object still reported a contact";

  // And back again, in the same call shape, mixing both directions in one batch.
  checker.setCollisionObjectsEnabled({ { LinkId("b"), true }, { LinkId("a"), true } });
  ContactResultMap reenabled;
  checker.contactTest(reenabled, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(reenabled.count(), before.count());
}

// Disabling the active link, not just the static one, must silence its contacts. On a cast manager the active
// link's broadphase presence is its cast wrapper, a different object from the one the static link registers, so
// this exercises a write the equivalence test above never reaches.
template <typename ManagerType>
void runBulkEnabledActiveLinkTest()
{
  ManagerType checker;
  addBox(checker, LinkId("a"), at(-0.25));
  addBox(checker, LinkId("b"), at(0.25));
  checker.setActiveCollisionObjects({ LinkId("a") });
  checker.setDefaultCollisionMargin(0.0);

  ContactResultMap before;
  checker.contactTest(before, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(before.empty());

  checker.setCollisionObjectsEnabled({ { LinkId("a"), false } });
  EXPECT_FALSE(checker.isCollisionObjectEnabled(LinkId("a")));

  ContactResultMap disabled;
  checker.contactTest(disabled, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(disabled.empty()) << "disabled active object still reported a contact";
}

// An id the manager does not hold is skipped, not an error, matching the single-object contract. The absence is
// still reported, and the ids that were present still applied.
template <typename ManagerType>
void runBulkEnabledUnknownIdTest()
{
  ManagerType checker;
  addBox(checker, LinkId("a"), at(0));
  EXPECT_FALSE(checker.setCollisionObjectsEnabled(  // NOLINT
      { { LinkId("a"), false }, { LinkId("not_registered"), false } }));
  EXPECT_FALSE(checker.isCollisionObjectEnabled(LinkId("a")));
  EXPECT_TRUE(checker.setCollisionObjectsEnabled({ { LinkId("a"), true } }));
  EXPECT_TRUE(checker.isCollisionObjectEnabled(LinkId("a")));
}

// The range form builds the map form's argument, so it must show the same behaviour and the same return.
template <typename ManagerType>
void runBulkEnabledRangeFormTest()
{
  ManagerType checker;
  addBox(checker, LinkId("a"), at(-0.25));
  addBox(checker, LinkId("b"), at(0.25));
  checker.setActiveCollisionObjects({ LinkId("a") });
  checker.setDefaultCollisionMargin(0.0);

  ContactResultMap before;
  checker.contactTest(before, ContactRequest(ContactTestType::ALL));
  ASSERT_FALSE(before.empty());

  EXPECT_TRUE(checker.setCollisionObjectsEnabled(std::vector<LinkId>{ LinkId("a"), LinkId("b") }, false));
  EXPECT_FALSE(checker.isCollisionObjectEnabled(LinkId("a")));
  EXPECT_FALSE(checker.isCollisionObjectEnabled(LinkId("b")));

  ContactResultMap disabled;
  checker.contactTest(disabled, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(disabled.empty()) << "disabled objects still reported a contact";

  EXPECT_TRUE(checker.setCollisionObjectsEnabled(std::vector<LinkId>{ LinkId("a"), LinkId("b") }, true));
  ContactResultMap reenabled;
  checker.contactTest(reenabled, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(reenabled.count(), before.count());

  EXPECT_FALSE(checker.setCollisionObjectsEnabled(std::vector<LinkId>{ LinkId("not_registered") }, false));
  EXPECT_TRUE(checker.setCollisionObjectsEnabled(std::vector<LinkId>{}, false));
}

TEST(TesseractCollisionBulkEnabledUnit, BulletDiscreteEquivalence)  // NOLINT
{
  runBulkEnabledEquivalenceTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, BulletCastEquivalence)  // NOLINT
{
  runBulkEnabledEquivalenceTest<BulletCastBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, FCLEquivalence)  // NOLINT
{
  runBulkEnabledEquivalenceTest<FCLDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, BulletDiscreteActiveLink)  // NOLINT
{
  runBulkEnabledActiveLinkTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, BulletCastActiveLink)  // NOLINT
{
  runBulkEnabledActiveLinkTest<BulletCastBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, FCLActiveLink)  // NOLINT
{
  runBulkEnabledActiveLinkTest<FCLDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, BulletDiscreteUnknownId)  // NOLINT
{
  runBulkEnabledUnknownIdTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, BulletCastUnknownId)  // NOLINT
{
  runBulkEnabledUnknownIdTest<BulletCastBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, FCLUnknownId)  // NOLINT
{
  runBulkEnabledUnknownIdTest<FCLDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, BulletDiscreteRangeForm)  // NOLINT
{
  runBulkEnabledRangeFormTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, BulletCastRangeForm)  // NOLINT
{
  runBulkEnabledRangeFormTest<BulletCastBVHManager>();
}

TEST(TesseractCollisionBulkEnabledUnit, FCLRangeForm)  // NOLINT
{
  runBulkEnabledRangeFormTest<FCLDiscreteBVHManager>();
}

// The range overload is a non-virtual base member, and a class that overrides the map overload hides every base
// overload of that name. These two managers override it, so without a using-declaration the calls below do not
// compile through the derived static type. Bind a derived reference so no base-class conversion can hide that.
TEST(TesseractCollisionBulkEnabledUnit, RangeFormReachableThroughDerivedReference)  // NOLINT
{
  BulletCastBVHManager cast_checker;
  addBox(cast_checker, LinkId("a"), at(-0.25));
  addBox(cast_checker, LinkId("b"), at(0.25));
  cast_checker.setActiveCollisionObjects({ LinkId("a") });
  cast_checker.setDefaultCollisionMargin(0.0);

  BulletCastBVHManager& cast_ref = cast_checker;
  EXPECT_TRUE(cast_ref.setCollisionObjectsEnabled(std::vector<LinkId>{ LinkId("a") }, false));
  EXPECT_FALSE(cast_ref.isCollisionObjectEnabled(LinkId("a")));

  ContactResultMap cast_result;
  cast_ref.contactTest(cast_result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(cast_result.empty());

  BulletDiscreteBVHManager discrete_checker;
  addBox(discrete_checker, LinkId("a"), at(-0.25));
  addBox(discrete_checker, LinkId("b"), at(0.25));
  discrete_checker.setActiveCollisionObjects({ LinkId("a"), LinkId("b") });
  discrete_checker.setDefaultCollisionMargin(0.0);

  BulletDiscreteBVHManager& discrete_ref = discrete_checker;
  EXPECT_TRUE(discrete_ref.setCollisionObjectsEnabled(std::vector<LinkId>{ LinkId("a") }, false));
  EXPECT_FALSE(discrete_ref.isCollisionObjectEnabled(LinkId("a")));

  ContactResultMap discrete_result;
  discrete_ref.contactTest(discrete_result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(discrete_result.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
