#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/geometry/impl/box.h>

using namespace tesseract::collision;
using tesseract::common::LinkId;
using tesseract::common::LinkIdTransformMap;

namespace
{
template <typename ManagerType>
void addBox(ManagerType& checker, const LinkId& id)
{
  CollisionShapePtr box = std::make_shared<tesseract::geometry::Box>(1, 1, 1);
  CollisionShapesConst shapes{ box };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject(id, 0, shapes, poses);
}

Eigen::Isometry3d at(double x)
{
  Eigen::Isometry3d tf{ Eigen::Isometry3d::Identity() };
  tf.translation() = Eigen::Vector3d(x, 0, 0);
  return tf;
}

/** @brief Two boxes, both active, margin 0, both parked far apart */
template <typename ManagerType>
void setup(ManagerType& checker)
{
  addBox(checker, LinkId("box_a"));
  addBox(checker, LinkId("box_b"));
  checker.setActiveCollisionObjects({ LinkId("box_a"), LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);
  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" },
                                       tesseract::common::VectorIsometry3d{ at(-10), at(10) });
}
}  // namespace

template <typename ManagerType, typename IdRange>
void runEquivalenceTest()
{
  const LinkIdTransformMap state{ { LinkId("box_a"), at(-0.25) }, { LinkId("box_b"), at(0.25) } };

  ManagerType through_map;
  setup(through_map);
  through_map.setCollisionObjectsTransform(IdRange{ LinkId("box_a"), LinkId("box_b") }, state);

  ManagerType through_array;
  setup(through_array);
  through_array.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" },
                                             tesseract::common::VectorIsometry3d{ at(-0.25), at(0.25) });

  for (const auto& id : { LinkId("box_a"), LinkId("box_b") })
    EXPECT_TRUE(
        through_map.getCollisionObjectsTransform(id).isApprox(through_array.getCollisionObjectsTransform(id), 1e-9));

  ContactResultMap map_result;
  through_map.contactTest(map_result, ContactRequest(ContactTestType::ALL));
  ContactResultMap array_result;
  through_array.contactTest(array_result, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(map_result.count(), array_result.count());
  EXPECT_FALSE(map_result.empty());  // guard against both paths being trivially no-ops
}

TEST(TesseractCollisionRangeTransformUnit, BulletEquivalenceSet)  // NOLINT
{
  runEquivalenceTest<BulletDiscreteBVHManager, std::unordered_set<LinkId>>();
}

TEST(TesseractCollisionRangeTransformUnit, BulletEquivalenceVector)  // NOLINT
{
  runEquivalenceTest<BulletDiscreteBVHManager, std::vector<LinkId>>();
}

TEST(TesseractCollisionRangeTransformUnit, FCLEquivalenceSet)  // NOLINT
{
  runEquivalenceTest<FCLDiscreteBVHManager, std::unordered_set<LinkId>>();
}

TEST(TesseractCollisionRangeTransformUnit, FCLEquivalenceVector)  // NOLINT
{
  runEquivalenceTest<FCLDiscreteBVHManager, std::vector<LinkId>>();
}

TEST(TesseractCollisionRangeTransformUnit, StateMayCarryLinksOutsideTheIdRange)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  setup(checker);

  // The state names both boxes; the id range names only one.
  const LinkIdTransformMap state{ { LinkId("box_a"), at(-0.25) }, { LinkId("box_b"), at(0.25) } };
  checker.setCollisionObjectsTransform(std::unordered_set<LinkId>{ LinkId("box_a") }, state);

  EXPECT_TRUE(checker.getCollisionObjectsTransform(LinkId("box_a")).isApprox(at(-0.25), 1e-9));
  EXPECT_TRUE(checker.getCollisionObjectsTransform(LinkId("box_b")).isApprox(at(10), 1e-9));

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty());  // box_b never moved, so nothing is in contact
}

TEST(TesseractCollisionRangeTransformUnit, MissingIdThrowsNamingTheLink)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  setup(checker);
  const LinkIdTransformMap state{ { LinkId("box_a"), at(-0.25) } };  // box_b absent

  try
  {
    checker.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" }, state);
    FAIL() << "expected std::out_of_range";
  }
  catch (const std::out_of_range& e)
  {
    EXPECT_NE(std::string(e.what()).find("box_b"), std::string::npos) << e.what();
  }

  // The throw must not leave a half-applied update behind: box_a is gathered before box_b is looked up, and the
  // forward to the array setter has not happened yet, so nothing moved.
  EXPECT_TRUE(checker.getCollisionObjectsTransform(LinkId("box_a")).isApprox(at(-10), 1e-9));
}

TEST(TesseractCollisionRangeTransformUnit, CastSameMapTwiceMatchesOnePose)  // NOLINT
{
  const LinkIdTransformMap state{ { LinkId("box_a"), at(-0.25) }, { LinkId("box_b"), at(0.25) } };

  BulletCastBVHManager two_map;
  setup(two_map);
  two_map.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" }, state, state);

  BulletCastBVHManager one_pose;
  setup(one_pose);
  one_pose.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" }, state);

  ContactResultMap two_map_result;
  two_map.contactTest(two_map_result, ContactRequest(ContactTestType::ALL));
  ContactResultMap one_pose_result;
  one_pose.contactTest(one_pose_result, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(two_map_result.count(), one_pose_result.count());
  EXPECT_FALSE(two_map_result.empty());
}

TEST(TesseractCollisionRangeTransformUnit, CastTwoDifferentMapsSweep)  // NOLINT
{
  BulletCastBVHManager checker;
  addBox(checker, LinkId("box_a"));
  addBox(checker, LinkId("box_b"));
  checker.setActiveCollisionObjects({ LinkId("box_a") });  // box_b static, sitting at the origin
  checker.setDefaultCollisionMargin(0.0);

  const LinkIdTransformMap state0{ { LinkId("box_a"), at(-5) } };
  const LinkIdTransformMap state1{ { LinkId("box_a"), at(5) } };
  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a" }, state0, state1);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());  // the swept box crosses box_b at the origin
}

TEST(TesseractCollisionRangeTransformUnit, CastMissingIdInSecondMapThrowsNamingTheLink)  // NOLINT
{
  BulletCastBVHManager checker;
  setup(checker);
  const LinkIdTransformMap state0{ { LinkId("box_a"), at(-5) } };
  const LinkIdTransformMap state1;  // empty — box_a present in state0 only

  try
  {
    checker.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a" }, state0, state1);
    FAIL() << "expected std::out_of_range";
  }
  catch (const std::out_of_range& e)
  {
    EXPECT_NE(std::string(e.what()).find("box_a"), std::string::npos) << e.what();
  }
}

TEST(TesseractCollisionRangeTransformUnit, BothBaseClassesThrowTheSameMessage)  // NOLINT
{
  // The discrete and continuous base classes each carry their own copy of the id-to-pose gather. A caller must get
  // the same diagnostic whichever one it went through, so the two messages are compared against each other rather
  // than matched against a pattern: a reworded message in one copy alone fails here.
  const LinkIdTransformMap state{ { LinkId("box_a"), at(-0.25) } };  // box_b absent

  std::string discrete_what;
  {
    BulletDiscreteBVHManager checker;
    setup(checker);
    try
    {
      checker.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" }, state);
      FAIL() << "expected std::out_of_range";
    }
    catch (const std::out_of_range& e)
    {
      discrete_what = e.what();
    }
  }

  std::string continuous_what;
  {
    BulletCastBVHManager checker;
    setup(checker);
    try
    {
      checker.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" }, state);
      FAIL() << "expected std::out_of_range";
    }
    catch (const std::out_of_range& e)
    {
      continuous_what = e.what();
    }
  }

  EXPECT_FALSE(discrete_what.empty());
  EXPECT_EQ(discrete_what, continuous_what);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
