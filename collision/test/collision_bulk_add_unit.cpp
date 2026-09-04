#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
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

/** @brief A unit-box spec centred at the origin; the caller positions it with setCollisionObjectsTransform */
CollisionObjectSpec boxSpec(const LinkId& id, double edge = 1.0)
{
  CollisionObjectSpec spec;
  spec.id = id;
  spec.shapes = CollisionShapesConst{ std::make_shared<tesseract::geometry::Box>(edge, edge, edge) };
  spec.shape_poses = tesseract::common::VectorIsometry3d{ Eigen::Isometry3d::Identity() };
  return spec;
}

std::size_t countEntries(const std::vector<LinkId>& ids, const LinkId& id)
{
  return static_cast<std::size_t>(std::count(ids.begin(), ids.end(), id));
}

/**
 * @brief The contacts a result holds, as (pair signature, distance)
 *
 * Sorted, so map iteration order does not leak into a comparison, while the link ordering within each
 * contact — which the registration order decides — still does.
 */
std::vector<std::pair<std::string, double>> contactDigest(const ContactResultMap& result)
{
  ContactResultVector flat;
  result.flattenCopyResults(flat);

  std::vector<std::pair<std::string, double>> digest;
  digest.reserve(flat.size());
  for (const auto& contact : flat)
    digest.emplace_back(contact.link_ids[0].name() + "|" + contact.link_ids[1].name(), contact.distance);

  std::sort(digest.begin(), digest.end());
  return digest;
}

/**
 * @brief The distinct link pairs a result reports, each pair's names ordered so the comparison does not depend
 *        on which side of the pair a backend puts first
 */
std::vector<std::string> contactPairs(const ContactResultMap& result)
{
  ContactResultVector flat;
  result.flattenCopyResults(flat);

  std::vector<std::string> pairs;
  pairs.reserve(flat.size());
  for (const auto& contact : flat)
  {
    const std::string first = contact.link_ids[0].name();
    const std::string second = contact.link_ids[1].name();
    pairs.push_back(first < second ? first + "|" + second : second + "|" + first);
  }

  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
  return pairs;
}
}  // namespace

template <typename ManagerType>
void runBulkAddEquivalenceTest()
{
  const std::vector<CollisionObjectSpec> specs{ boxSpec(LinkId("box_a")),
                                                boxSpec(LinkId("box_b")),
                                                boxSpec(LinkId("box_c")) };

  ManagerType bulk;
  EXPECT_TRUE(bulk.addCollisionObjects(specs));

  ManagerType looped;
  for (const auto& spec : specs)
    EXPECT_TRUE(looped.addCollisionObject(spec.id, spec.mask_id, spec.shapes, spec.shape_poses, spec.enabled));

  EXPECT_EQ(bulk.getCollisionObjects(), looped.getCollisionObjects());  // same ids, same order

  for (auto* checker : { &bulk, &looped })
  {
    checker->setActiveCollisionObjects({ LinkId("box_a"), LinkId("box_b"), LinkId("box_c") });
    checker->setDefaultCollisionMargin(0.0);
    // box_a overlaps box_b and box_b overlaps box_c, but box_a does not reach box_c
    checker->setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b", "box_c" },
                                          tesseract::common::VectorIsometry3d{ at(-0.4), at(0.0), at(0.7) });
  }

  ContactResultMap bulk_result;
  bulk.contactTest(bulk_result, ContactRequest(ContactTestType::ALL));
  ContactResultMap looped_result;
  looped.contactTest(looped_result, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(bulk_result.count(), looped_result.count());
  EXPECT_FALSE(bulk_result.empty());

  // Same contacts, same link ordering within each, same distances - not merely the same number of them
  const std::vector<std::pair<std::string, double>> bulk_digest = contactDigest(bulk_result);
  const std::vector<std::pair<std::string, double>> looped_digest = contactDigest(looped_result);
  ASSERT_EQ(bulk_digest.size(), looped_digest.size());
  for (std::size_t i = 0; i < bulk_digest.size(); ++i)
  {
    EXPECT_EQ(bulk_digest[i].first, looped_digest[i].first);
    EXPECT_NEAR(bulk_digest[i].second, looped_digest[i].second, 1e-9);
  }
}

TEST(TesseractCollisionBulkAddUnit, BulletEquivalence)  // NOLINT
{
  runBulkAddEquivalenceTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkAddUnit, FCLEquivalence)  // NOLINT
{
  runBulkAddEquivalenceTest<FCLDiscreteBVHManager>();
}

template <typename ManagerType>
void runBulkAddDuplicateIdTest()
{
  ManagerType checker;
  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("box_a"), 1.0), boxSpec(LinkId("box_b"), 1.0) }));

  // Re-register box_a with a box small enough that it no longer reaches box_b.
  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("box_a"), 0.1) }));

  // Half one: no second entry for the id.
  EXPECT_EQ(countEntries(checker.getCollisionObjects(), LinkId("box_a")), 1U);

  // Half two: the broadphase sees the new geometry, not the old one still sitting in the tree.
  checker.setActiveCollisionObjects({ LinkId("box_a"), LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);
  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" },
                                       tesseract::common::VectorIsometry3d{ at(-0.4), at(0.4) });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty()) << "the displaced 1.0 box is still in the broadphase";
}

TEST(TesseractCollisionBulkAddUnit, BulletDuplicateIdReplaces)  // NOLINT
{
  runBulkAddDuplicateIdTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkAddUnit, FCLDuplicateIdReplaces)  // NOLINT
{
  runBulkAddDuplicateIdTest<FCLDiscreteBVHManager>();
}

// A repeated id inside one batch resolves the way the repeated single calls do: the earlier entry is displaced,
// so the last one decides the object and takes its position at the end of the list.
template <typename ManagerType>
void runBulkAddRepeatedIdWithinBatchTest()
{
  ManagerType checker;
  EXPECT_TRUE(checker.addCollisionObjects(
      { boxSpec(LinkId("box_a"), 1.0), boxSpec(LinkId("box_b"), 1.0), boxSpec(LinkId("box_a"), 0.1) }));

  EXPECT_EQ(countEntries(checker.getCollisionObjects(), LinkId("box_a")), 1U);
  EXPECT_EQ(checker.getCollisionObjects(), std::vector<LinkId>({ LinkId("box_b"), LinkId("box_a") }));

  // The surviving box_a is the 0.1 one, so it no longer reaches box_b.
  checker.setActiveCollisionObjects({ LinkId("box_a"), LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);
  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" },
                                       tesseract::common::VectorIsometry3d{ at(-0.4), at(0.4) });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty());
}

TEST(TesseractCollisionBulkAddUnit, BulletRepeatedIdWithinBatch)  // NOLINT
{
  runBulkAddRepeatedIdWithinBatchTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkAddUnit, FCLRepeatedIdWithinBatch)  // NOLINT
{
  runBulkAddRepeatedIdWithinBatchTest<FCLDiscreteBVHManager>();
}

// The single-object form removes any existing object for the id before it tries to build the new one, so a spec
// that fails to build leaves the id unregistered rather than leaving the old object in place.
template <typename ManagerType>
void runBulkAddFailedSpecForRegisteredIdTest()
{
  CollisionObjectSpec bad;
  bad.id = LinkId("box_a");  // empty shapes and shape_poses

  ManagerType already_registered;
  EXPECT_TRUE(already_registered.addCollisionObjects({ boxSpec(LinkId("box_a")), boxSpec(LinkId("box_b")) }));
  ASSERT_TRUE(already_registered.hasCollisionObject(LinkId("box_a")));

  EXPECT_FALSE(already_registered.addCollisionObjects({ bad }));
  EXPECT_FALSE(already_registered.hasCollisionObject(LinkId("box_a")));
  EXPECT_EQ(already_registered.getCollisionObjects(), std::vector<LinkId>({ LinkId("box_b") }));

  // Same within a single batch: the failing spec displaces the entry the earlier one made.
  ManagerType within_batch;
  EXPECT_FALSE(within_batch.addCollisionObjects({ boxSpec(LinkId("box_a")), bad }));
  EXPECT_FALSE(within_batch.hasCollisionObject(LinkId("box_a")));
}

TEST(TesseractCollisionBulkAddUnit, BulletFailedSpecForRegisteredId)  // NOLINT
{
  runBulkAddFailedSpecForRegisteredIdTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkAddUnit, FCLFailedSpecForRegisteredId)  // NOLINT
{
  runBulkAddFailedSpecForRegisteredIdTest<FCLDiscreteBVHManager>();
}

template <typename ManagerType>
void runBulkAddPartialFailureTest()
{
  CollisionObjectSpec bad;
  bad.id = LinkId("no_geometry");  // empty shapes and shape_poses

  ManagerType checker;
  EXPECT_FALSE(checker.addCollisionObjects({ boxSpec(LinkId("box_a")), bad, boxSpec(LinkId("box_b")) }));

  EXPECT_TRUE(checker.hasCollisionObject(LinkId("box_a")));
  EXPECT_TRUE(checker.hasCollisionObject(LinkId("box_b")));
  EXPECT_FALSE(checker.hasCollisionObject(LinkId("no_geometry")));
}

TEST(TesseractCollisionBulkAddUnit, BulletPartialFailure)  // NOLINT
{
  runBulkAddPartialFailureTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkAddUnit, FCLPartialFailure)  // NOLINT
{
  runBulkAddPartialFailureTest<FCLDiscreteBVHManager>();
}

template <typename ManagerType>
void runBulkAddBroadphaseLiveTest()
{
  ManagerType checker;
  // Position through the spec's own shape_poses, so nothing after the bulk add can refresh the broadphase.
  CollisionObjectSpec a = boxSpec(LinkId("box_a"));
  a.shape_poses = tesseract::common::VectorIsometry3d{ at(-0.25) };
  CollisionObjectSpec b = boxSpec(LinkId("box_b"));
  b.shape_poses = tesseract::common::VectorIsometry3d{ at(0.25) };

  checker.setActiveCollisionObjects({ LinkId("box_a"), LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);
  EXPECT_TRUE(checker.addCollisionObjects({ a, b }));

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty()) << "bulk add left the broadphase unrefreshed";
}

TEST(TesseractCollisionBulkAddUnit, BulletBroadphaseLiveAfterBulkAdd)  // NOLINT
{
  runBulkAddBroadphaseLiveTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkAddUnit, FCLBroadphaseLiveAfterBulkAdd)  // NOLINT
{
  runBulkAddBroadphaseLiveTest<FCLDiscreteBVHManager>();
}

// Adding a batch to a manager that already holds objects and already has a non-empty active set, which is the
// shape a scene-graph insertion takes.
template <typename ManagerType>
void runBulkAddIntoPopulatedManagerTest()
{
  ManagerType checker;
  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("existing_a")), boxSpec(LinkId("existing_b")) }));

  // "added" is named active before it exists, so the batch below lands while active_ is already populated
  checker.setActiveCollisionObjects({ LinkId("existing_a"), LinkId("added") });
  checker.setDefaultCollisionMargin(0.0);
  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "existing_a", "existing_b" },
                                       tesseract::common::VectorIsometry3d{ at(-0.25), at(20) });

  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("added")) }));

  // The batch appends without disturbing what was there
  EXPECT_EQ(checker.getCollisionObjects(),
            std::vector<LinkId>({ LinkId("existing_a"), LinkId("existing_b"), LinkId("added") }));

  // The new object went into the active tree and collides with the active object already registered, while the
  // out-of-reach static one is not reported
  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "added" }, tesseract::common::VectorIsometry3d{ at(0.25) });
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(contactPairs(result), std::vector<std::string>({ "added|existing_a" }));

  // The pre-existing static object still answers once it is moved into reach, and only against the active ones
  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "existing_b" },
                                       tesseract::common::VectorIsometry3d{ at(0.9) });
  ContactResultMap with_static;
  checker.contactTest(with_static, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(contactPairs(with_static), std::vector<std::string>({ "added|existing_a", "added|existing_b" }));
}

TEST(TesseractCollisionBulkAddUnit, BulletBulkAddIntoPopulatedManager)  // NOLINT
{
  runBulkAddIntoPopulatedManagerTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkAddUnit, FCLBulkAddIntoPopulatedManager)  // NOLINT
{
  runBulkAddIntoPopulatedManagerTest<FCLDiscreteBVHManager>();
}

template <typename ManagerType>
void runBulkAddStaticDynamicSplitTest()
{
  ManagerType checker;
  checker.setActiveCollisionObjects({ LinkId("moving") });  // "fixed" is static
  checker.setDefaultCollisionMargin(0.0);
  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("moving")), boxSpec(LinkId("fixed")) }));

  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "moving", "fixed" },
                                       tesseract::common::VectorIsometry3d{ at(-0.25), at(0.25) });

  // An active-vs-static pair collides; a static-vs-static pair does not.
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());

  // An empty active set means every link is active, so demote both links by activating an unrelated one.
  checker.setActiveCollisionObjects({ LinkId("unrelated") });
  ContactResultMap none_active;
  checker.contactTest(none_active, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(none_active.empty());
}

TEST(TesseractCollisionBulkAddUnit, BulletStaticDynamicSplit)  // NOLINT
{
  runBulkAddStaticDynamicSplitTest<BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionBulkAddUnit, FCLStaticDynamicSplit)  // NOLINT
{
  runBulkAddStaticDynamicSplitTest<FCLDiscreteBVHManager>();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
