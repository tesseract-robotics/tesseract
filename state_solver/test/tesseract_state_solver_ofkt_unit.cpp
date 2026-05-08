#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <unordered_set>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/state_solver/ofkt/ofkt_nodes.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>
#include "state_solver_test_suite.h"

using namespace tesseract::scene_graph;

// Most of OFKT is tested in the tesseract_environment_unit.cpp
TEST(TesseractStateSolverUnit, OFKTNodeBaseAndFailuresUnit)  // NOLINT
{
  {  // OFKTRootNode
    OFKTRootNode node("base_link");
    EXPECT_ANY_THROW(node.setParent(nullptr));                                      // NOLINT
    EXPECT_ANY_THROW(node.storeJointValue(0));                                      // NOLINT
    EXPECT_ANY_THROW(node.setStaticTransformation(Eigen::Isometry3d::Identity()));  // NOLINT
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
    node.computeAndStoreLocalTransformation();
    node.computeAndStoreWorldTransformation();
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.getLocalTransformation(), 1e-6));
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.getWorldTransformation(), 1e-6));
  }

  {  // OFKTRootNode
    OFKTRootNode node("base_link");
    EXPECT_ANY_THROW(node.setParent(nullptr));                                      // NOLINT
    EXPECT_ANY_THROW(node.storeJointValue(0));                                      // NOLINT
    EXPECT_ANY_THROW(node.setStaticTransformation(Eigen::Isometry3d::Identity()));  // NOLINT
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
  }

  {  // OFKTFixedNode
    OFKTRootNode root_node("base_link");
    OFKTFixedNode node(&root_node, "base_link", "joint_a1", Eigen::Isometry3d::Identity());
    const OFKTFixedNode& const_node = node;
    EXPECT_TRUE(const_node.getParent() == &root_node);
    EXPECT_ANY_THROW(node.storeJointValue(M_PI_2));  // NOLINT
    EXPECT_ANY_THROW(node.getJointValue());          // NOLINT
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
    EXPECT_TRUE(node.getStaticTransformation().isApprox(Eigen::Isometry3d::Identity(), 1e-6));
    node.computeAndStoreLocalTransformation();
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.getLocalTransformation(), 1e-6));

    Eigen::Isometry3d static_tf = Eigen::Isometry3d::Identity();
    static_tf.translation() = Eigen::Vector3d(1, 2, 3);
    node.setStaticTransformation(static_tf);
    EXPECT_TRUE(node.getStaticTransformation().isApprox(static_tf, 1e-6));
  }

  {  // OFKTFloatingNode
    OFKTRootNode root_node("base_link");
    OFKTFloatingNode node(&root_node, "base_link", "joint_a1", Eigen::Isometry3d::Identity());
    const OFKTFloatingNode& const_node = node;
    EXPECT_TRUE(const_node.getParent() == &root_node);
    EXPECT_ANY_THROW(node.storeJointValue(M_PI_2));  // NOLINT
    EXPECT_ANY_THROW(node.getJointValue());          // NOLINT
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
    EXPECT_TRUE(node.getStaticTransformation().isApprox(Eigen::Isometry3d::Identity(), 1e-6));
    node.computeAndStoreLocalTransformation();
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.getLocalTransformation(), 1e-6));

    Eigen::Isometry3d static_tf = Eigen::Isometry3d::Identity();
    static_tf.translation() = Eigen::Vector3d(1, 2, 3);
    node.setStaticTransformation(static_tf);
    EXPECT_TRUE(node.getStaticTransformation().isApprox(static_tf, 1e-6));
  }

  {  // OFKTRevoluteNode
    auto check = Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1));
    OFKTRootNode root_node("base_link");
    OFKTRevoluteNode node(&root_node, "base_link", "joint_a1", Eigen::Isometry3d::Identity(), Eigen::Vector3d(0, 0, 1));
    EXPECT_TRUE(node.getParent() == &root_node);
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(node.getAxis().isApprox(Eigen::Vector3d(0, 0, 1), 1e-6));
    EXPECT_NO_THROW(node.storeJointValue(M_PI_2));  // NOLINT
    EXPECT_NEAR(node.getJointValue(), M_PI_2, 1e-6);
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(check.isApprox(node.computeLocalTransformation(M_PI_2), 1e-6));
    node.computeAndStoreLocalTransformation();
    EXPECT_TRUE(node.getLocalTransformation().isApprox(check, 1e-6));
    node.computeAndStoreWorldTransformation();
    EXPECT_TRUE(check.isApprox(node.getWorldTransformation(), 1e-6));
  }

  {  // OFKTContinuousNode
    auto check = Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1));
    OFKTRootNode root_node("base_link");
    OFKTContinuousNode node(
        &root_node, "base_link", "joint_a1", Eigen::Isometry3d::Identity(), Eigen::Vector3d(0, 0, 1));
    const OFKTContinuousNode& const_node = node;
    EXPECT_TRUE(const_node.getParent() == &root_node);
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(node.getAxis().isApprox(Eigen::Vector3d(0, 0, 1), 1e-6));
    EXPECT_NO_THROW(node.storeJointValue(M_PI_2));  // NOLINT
    EXPECT_NEAR(node.getJointValue(), M_PI_2, 1e-6);
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(check.isApprox(node.computeLocalTransformation(M_PI_2), 1e-6));
    node.computeAndStoreLocalTransformation();
    EXPECT_TRUE(node.getLocalTransformation().isApprox(check, 1e-6));
    node.computeAndStoreWorldTransformation();
    EXPECT_TRUE(check.isApprox(node.getWorldTransformation(), 1e-6));
  }

  {  // OFKTPrismaticNode
    auto check = Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.45, 0, 0);
    OFKTRootNode root_node("base_link");
    OFKTPrismaticNode node(
        &root_node, "base_link", "joint_a1", Eigen::Isometry3d::Identity(), Eigen::Vector3d(1, 0, 0));
    EXPECT_TRUE(node.getParent() == &root_node);
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(node.getAxis().isApprox(Eigen::Vector3d(1, 0, 0), 1e-6));
    EXPECT_NO_THROW(node.storeJointValue(1.45));  // NOLINT
    EXPECT_NEAR(node.getJointValue(), 1.45, 1e-6);
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(check.isApprox(node.computeLocalTransformation(1.45), 1e-6));
    node.computeAndStoreLocalTransformation();
    EXPECT_TRUE(node.getLocalTransformation().isApprox(check, 1e-6));
    node.computeAndStoreWorldTransformation();
    EXPECT_TRUE(check.isApprox(node.getWorldTransformation(), 1e-6));
  }
}

TEST(TesseractStateSolverUnit, OFKTAddRemoveLinkUnit)  // NOLINT
{
  test_suite::runAddandRemoveLinkTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTAddSceneGraphUnit)  // NOLINT
{
  test_suite::runAddSceneGraphTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTChangeJointOriginUnit)  // NOLINT
{
  test_suite::runChangeJointOriginTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTMoveJointUnit)  // NOLINT
{
  test_suite::runMoveJointTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTMoveLinkUnit)  // NOLINT
{
  test_suite::runMoveLinkTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTReplaceJointUnit)  // NOLINT
{
  test_suite::runReplaceJointTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTChangeJointLimitsUnit)  // NOLINT
{
  test_suite::runChangeJointLimitsTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, KDLGetJacobianUnit)  // NOLINT
{
  test_suite::runJacobianTest<KDLStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTGetJacobianUnit)  // NOLINT
{
  test_suite::runJacobianTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTSetFloatingJointStateUnit)  // NOLINT
{
  test_suite::runSetFloatingJointStateTest<OFKTStateSolver>();
}

// All OFKT entrypoints that overlay user-supplied floating-joint values used to call bare .at(),
// throwing std::out_of_range with no context. Verify the safety-sweep replacement throws a
// std::runtime_error whose message names the offending joint id, so callers can identify the
// bad entry instead of debugging a context-free out_of_range.
TEST(TesseractStateSolverUnit, OFKTApplyFloatingValuesUnknownIdThrows)  // NOLINT
{
  using tesseract::common::JointId;
  using tesseract::common::JointIdTransformMap;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::scene_graph::test_suite::getSceneGraph(locator);
  OFKTStateSolver solver(*scene_graph);

  JointIdTransformMap bad;
  bad[JointId("does_not_exist_floating")] = Eigen::Isometry3d::Identity();

  // The setState(JointIdTransformMap) overload is the simplest user-facing entrypoint into the
  // floating-joint overlay path. It must throw std::runtime_error (not std::out_of_range).
  EXPECT_THROW(solver.setState(bad), std::runtime_error);  // NOLINT

  // The throw message must name the offending joint id so callers can identify the bad entry.
  try
  {
    solver.setState(bad);
    FAIL() << "expected std::runtime_error from setState with unknown floating-joint id";
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_NE(std::string(e.what()).find("does_not_exist_floating"), std::string::npos)
        << "throw message did not name the offending joint id: " << e.what();
  }
}

TEST(TesseractStateSolverUnit, OFKTUnit)  // NOLINT
{
  OFKTStateSolver solver("test");
  EXPECT_TRUE(solver.getLinkIds().size() == 1);
  EXPECT_TRUE(solver.getLinkIds().at(0) == "test");
  EXPECT_TRUE(solver.getLinkTransform("test").isApprox(Eigen::Isometry3d::Identity(), 1e-6));
  EXPECT_TRUE(solver.getRevision() == 0);
  solver.setRevision(100);
  EXPECT_TRUE(solver.getRevision() == 100);
}

// =============================================================================
// Phase 2 test additions — SceneState integer-keyed tests
// =============================================================================

TEST(TesseractStateSolverUnit, SceneStateLinkIdTransformMapUnit)  // NOLINT
{
  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::scene_graph::test_suite::getSceneGraph(locator);

  OFKTStateSolver solver(*scene_graph);

  // getState() returns SceneState with LinkIdTransformMap
  const auto& state = solver.getState();

  // link_transforms is keyed by LinkId
  EXPECT_TRUE(state.link_transforms.count("base_link") > 0);

  // All link names should map to a LinkId entry in link_transforms
  for (const auto& link_id : solver.getLinkIds())
  {
    auto id = LinkId(link_id);
    EXPECT_TRUE(state.link_transforms.count(id) > 0) << "Missing LinkId entry for link: " << link_id.name();
  }

  // joints is keyed by JointId
  for (const auto& joint_id : solver.getActiveJointIds())
  {
    auto jid = JointId(joint_id);
    EXPECT_TRUE(state.joints.count(jid) > 0) << "Missing JointId entry for joint: " << joint_id.name();
  }

  // Verify getState(ids, values) also produces LinkIdTransformMap
  auto ids = solver.getActiveJointIds();
  Eigen::VectorXd values = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(ids.size()));
  values[0] = 0.3;
  auto new_state = solver.getState(ids, values);

  for (const auto& link_id : solver.getLinkIds())
  {
    EXPECT_TRUE(new_state.link_transforms.count(link_id) > 0);
  }
}

// Validates that KDLStateSolver::operator= rebuilds segment_id_cache_ using its own tree's
// pointers, not stale ones from the source. If the cache kept source pointers, FK on the
// clone would segfault or return garbage once the source is destroyed.
TEST(TesseractStateSolverUnit, KDLSegmentIdCacheCopyUnit)  // NOLINT
{
  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::scene_graph::test_suite::getSceneGraph(locator);

  // Take a reference FK result from a throwaway solver for later comparison
  tesseract::common::LinkIdTransformMap expected_transforms;
  std::vector<JointId> active_ids;
  Eigen::VectorXd values;
  {
    KDLStateSolver reference(*scene_graph);
    active_ids = reference.getActiveJointIds();
    values = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(active_ids.size()));
    for (Eigen::Index i = 0; i < values.size(); ++i)
      values[i] = 0.1 * static_cast<double>(i + 1);
    expected_transforms = reference.getState(active_ids, values).link_transforms;
  }

  // Build clone via copy-construction, then destroy the source.
  auto source = std::make_unique<KDLStateSolver>(*scene_graph);
  auto clone_via_copy = std::make_unique<KDLStateSolver>(*source);
  source.reset();  // freeing source's tree invalidates any stale pointers the clone might hold

  // FK on the clone must succeed (no segment_id_cache_.at() throw) and match the reference.
  SceneState clone_state;
  ASSERT_NO_THROW(clone_state = clone_via_copy->getState(active_ids, values));  // NOLINT
  EXPECT_EQ(clone_state.link_transforms.size(), expected_transforms.size());
  for (const auto& [link_id, expected_tf] : expected_transforms)
  {
    ASSERT_TRUE(clone_state.link_transforms.count(link_id) > 0) << "Missing link: " << link_id.name();
    EXPECT_TRUE(clone_state.link_transforms.at(link_id).isApprox(expected_tf, 1e-6))
        << "Transform mismatch for link: " << link_id.name();
  }

  // Repeat via clone() (which internally uses copy-construction / operator=).
  auto source2 = std::make_unique<KDLStateSolver>(*scene_graph);
  StateSolver::UPtr clone_via_clone = source2->clone();
  source2.reset();

  SceneState clone2_state;
  ASSERT_NO_THROW(clone2_state = clone_via_clone->getState(active_ids, values));  // NOLINT
  EXPECT_EQ(clone2_state.link_transforms.size(), expected_transforms.size());
  for (const auto& [link_id, expected_tf] : expected_transforms)
  {
    ASSERT_TRUE(clone2_state.link_transforms.count(link_id) > 0) << "Missing link: " << link_id.name();
    EXPECT_TRUE(clone2_state.link_transforms.at(link_id).isApprox(expected_tf, 1e-6))
        << "Transform mismatch for link: " << link_id.name();
  }
}

// Validates that OFKTStateSolver::isActiveLinkId (parent-chain walk) agrees with
// getActiveLinkIds() (full downward traversal) for every link in the scene, and that
// unknown ids return false. Locks down the equivalence after switching from a
// rebuild-and-find implementation to the O(depth) walk-up.
TEST(TesseractStateSolverUnit, OFKTIsActiveLinkIdMatchesActiveSetUnit)  // NOLINT
{
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::scene_graph::test_suite::getSceneGraph(locator);

  OFKTStateSolver solver(*scene_graph);

  const auto active = solver.getActiveLinkIds();
  const std::unordered_set<LinkId> active_set(active.begin(), active.end());

  // For every known link, the walk-up answer must match membership in getActiveLinkIds().
  // This implicitly covers the root (which is fixed-only and therefore not active) and
  // every active/static descendant produced by the recursive traversal.
  for (const auto& link_id : solver.getLinkIds())
  {
    const bool expected = active_set.count(link_id) > 0;
    EXPECT_EQ(solver.isActiveLinkId(link_id), expected) << "Mismatch for link: " << link_id.name();
  }

  // Probe an id that does not exist in the scene — must return false, not throw.
  EXPECT_FALSE(solver.isActiveLinkId(LinkId("does_not_exist_in_this_scene")));

  // The scene has at least one active link (the test fixture is a real robot), and the
  // base link must be reported as not active (its only ancestor chain is the root).
  ASSERT_FALSE(active.empty());
  EXPECT_TRUE(solver.isActiveLinkId(active.front()));
  EXPECT_FALSE(solver.isActiveLinkId(solver.getBaseLinkId()));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
