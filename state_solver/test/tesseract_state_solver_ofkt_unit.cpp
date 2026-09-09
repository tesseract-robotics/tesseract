#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <unordered_set>
#include <thread>
#include <atomic>
#include <kdl/segment.hpp>
#include <unordered_map>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/state_solver/ofkt/ofkt_nodes.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>
#include <tesseract/scene_graph/kdl_parser.h>
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

// Every OFKT entrypoint that overlays user-supplied floating-joint values must reject an unknown
// joint id with a std::runtime_error whose message names that id, so callers can identify the bad
// entry from the exception alone.
TEST(TesseractStateSolverUnit, OFKTApplyFloatingValuesUnknownIdThrows)  // NOLINT
{
  using tesseract::common::JointIdTransformMap;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::scene_graph::test_suite::getSceneGraph(locator);
  OFKTStateSolver solver(*scene_graph);

  JointIdTransformMap bad;
  bad["does_not_exist_floating"] = Eigen::Isometry3d::Identity();

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
// SceneState integer-keyed tests
// =============================================================================

TEST(TesseractStateSolverUnit, SceneStateLinkIdTransformMapUnit)  // NOLINT
{
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
    EXPECT_TRUE(state.link_transforms.count(link_id) > 0) << "Missing LinkId entry for link: " << link_id;
  }

  // joints is keyed by JointId
  for (const auto& joint_id : solver.getActiveJointIds())
  {
    EXPECT_TRUE(state.joints.count(joint_id) > 0) << "Missing JointId entry for joint: " << joint_id;
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
    ASSERT_TRUE(clone_state.link_transforms.count(link_id) > 0) << "Missing link: " << link_id;
    EXPECT_TRUE(clone_state.link_transforms.at(link_id).isApprox(expected_tf, 1e-6))
        << "Transform mismatch for link: " << link_id;
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
    ASSERT_TRUE(clone2_state.link_transforms.count(link_id) > 0) << "Missing link: " << link_id;
    EXPECT_TRUE(clone2_state.link_transforms.at(link_id).isApprox(expected_tf, 1e-6))
        << "Transform mismatch for link: " << link_id;
  }
}

// Validates that OFKTStateSolver::isActiveLinkId (parent-chain walk) agrees with
// getActiveLinkIds() (full downward traversal) for every link in the scene, and that
// unknown ids return false.
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
    EXPECT_EQ(solver.isActiveLinkId(link_id), expected) << "Mismatch for link: " << link_id;
  }

  // Probe an id that does not exist in the scene — must return false, not throw.
  EXPECT_FALSE(solver.isActiveLinkId("does_not_exist_in_this_scene"));

  // The scene has at least one active link (the test fixture is a real robot), and the
  // base link must be reported as not active (its only ancestor chain is the root).
  ASSERT_FALSE(active.empty());
  EXPECT_TRUE(solver.isActiveLinkId(active.front()));
  EXPECT_FALSE(solver.isActiveLinkId(solver.getBaseLinkId()));
}

// Concurrent callers must each get the jacobian for the configuration they asked about. KDL caches
// joint poses inside the solver, so a solver shared between threads returns another thread's answer.
TEST(TesseractStateSolverUnit, KDLJacobianIsThreadSafeUnit)  // NOLINT
{
  if (std::thread::hardware_concurrency() < 2)
    GTEST_SKIP() << "needs more than one core to exercise concurrent access";

  tesseract::common::GeneralResourceLocator locator;
  SceneGraph::UPtr scene_graph = test_suite::getSceneGraph(locator);
  const KDLStateSolver solver(*scene_graph);

  const std::vector<tesseract::common::JointId> joint_ids = solver.getActiveJointIds();
  const tesseract::common::LinkId link_id("tool0");
  ASSERT_TRUE(solver.isActiveLinkId(link_id));
  const auto dof = static_cast<Eigen::Index>(joint_ids.size());
  ASSERT_GT(dof, 0);

  std::vector<Eigen::VectorXd> configs;
  std::vector<Eigen::MatrixXd> expected;
  for (int i = 0; i < 32; ++i)
  {
    Eigen::VectorXd q(dof);
    for (Eigen::Index j = 0; j < dof; ++j)
      q[j] = -1.2 + (0.37 * static_cast<double>((static_cast<Eigen::Index>(i) * 7 + j * 3) % 11));

    configs.push_back(q);
    expected.push_back(solver.getJacobian(joint_ids, q, link_id));
    // A jacobian of zeros would satisfy the comparison below whatever the solver returned.
    ASSERT_GT(expected.back().norm(), 1e-3);
  }

  constexpr int kThreads = 8;
  constexpr int kIterations = 500;
  std::atomic<int> mismatches{ 0 };
  std::vector<std::thread> workers;
  workers.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t)
  {
    workers.emplace_back([&, t] {
      for (int i = 0; i < kIterations; ++i)
      {
        const auto k = static_cast<std::size_t>((i * 13 + t * 5) % static_cast<int>(configs.size()));
        const Eigen::MatrixXd jacobian = solver.getJacobian(joint_ids, configs[k], link_id);
        // Same inputs through the same code path, so the answer is bit-identical or it is a race.
        if (jacobian != expected[k])
          ++mismatches;
      }
    });
  }

  for (auto& worker : workers)
    worker.join();

  EXPECT_EQ(mismatches.load(), 0);
}

namespace
{
/** @brief Walk a KDL tree the way KDL itself does, as an independent oracle for link transforms */
void poseTraversal(const KDL::Tree& tree,
                   const KDL::JntArray& q,
                   const KDL::SegmentMap::const_iterator& it,
                   const Eigen::Isometry3d& parent,
                   tesseract::common::LinkIdTransformMap& transforms)
{
  if (it == tree.getSegments().end())
    return;

  const KDL::TreeElementType& element = it->second;
  const KDL::Segment& segment = GetTreeElementSegment(element);
  const Eigen::Isometry3d global =
      parent * tesseract::scene_graph::convert(segment.pose(q(GetTreeElementQNr(element))));
  transforms[tesseract::common::LinkId(it->first)] = global;

  for (const auto& child : element.children)
    poseTraversal(tree, q, child, global, transforms);
}
}  // namespace

// The link transforms must match what KDL's own segment pose produces, for every link and every
// configuration - the solver computes the joint frame itself rather than asking KDL for it.
TEST(TesseractStateSolverUnit, KDLLinkTransformsMatchSegmentPoseUnit)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  SceneGraph::UPtr scene_graph = test_suite::getSceneGraph(locator);
  const KDLStateSolver solver(*scene_graph);
  tesseract::scene_graph::KDLTreeData data = tesseract::scene_graph::parseSceneGraph(*scene_graph);

  const std::vector<tesseract::common::JointId> joint_ids = solver.getActiveJointIds();
  const auto dof = static_cast<Eigen::Index>(joint_ids.size());
  ASSERT_GT(dof, 0);

  // KDL assigns each joint a q index by tree construction order, which is not the order joint ids
  // come back in. Map by name, or the oracle evaluates a different configuration than the solver.
  std::unordered_map<std::string, unsigned> name_to_qnr;
  for (const auto& seg : data.tree.getSegments())
  {
    const KDL::Joint& joint = seg.second.segment.getJoint();
    if (joint.getType() != KDL::Joint::None)
      name_to_qnr[joint.getName()] = seg.second.q_nr;
  }
  ASSERT_EQ(name_to_qnr.size(), joint_ids.size());

  for (int i = 0; i < 64; ++i)
  {
    Eigen::VectorXd q(dof);
    for (Eigen::Index j = 0; j < dof; ++j)
      q[j] = -2.5 + (0.41 * static_cast<double>((static_cast<Eigen::Index>(i) * 5 + j * 7) % 13));

    KDL::JntArray kdl_q(data.tree.getNrOfJoints());
    for (Eigen::Index j = 0; j < dof; ++j)
      kdl_q(name_to_qnr.at(joint_ids[static_cast<std::size_t>(j)].name())) = q[j];

    tesseract::common::LinkIdTransformMap expected;
    poseTraversal(data.tree, kdl_q, data.tree.getRootSegment(), Eigen::Isometry3d::Identity(), expected);

    tesseract::common::LinkIdTransformMap actual;
    solver.getLinkTransforms(actual, joint_ids, q);

    ASSERT_FALSE(expected.empty());
    for (const auto& pair : expected)
      EXPECT_TRUE(actual.at(pair.first).isApprox(pair.second, 1e-12)) << "link " << pair.first.name();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
