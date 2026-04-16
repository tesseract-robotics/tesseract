#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/state_solver/ofkt/ofkt_nodes.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>
#include "state_solver_test_suite.h"

using namespace tesseract::scene_graph;
using tesseract::common::JointId;
using tesseract::common::LinkId;

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
    OFKTFixedNode node(
        &root_node, "base_link", "joint_a1", Eigen::Isometry3d::Identity());
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
    OFKTFloatingNode node(
        &root_node, "base_link", "joint_a1", Eigen::Isometry3d::Identity());
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
    OFKTRevoluteNode node(&root_node,
                          "base_link",
                          "joint_a1",
                          Eigen::Isometry3d::Identity(),
                          Eigen::Vector3d(0, 0, 1));
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
    OFKTContinuousNode node(&root_node,
                            "base_link",
                            "joint_a1",
                            Eigen::Isometry3d::Identity(),
                            Eigen::Vector3d(0, 0, 1));
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
    OFKTPrismaticNode node(&root_node,
                           "base_link",
                           "joint_a1",
                           Eigen::Isometry3d::Identity(),
                           Eigen::Vector3d(1, 0, 0));
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

TEST(TesseractStateSolverUnit, OFKTUnit)  // NOLINT
{
  OFKTStateSolver solver("test");
  EXPECT_TRUE(solver.getLinkNames().size() == 1);
  auto ln = tesseract::common::LinkId("test");
  EXPECT_TRUE(solver.getLinkNames().at(0) == ln.name());
  EXPECT_TRUE(solver.getLinkTransform(ln).isApprox(Eigen::Isometry3d::Identity(), 1e-6));
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
  for (const auto& link_name : solver.getLinkNames())
  {
    auto id = LinkId(link_name);
    EXPECT_TRUE(state.link_transforms.count(id) > 0) << "Missing LinkId entry for link: " << link_name;
  }

  // joints is keyed by JointId
  for (const auto& joint_name : solver.getActiveJointNames())
  {
    auto jid = JointId(joint_name);
    EXPECT_TRUE(state.joints.count(jid) > 0) << "Missing JointId entry for joint: " << joint_name;
  }

  // Verify getState(names, values) also produces LinkIdTransformMap
  auto names = solver.getActiveJointNames();
  Eigen::VectorXd values = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(names.size()));
  values[0] = 0.3;
  auto new_state = solver.getState(names, values);

  for (const auto& link_name : solver.getLinkNames())
  {
    EXPECT_TRUE(new_state.link_transforms.count(LinkId(link_name)) > 0);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
