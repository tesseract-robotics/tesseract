#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "kinematics_test_utils.h"

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr_factory.h>

using namespace tesseract_kinematics::test_suite;

TEST(TesseractKinematicsUnit, KDLKinChainUnit)  // NOLINT
{
  // Check initialized
  tesseract_kinematics::KDLFwdKinChain derived_kin;
  EXPECT_FALSE(derived_kin.checkInitialized());

  auto scene_graph_empty = std::make_shared<tesseract_scene_graph::SceneGraph>();
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraphIIWA();
  tesseract_kinematics::KDLFwdKinChainFactory kin_factory;
  EXPECT_EQ(kin_factory.getName(), "KDLFwdKinChain");
  EXPECT_EQ(kin_factory.getType(), tesseract_kinematics::ForwardKinematicsFactoryType::CHAIN);

  bool all_poses_supported = true;
#ifdef KDL_LESS_1_4_0
  all_poses_supported = false;
#endif

  EXPECT_TRUE(derived_kin.init(scene_graph, "base_link", "tool0", "manip"));
  EXPECT_TRUE(derived_kin.getSceneGraph() == scene_graph);

  // Check create method with empty scene graph
  tesseract_kinematics::ForwardKinematics::Ptr kin_empty =
      kin_factory.create(scene_graph_empty, "base_link", "tool0", "manip");
  EXPECT_TRUE(kin_empty == nullptr);

  // Check create method using base_link and tool0
  tesseract_kinematics::ForwardKinematics::Ptr kin = kin_factory.create(scene_graph, "base_link", "tool0", "manip");
  tesseract_common::KinematicLimits target_limits = getTargetLimits(scene_graph, kin->getJointNames());
  EXPECT_TRUE(kin != nullptr);
  EXPECT_EQ(kin->getName(), "manip");
  EXPECT_EQ(kin->getSolverName(), "KDLFwdKinChain");
  EXPECT_EQ(kin->numJoints(), 7);
  EXPECT_EQ(kin->getBaseLinkName(), "base_link");
  EXPECT_EQ(kin->getTipLinkName(), "tool0");

  runFwdKinIIWATest(*kin);
  runFwdKinAllPosesIIWATest(*kin, all_poses_supported);
  runJacobianIIWATest(*kin);
  runActiveLinkNamesIIWATest(*kin, false);
  runKinJointLimitsTest(kin->getLimits(), target_limits);

  // Check create method using chain pairs
  tesseract_kinematics::ForwardKinematics::Ptr kin2 =
      kin_factory.create(scene_graph, { std::make_pair("base_link", "tool0") }, "manip");
  target_limits = getTargetLimits(scene_graph, kin2->getJointNames());
  EXPECT_TRUE(kin2 != nullptr);
  EXPECT_EQ(kin2->getName(), "manip");
  EXPECT_EQ(kin2->getSolverName(), "KDLFwdKinChain");
  EXPECT_EQ(kin2->numJoints(), 7);
  EXPECT_EQ(kin2->getBaseLinkName(), "base_link");
  EXPECT_EQ(kin2->getTipLinkName(), "tool0");

  runFwdKinIIWATest(*kin2);
  runFwdKinAllPosesIIWATest(*kin2, all_poses_supported);
  runJacobianIIWATest(*kin2);
  runActiveLinkNamesIIWATest(*kin2, false);
  runKinJointLimitsTest(kin2->getLimits(), target_limits);

  // Checked cloned
  tesseract_kinematics::ForwardKinematics::Ptr kin3 = kin->clone();
  target_limits = getTargetLimits(scene_graph, kin3->getJointNames());
  EXPECT_TRUE(kin3 != nullptr);
  EXPECT_EQ(kin3->getName(), "manip");
  EXPECT_EQ(kin3->getSolverName(), "KDLFwdKinChain");
  EXPECT_EQ(kin3->numJoints(), 7);
  EXPECT_EQ(kin3->getBaseLinkName(), "base_link");
  EXPECT_EQ(kin3->getTipLinkName(), "tool0");

  runFwdKinIIWATest(*kin3);
  runFwdKinAllPosesIIWATest(*kin3, all_poses_supported);
  runJacobianIIWATest(*kin3);
  runActiveLinkNamesIIWATest(*kin3, false);
  runKinJointLimitsTest(kin3->getLimits(), target_limits);

  // Checked update
  kin3->update();
  target_limits = getTargetLimits(scene_graph, kin3->getJointNames());
  EXPECT_TRUE(kin3 != nullptr);
  EXPECT_EQ(kin3->getName(), "manip");
  EXPECT_EQ(kin3->getSolverName(), "KDLFwdKinChain");
  EXPECT_EQ(kin3->numJoints(), 7);
  EXPECT_EQ(kin3->getBaseLinkName(), "base_link");
  EXPECT_EQ(kin3->getTipLinkName(), "tool0");

  runFwdKinIIWATest(*kin3);
  runFwdKinAllPosesIIWATest(*kin3, all_poses_supported);
  runJacobianIIWATest(*kin3);
  runActiveLinkNamesIIWATest(*kin3, false);
  runKinJointLimitsTest(kin3->getLimits(), target_limits);

  // Test setJointLimits
  runKinSetJointLimitsTest(*kin);

  // Test failure
  kin = kin_factory.create(scene_graph, "missing_link", "tool0", "manip");
  EXPECT_TRUE(kin == nullptr);

  kin2 = kin_factory.create(scene_graph, { std::make_pair("missing_link", "tool0") }, "manip");
  EXPECT_TRUE(kin2 == nullptr);

  kin = kin_factory.create(nullptr, "base_link", "tool0", "manip");
  EXPECT_TRUE(kin == nullptr);

  kin2 = kin_factory.create(nullptr, { std::make_pair("base_link", "tool0") }, "manip");
  EXPECT_TRUE(kin2 == nullptr);

  EXPECT_ANY_THROW(derived_kin.calcJacobian(Eigen::VectorXd::Zero(7), "missing_link"));  // NOLINT
}

TEST(TesseractKinematicsUnit, KDLKinTreeUnit)  // NOLINT
{
  tesseract_kinematics::KDLFwdKinTree derived_kin;
  EXPECT_FALSE(derived_kin.checkInitialized());

  auto scene_graph_empty = std::make_shared<tesseract_scene_graph::SceneGraph>();
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraphIIWA();
  tesseract_kinematics::KDLFwdKinTreeFactory kin_factory;
  EXPECT_EQ(kin_factory.getName(), "KDLFwdKinTree");
  EXPECT_EQ(kin_factory.getType(), tesseract_kinematics::ForwardKinematicsFactoryType::TREE);

  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };

  std::unordered_map<std::string, double> start_state;
  start_state["joint_a1"] = 0;
  start_state["joint_a2"] = 0;
  start_state["joint_a3"] = 0;
  start_state["joint_a4"] = 0;
  start_state["joint_a5"] = 0;
  start_state["joint_a6"] = 0;
  start_state["joint_a7"] = 0;

  EXPECT_TRUE(derived_kin.init(scene_graph, joint_names, "manip", start_state));
  EXPECT_TRUE(derived_kin.getSceneGraph() == scene_graph);

  // Check create method with empty scene graph
  tesseract_kinematics::ForwardKinematics::Ptr kin_empty =
      kin_factory.create(scene_graph_empty, joint_names, "manip", start_state);
  EXPECT_TRUE(kin_empty == nullptr);

  tesseract_kinematics::ForwardKinematics::Ptr kin = kin_factory.create(scene_graph, joint_names, "manip", start_state);
  tesseract_common::KinematicLimits target_limits = getTargetLimits(scene_graph, kin->getJointNames());
  EXPECT_TRUE(kin != nullptr);
  EXPECT_EQ(kin->getName(), "manip");
  EXPECT_EQ(kin->getSolverName(), "KDLFwdKinTree");
  EXPECT_EQ(kin->numJoints(), 7);
  EXPECT_EQ(kin->getBaseLinkName(), scene_graph->getRoot());
  EXPECT_TRUE(scene_graph->getLink(kin->getTipLinkName()) != nullptr);

  runActiveLinkNamesIIWATest(*kin, true);
  runFwdKinIIWATest(*kin);
  runJacobianIIWATest(*kin, true);
  runKinJointLimitsTest(kin->getLimits(), target_limits);

  // Check cloned
  tesseract_kinematics::ForwardKinematics::Ptr kin2 = kin->clone();
  target_limits = getTargetLimits(scene_graph, kin2->getJointNames());
  EXPECT_TRUE(kin2 != nullptr);
  EXPECT_EQ(kin2->getName(), "manip");
  EXPECT_EQ(kin2->getSolverName(), "KDLFwdKinTree");
  EXPECT_EQ(kin2->numJoints(), 7);
  EXPECT_EQ(kin2->getBaseLinkName(), scene_graph->getRoot());
  EXPECT_TRUE(scene_graph->getLink(kin2->getTipLinkName()) != nullptr);

  runActiveLinkNamesIIWATest(*kin2, true);
  runFwdKinIIWATest(*kin2);
  runJacobianIIWATest(*kin2, true);
  runKinJointLimitsTest(kin2->getLimits(), target_limits);

  // Check update
  kin2->update();
  target_limits = getTargetLimits(scene_graph, kin2->getJointNames());
  EXPECT_TRUE(kin2 != nullptr);
  EXPECT_EQ(kin2->getName(), "manip");
  EXPECT_EQ(kin2->getSolverName(), "KDLFwdKinTree");
  EXPECT_EQ(kin2->numJoints(), 7);
  EXPECT_EQ(kin2->getBaseLinkName(), scene_graph->getRoot());
  EXPECT_TRUE(scene_graph->getLink(kin2->getTipLinkName()) != nullptr);

  runActiveLinkNamesIIWATest(*kin2, true);
  runFwdKinIIWATest(*kin2);
  runJacobianIIWATest(*kin2, true);
  runKinJointLimitsTest(kin2->getLimits(), target_limits);

  // Test setJointLimits
  runKinSetJointLimitsTest(*kin);

  // Test failure
  kin = kin_factory.create(nullptr, joint_names, "manip", start_state);
  EXPECT_TRUE(kin == nullptr);

  joint_names[0] = "missing_joint";
  kin = kin_factory.create(scene_graph, joint_names, "manip", start_state);
  EXPECT_TRUE(kin == nullptr);

  joint_names.clear();
  kin = kin_factory.create(scene_graph, joint_names, "manip", start_state);
  EXPECT_TRUE(kin == nullptr);

  EXPECT_ANY_THROW(derived_kin.calcJacobian(Eigen::VectorXd::Zero(7), "missing_link"));
}

TEST(TesseractKinematicsUnit, KDLKinChainLMAInverseKinematicUnit)  // NOLINT
{
  tesseract_kinematics::KDLInvKinChainLMA derived_kin;
  EXPECT_FALSE(derived_kin.checkInitialized());

  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraphIIWA();

  EXPECT_TRUE(derived_kin.init(scene_graph, "base_link", "tool0", "manip"));
  EXPECT_TRUE(derived_kin.getSceneGraph() == scene_graph);

  tesseract_kinematics::KDLFwdKinChainFactory fwd_kin_factory;
  tesseract_kinematics::KDLInvKinChainLMAFactory inv_kin_factory;
  runInvKinIIWATest(
      inv_kin_factory, fwd_kin_factory, "KDLInvKinChainLMA", tesseract_kinematics::InverseKinematicsFactoryType::CHAIN);
}

TEST(TesseractKinematicsUnit, KDLKinChainNRInverseKinematicUnit)  // NOLINT
{
  tesseract_kinematics::KDLInvKinChainNR derived_kin;
  EXPECT_FALSE(derived_kin.checkInitialized());

  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraphIIWA();

  EXPECT_TRUE(derived_kin.init(scene_graph, "base_link", "tool0", "manip"));
  EXPECT_TRUE(derived_kin.getSceneGraph() == scene_graph);

  tesseract_kinematics::KDLFwdKinChainFactory fwd_kin_factory;
  tesseract_kinematics::KDLInvKinChainNRFactory inv_kin_factory;
  runInvKinIIWATest(
      inv_kin_factory, fwd_kin_factory, "KDLInvKinChainNR", tesseract_kinematics::InverseKinematicsFactoryType::CHAIN);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
