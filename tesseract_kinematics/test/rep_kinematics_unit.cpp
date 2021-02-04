#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "kinematics_test_utils.h"

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>
#include <opw_kinematics/opw_parameters.h>

using namespace tesseract_kinematics::test_suite;

inline opw_kinematics::Parameters<double> getOPWKinematicsParamABB()
{
  opw_kinematics::Parameters<double> opw_params;
  opw_params.a1 = (0.100);
  opw_params.a2 = (-0.135);
  opw_params.b = (0.000);
  opw_params.c1 = (0.615);
  opw_params.c2 = (0.705);
  opw_params.c3 = (0.755);
  opw_params.c4 = (0.085);

  opw_params.offsets[2] = -M_PI / 2.0;

  return opw_params;
}

tesseract_kinematics::ForwardKinematics::Ptr
getRobotFwdKinematics(const tesseract_scene_graph::SceneGraph::Ptr& scene_graph)
{
  auto fwd_kin = std::make_shared<tesseract_kinematics::KDLFwdKinChain>();
  EXPECT_TRUE(fwd_kin->init(scene_graph, "world", "tool0", "manip"));
  return fwd_kin;
}

tesseract_kinematics::ForwardKinematics::Ptr
getFullFwdKinematics(const tesseract_scene_graph::SceneGraph::Ptr& scene_graph)
{
  auto fwd_kin = std::make_shared<tesseract_kinematics::KDLFwdKinChain>();
  EXPECT_TRUE(fwd_kin->init(scene_graph, "positioner_tool0", "tool0", "robot_external_positioner"));
  return fwd_kin;
}

tesseract_kinematics::ForwardKinematics::Ptr
getPositionerFwdKinematics(const tesseract_scene_graph::SceneGraph::Ptr& scene_graph)
{
  auto fwd_kin = std::make_shared<tesseract_kinematics::KDLFwdKinChain>();
  EXPECT_TRUE(fwd_kin->init(scene_graph, "world", "positioner_tool0", "positioner"));
  return fwd_kin;
}

tesseract_kinematics::InverseKinematics::Ptr
getFullInvKinematics(const tesseract_scene_graph::SceneGraph::Ptr& scene_graph, bool common_base = true)
{
  tesseract_common::TransformMap link_map;
  link_map["world"] = Eigen::Isometry3d::Identity();
  link_map["positioner_base_link"] = Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0, 1);

  auto robot_fwd_kin = getRobotFwdKinematics(scene_graph);
  opw_kinematics::Parameters<double> opw_params = getOPWKinematicsParamABB();

  auto opw_kin = std::make_shared<tesseract_kinematics::OPWInvKin>();
  opw_kin->init("robot",
                opw_params,
                robot_fwd_kin->getBaseLinkName(),
                robot_fwd_kin->getTipLinkName(),
                robot_fwd_kin->getJointNames(),
                robot_fwd_kin->getLinkNames(),
                robot_fwd_kin->getActiveLinkNames(),
                robot_fwd_kin->getLimits());

  auto positioner_kin = getPositionerFwdKinematics(scene_graph);
  Eigen::VectorXd positioner_resolution = Eigen::VectorXd::Constant(1, 1, 0.1);
  auto rep_inv_kin = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
  EXPECT_FALSE(rep_inv_kin->checkInitialized());
  if (common_base)
  {
    rep_inv_kin->init(scene_graph, opw_kin, 2.5, positioner_kin, positioner_resolution, "robot_external_positioner");
  }
  else
  {
    auto positioner_kin_secondary = std::make_shared<tesseract_kinematics::KDLFwdKinChain>();
    positioner_kin_secondary->init(scene_graph, "positioner_base_link", "positioner_tool0", "positioner");
    rep_inv_kin->init(scene_graph,
                      opw_kin,
                      2.5,
                      positioner_kin_secondary,
                      positioner_resolution,
                      link_map,
                      "robot_external_positioner");
  }
  EXPECT_TRUE(rep_inv_kin->getSceneGraph() == scene_graph);
  EXPECT_TRUE(rep_inv_kin->checkInitialized());

  {  // Test failure
    auto scene_graph_empty = std::make_shared<tesseract_scene_graph::SceneGraph>();
    auto rep_inv_kin_failure = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
    EXPECT_FALSE(rep_inv_kin_failure->init(
        scene_graph_empty, opw_kin, 2.5, positioner_kin, positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rep_inv_kin_failure->checkInitialized());

    rep_inv_kin_failure = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
    EXPECT_FALSE(
        rep_inv_kin_failure->init(nullptr, opw_kin, 2.5, positioner_kin, positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rep_inv_kin_failure->checkInitialized());

    rep_inv_kin_failure = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
    EXPECT_FALSE(rep_inv_kin_failure->init(
        scene_graph, nullptr, 2.5, positioner_kin, positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rep_inv_kin_failure->checkInitialized());
    EXPECT_FALSE(rep_inv_kin_failure->init(
        scene_graph, nullptr, 2.5, positioner_kin, positioner_resolution, link_map, "robot_on_positioner"));
    EXPECT_FALSE(rep_inv_kin_failure->checkInitialized());

    rep_inv_kin_failure = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
    EXPECT_FALSE(rep_inv_kin_failure->init(
        scene_graph, opw_kin, -2.5, positioner_kin, positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rep_inv_kin_failure->checkInitialized());

    rep_inv_kin_failure = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
    EXPECT_FALSE(
        rep_inv_kin_failure->init(scene_graph, opw_kin, 2.5, nullptr, positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rep_inv_kin_failure->checkInitialized());
    EXPECT_FALSE(rep_inv_kin_failure->init(
        scene_graph, opw_kin, 2.5, nullptr, positioner_resolution, link_map, "robot_on_positioner"));
    EXPECT_FALSE(rep_inv_kin_failure->checkInitialized());

    rep_inv_kin_failure = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
    auto positioner_kin_failure = std::make_shared<tesseract_kinematics::KDLFwdKinChain>();
    positioner_kin_failure->init(scene_graph, "positioner_base_link", "positioner_tool0", "positioner");
    EXPECT_FALSE(rep_inv_kin_failure->init(
        scene_graph, opw_kin, 2.5, positioner_kin_failure, positioner_resolution, "robot_on_positioner"));

    positioner_resolution = Eigen::VectorXd();
    rep_inv_kin_failure = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
    EXPECT_FALSE(rep_inv_kin_failure->init(
        scene_graph, opw_kin, 2.5, positioner_kin, positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rep_inv_kin_failure->checkInitialized());

    positioner_resolution = Eigen::VectorXd::Constant(1, -0.1);
    rep_inv_kin_failure = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
    EXPECT_FALSE(rep_inv_kin_failure->init(
        scene_graph, opw_kin, 2.5, positioner_kin, positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rep_inv_kin_failure->checkInitialized());
  }

  return rep_inv_kin;
}

TEST(TesseractKinematicsUnit, RobotWithExternalPositionerInverseKinematicUnit)  // NOLINT
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraphABBExternalPositioner();

  auto fwd_kin = getFullFwdKinematics(scene_graph);
  auto inv_kin = getFullInvKinematics(scene_graph);
  auto inv_kin2 = getFullInvKinematics(scene_graph, false);

  const std::vector<std::string>& fwd_joint_names = fwd_kin->getJointNames();
  const std::vector<std::string>& inv_joint_names = inv_kin->getJointNames();
  const std::vector<std::string>& inv2_joint_names = inv_kin2->getJointNames();

  // TODO: Need to add a way to synchronize two kinematics so joints are in the same order.
  EXPECT_TRUE(std::equal(fwd_joint_names.begin(), fwd_joint_names.end(), inv_joint_names.begin()));
  EXPECT_TRUE(std::equal(fwd_joint_names.begin(), fwd_joint_names.end(), inv2_joint_names.begin()));

  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.1;

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(fwd_kin->numJoints());

  EXPECT_TRUE(inv_kin != nullptr);
  EXPECT_EQ(inv_kin->getName(), "robot_external_positioner");
  EXPECT_EQ(inv_kin->getSolverName(), "RobotWithExternalPositionerInvKin");
  EXPECT_EQ(inv_kin->numJoints(), 7);
  EXPECT_EQ(inv_kin->getBaseLinkName(), "positioner_tool0");
  EXPECT_EQ(inv_kin->getTipLinkName(), "tool0");
  tesseract_common::KinematicLimits target_limits = getTargetLimits(scene_graph, inv_kin->getJointNames());

  runInvKinTest(*inv_kin, *fwd_kin, pose, seed);
  runActiveLinkNamesABBExternalPositionerTest(*inv_kin);
  runKinJointLimitsTest(inv_kin->getLimits(), target_limits);

  // Inverse Kinematics using different init function
  EXPECT_TRUE(inv_kin2 != nullptr);
  EXPECT_EQ(inv_kin2->getName(), "robot_external_positioner");
  EXPECT_EQ(inv_kin2->getSolverName(), "RobotWithExternalPositionerInvKin");
  EXPECT_EQ(inv_kin2->numJoints(), 7);
  EXPECT_EQ(inv_kin2->getBaseLinkName(), "positioner_tool0");
  EXPECT_EQ(inv_kin2->getTipLinkName(), "tool0");

  runInvKinTest(*inv_kin2, *fwd_kin, pose, seed);
  runActiveLinkNamesABBExternalPositionerTest(*inv_kin2);
  runKinJointLimitsTest(inv_kin2->getLimits(), target_limits);

  // Check cloned
  tesseract_kinematics::InverseKinematics::Ptr inv_kin3 = inv_kin->clone();
  EXPECT_TRUE(inv_kin3 != nullptr);
  EXPECT_EQ(inv_kin3->getName(), "robot_external_positioner");
  EXPECT_EQ(inv_kin3->getSolverName(), "RobotWithExternalPositionerInvKin");
  EXPECT_EQ(inv_kin3->numJoints(), 7);
  EXPECT_EQ(inv_kin3->getBaseLinkName(), "positioner_tool0");
  EXPECT_EQ(inv_kin3->getTipLinkName(), "tool0");

  runInvKinTest(*inv_kin3, *fwd_kin, pose, seed);
  runActiveLinkNamesABBExternalPositionerTest(*inv_kin3);
  runKinJointLimitsTest(inv_kin3->getLimits(), target_limits);

  // Check update
  inv_kin3->update();
  EXPECT_TRUE(inv_kin3 != nullptr);
  EXPECT_EQ(inv_kin3->getName(), "robot_external_positioner");
  EXPECT_EQ(inv_kin3->getSolverName(), "RobotWithExternalPositionerInvKin");
  EXPECT_EQ(inv_kin3->numJoints(), 7);
  EXPECT_EQ(inv_kin3->getBaseLinkName(), "positioner_tool0");
  EXPECT_EQ(inv_kin3->getTipLinkName(), "tool0");

  runInvKinTest(*inv_kin3, *fwd_kin, pose, seed);
  runActiveLinkNamesABBExternalPositionerTest(*inv_kin3);
  runKinJointLimitsTest(inv_kin3->getLimits(), target_limits);

  // Test setJointLimits
  runKinSetJointLimitsTest(*inv_kin);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
