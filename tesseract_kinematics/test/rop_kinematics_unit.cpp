#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "kinematics_test_utils.h"

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
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

tesseract_kinematics::ForwardKinematics::UPtr
getRobotFwdKinematics(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  auto fwd_kin = std::make_unique<tesseract_kinematics::KDLFwdKinChain>();
  EXPECT_TRUE(fwd_kin->init(scene_graph, "base_link", "tool0", "manip"));
  return fwd_kin;
}

tesseract_kinematics::ForwardKinematics::UPtr getFullFwdKinematics(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  auto fwd_kin = std::make_unique<tesseract_kinematics::KDLFwdKinChain>();
  EXPECT_TRUE(fwd_kin->init(scene_graph, "positioner_base_link", "tool0", "robot_on_positioner"));
  return fwd_kin;
}

tesseract_kinematics::ForwardKinematics::UPtr
getPositionerFwdKinematics(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  auto fwd_kin = std::make_unique<tesseract_kinematics::KDLFwdKinChain>();
  EXPECT_TRUE(fwd_kin->init(scene_graph, "positioner_base_link", "positioner_tool0", "positioner"));
  return fwd_kin;
}

tesseract_kinematics::InverseKinematics::UPtr getFullInvKinematics(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  auto robot_fwd_kin = getRobotFwdKinematics(scene_graph);

  opw_kinematics::Parameters<double> opw_params = getOPWKinematicsParamABB();

  auto opw_kin = std::make_unique<tesseract_kinematics::OPWInvKin>();
  opw_kin->init("robot",
                opw_params,
                robot_fwd_kin->getBaseLinkName(),
                robot_fwd_kin->getTipLinkNames()[0],
                robot_fwd_kin->getJointNames());

  auto positioner_kin = getPositionerFwdKinematics(scene_graph);
  Eigen::VectorXd positioner_resolution = Eigen::VectorXd::Constant(1, 1, 0.1);
  auto rop_inv_kin = std::make_unique<tesseract_kinematics::RobotOnPositionerInvKin>();
  EXPECT_FALSE(rop_inv_kin->checkInitialized());
  rop_inv_kin->init(
      scene_graph, opw_kin->clone(), 2.5, positioner_kin->clone(), positioner_resolution, "robot_on_positioner");
  EXPECT_TRUE(rop_inv_kin->checkInitialized());

  {  // Test failure
    tesseract_scene_graph::SceneGraph scene_graph_empty;
    auto rop_inv_kin_failure = std::make_unique<tesseract_kinematics::RobotOnPositionerInvKin>();
    EXPECT_FALSE(rop_inv_kin_failure->init(scene_graph_empty,
                                           opw_kin->clone(),
                                           2.5,
                                           positioner_kin->clone(),
                                           positioner_resolution,
                                           "robot_on_positioner"));
    EXPECT_FALSE(rop_inv_kin_failure->checkInitialized());

    rop_inv_kin_failure = std::make_unique<tesseract_kinematics::RobotOnPositionerInvKin>();
    EXPECT_FALSE(rop_inv_kin_failure->init(
        scene_graph, nullptr, 2.5, positioner_kin->clone(), positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rop_inv_kin_failure->checkInitialized());

    rop_inv_kin_failure = std::make_unique<tesseract_kinematics::RobotOnPositionerInvKin>();
    EXPECT_FALSE(rop_inv_kin_failure->init(
        scene_graph, opw_kin->clone(), -2.5, positioner_kin->clone(), positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rop_inv_kin_failure->checkInitialized());

    rop_inv_kin_failure = std::make_unique<tesseract_kinematics::RobotOnPositionerInvKin>();
    EXPECT_FALSE(rop_inv_kin_failure->init(
        scene_graph, opw_kin->clone(), 2.5, nullptr, positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rop_inv_kin_failure->checkInitialized());

    positioner_resolution = Eigen::VectorXd();
    rop_inv_kin_failure = std::make_unique<tesseract_kinematics::RobotOnPositionerInvKin>();
    EXPECT_FALSE(rop_inv_kin_failure->init(
        scene_graph, opw_kin->clone(), 2.5, positioner_kin->clone(), positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rop_inv_kin_failure->checkInitialized());

    positioner_resolution = Eigen::VectorXd::Constant(1, -0.1);
    rop_inv_kin_failure = std::make_unique<tesseract_kinematics::RobotOnPositionerInvKin>();
    EXPECT_FALSE(rop_inv_kin_failure->init(
        scene_graph, opw_kin->clone(), 2.5, positioner_kin->clone(), positioner_resolution, "robot_on_positioner"));
    EXPECT_FALSE(rop_inv_kin_failure->checkInitialized());
  }

  return rop_inv_kin;
}

TEST(TesseractKinematicsUnit, RobotOnPositionerInverseKinematicUnit)  // NOLINT
{
  auto scene_graph = getSceneGraphABBOnPositioner();
  std::string manip_name = "robot_on_positioner";
  std::string base_link_name = "positioner_base_link";
  std::string tip_link_name = "tool0";
  std::vector<std::string> joint_names{
    "positioner_joint_1", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
  };

  auto fwd_kin = getFullFwdKinematics(*scene_graph);
  auto inv_kin = getFullInvKinematics(*scene_graph);

  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 1;
  pose.translation()[1] = 0;
  pose.translation()[2] = 1.306;

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(fwd_kin->numJoints());

  EXPECT_TRUE(inv_kin != nullptr);
  EXPECT_EQ(inv_kin->getName(), "robot_on_positioner");
  EXPECT_EQ(inv_kin->getSolverName(), "RobotOnPositionerInvKin");
  EXPECT_EQ(inv_kin->numJoints(), 7);
  EXPECT_EQ(inv_kin->getBaseLinkName(), "positioner_base_link");
  EXPECT_EQ(inv_kin->getTipLinkNames().size(), 1);
  EXPECT_EQ(inv_kin->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(inv_kin->getJointNames(), joint_names);

  runInvKinTest(*inv_kin, *fwd_kin, pose, base_link_name, tip_link_name, seed);

  // Check cloned
  tesseract_kinematics::InverseKinematics::Ptr inv_kin2 = inv_kin->clone();
  EXPECT_TRUE(inv_kin2 != nullptr);
  EXPECT_EQ(inv_kin2->getName(), "robot_on_positioner");
  EXPECT_EQ(inv_kin2->getSolverName(), "RobotOnPositionerInvKin");
  EXPECT_EQ(inv_kin2->numJoints(), 7);
  EXPECT_EQ(inv_kin2->getBaseLinkName(), "positioner_base_link");
  EXPECT_EQ(inv_kin2->getTipLinkNames().size(), 1);
  EXPECT_EQ(inv_kin2->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(inv_kin2->getJointNames(), joint_names);

  runInvKinTest(*inv_kin2, *fwd_kin, pose, base_link_name, tip_link_name, seed);

  //  // Check update
  //  inv_kin2->update();
  //  EXPECT_TRUE(inv_kin2 != nullptr);
  //  EXPECT_EQ(inv_kin2->getName(), "robot_on_positioner");
  //  EXPECT_EQ(inv_kin2->getSolverName(), "RobotOnPositionerInvKin");
  //  EXPECT_EQ(inv_kin2->numJoints(), 7);
  //  EXPECT_EQ(inv_kin2->getBaseLinkName(), "positioner_base_link");
  //  EXPECT_EQ(inv_kin2->getTipLinkName(), "tool0");

  //  runInvKinTest(*inv_kin2, *fwd_kin, pose, seed);
  //  runActiveLinkNamesABBOnPositionerTest(*inv_kin2);
  //  runKinJointLimitsTest(inv_kin2->getLimits(), target_limits);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
