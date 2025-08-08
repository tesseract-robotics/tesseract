/**
 * @file opw_kinematics_unit.cpp
 * @brief Tesseract opw kinematics test
 *
 * @author Levi Armstrong
 * @date Feb 4, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "kinematics_test_utils.h"
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <opw_kinematics/opw_parameters.h>
#include <tesseract_kinematics/core/kinematic_group.h>

using namespace tesseract_kinematics::test_suite;
using namespace tesseract_kinematics;

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

TEST(TesseractKinematicsUnit, OPWInvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 1;
  pose.translation()[1] = 0;
  pose.translation()[2] = 1.306;

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(6);

  // Setup test
  tesseract_common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABB(locator);
  std::string manip_name = "manip";
  std::string base_link_name = "base_link";
  std::string tip_link_name = "tool0";
  std::vector<std::string> joint_names{ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  KDLFwdKinChain fwd_kin(*scene_graph, base_link_name, tip_link_name);

  opw_kinematics::Parameters<double> opw_params = getOPWKinematicsParamABB();

  auto inv_kin = std::make_shared<OPWInvKin>(opw_params, base_link_name, tip_link_name, joint_names);

  EXPECT_EQ(inv_kin->getSolverName(), OPW_INV_KIN_CHAIN_SOLVER_NAME);
  EXPECT_EQ(inv_kin->numJoints(), 6);
  EXPECT_EQ(inv_kin->getBaseLinkName(), base_link_name);
  EXPECT_EQ(inv_kin->getWorkingFrame(), base_link_name);
  EXPECT_EQ(inv_kin->getTipLinkNames().size(), 1);
  EXPECT_EQ(inv_kin->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(inv_kin->getJointNames(), joint_names);

  runInvKinTest(*inv_kin, fwd_kin, pose, tip_link_name, seed);

  // Check cloned
  InverseKinematics::Ptr inv_kin2 = inv_kin->clone();
  EXPECT_TRUE(inv_kin2 != nullptr);
  EXPECT_EQ(inv_kin2->getSolverName(), OPW_INV_KIN_CHAIN_SOLVER_NAME);
  EXPECT_EQ(inv_kin2->numJoints(), 6);
  EXPECT_EQ(inv_kin2->getBaseLinkName(), base_link_name);
  EXPECT_EQ(inv_kin2->getWorkingFrame(), base_link_name);
  EXPECT_EQ(inv_kin2->getTipLinkNames().size(), 1);
  EXPECT_EQ(inv_kin2->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(inv_kin2->getJointNames(), joint_names);

  runInvKinTest(*inv_kin2, fwd_kin, pose, tip_link_name, seed);
}

TEST(TesseractKinematicsUnit, OPWInvKinGroupUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 1;
  pose.translation()[1] = 0;
  pose.translation()[2] = 1.306;

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(6);

  // Setup test
  tesseract_common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABB(locator);
  std::string manip_name = "manip";
  std::string base_link_name = "base_link";
  std::string tip_link_name = "tool0";
  std::vector<std::string> joint_names{ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  std::vector<std::string> invalid_joint_names{
    "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"
  };

  opw_kinematics::Parameters<double> opw_params = getOPWKinematicsParamABB();

  auto inv_kin = std::make_unique<OPWInvKin>(opw_params, base_link_name, tip_link_name, joint_names);

  tesseract_scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  // Check Inverse Kinematics
  KinematicGroup kin_group(manip_name, joint_names, std::move(inv_kin), *scene_graph, scene_state);

  runInvKinTest(kin_group, pose, base_link_name, tip_link_name, seed);

  auto abb_joint_2 = scene_graph->getJoint("joint_2");
  abb_joint_2->limits->lower = -3.49065;
  abb_joint_2->limits->upper = 1.57079;

  auto inv_kin2 = std::make_unique<OPWInvKin>(opw_params, base_link_name, tip_link_name, joint_names);
  KinematicGroup kin_group2(manip_name, joint_names, std::move(inv_kin2), *scene_graph, scene_state);

  Eigen::Isometry3d pose2;
  pose2.setIdentity();
  pose2.translation()[0] = -0.268141;
  pose2.translation()[1] = -0.023459;
  pose2.translation()[2] = -0.753010;
  pose2.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();

  runInvKinTest(kin_group2, pose2, base_link_name, tip_link_name, seed);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
