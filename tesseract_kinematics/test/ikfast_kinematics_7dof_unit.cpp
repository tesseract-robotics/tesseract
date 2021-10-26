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
#include "iiwa7_ikfast_kinematics.h"
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>

using namespace tesseract_kinematics::test_suite;
using namespace tesseract_kinematics;

TEST(TesseractKinematicsUnit, IKFastInvKin7DOF)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.223;
  pose.translation()[1] = 0.354;
  pose.translation()[2] = 0.5;

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(7);

  // Setup test
  auto scene_graph = getSceneGraphIIWA7();
  std::string base_link_name = "link_0";
  std::string tip_link_name = "ikfast_tcp_link";
  std::vector<std::string> joint_names{ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7" };

  KDLFwdKinChain fwd_kin(*scene_graph, base_link_name, tip_link_name);

  std::vector<std::vector<double>> free_joint_states = { { -2.0 }, { -1.0 }, { 0.0 }, { 1.0 }, { 2.0 } };

  auto iiwa_inv_kin = std::make_shared<iiwa7Kinematics>(
      base_link_name, tip_link_name, joint_names, IKFAST_INV_KIN_CHAIN_SOLVER_NAME, free_joint_states);

  EXPECT_EQ(iiwa_inv_kin->getSolverName(), IKFAST_INV_KIN_CHAIN_SOLVER_NAME);
  EXPECT_EQ(iiwa_inv_kin->numJoints(), 7);
  EXPECT_EQ(iiwa_inv_kin->getBaseLinkName(), base_link_name);
  EXPECT_EQ(iiwa_inv_kin->getWorkingFrame(), base_link_name);
  EXPECT_EQ(iiwa_inv_kin->getTipLinkNames().size(), 1);
  EXPECT_EQ(iiwa_inv_kin->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(iiwa_inv_kin->getJointNames(), joint_names);

  runInvKinTest(*iiwa_inv_kin, fwd_kin, pose, tip_link_name, seed);

  // Check cloned
  InverseKinematics::Ptr iiwa_inv_kin2 = iiwa_inv_kin->clone();
  EXPECT_TRUE(iiwa_inv_kin2 != nullptr);
  EXPECT_EQ(iiwa_inv_kin2->getSolverName(), IKFAST_INV_KIN_CHAIN_SOLVER_NAME);
  EXPECT_EQ(iiwa_inv_kin2->numJoints(), 7);
  EXPECT_EQ(iiwa_inv_kin2->getBaseLinkName(), base_link_name);
  EXPECT_EQ(iiwa_inv_kin2->getWorkingFrame(), base_link_name);
  EXPECT_EQ(iiwa_inv_kin2->getTipLinkNames().size(), 1);
  EXPECT_EQ(iiwa_inv_kin2->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(iiwa_inv_kin2->getJointNames(), joint_names);

  runInvKinTest(*iiwa_inv_kin2, fwd_kin, pose, tip_link_name, seed);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
