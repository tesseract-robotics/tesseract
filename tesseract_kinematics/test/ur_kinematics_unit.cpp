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
#include <tesseract_kinematics/ur/ur_inv_kin.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>

using namespace tesseract_kinematics::test_suite;
using namespace tesseract_kinematics;

void runURKinematicsTests(const URParameters& params,
                          double shoulder_offset,
                          double elbow_offset,
                          const Eigen::Isometry3d& pose)
{
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(6);

  // Setup test
  auto scene_graph = getSceneGraphUR(params, shoulder_offset, elbow_offset);

  std::string manip_name = "manip";
  std::string base_link_name = "base_link";
  std::string tip_link_name = "tool0";
  std::vector<std::string> joint_names{ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

  KDLFwdKinChain fwd_kin(*scene_graph, base_link_name, tip_link_name);
  auto inv_kin = std::make_unique<URInvKin>(params, base_link_name, tip_link_name, joint_names);

  EXPECT_EQ(inv_kin->getSolverName(), UR_INV_KIN_CHAIN_SOLVER_NAME);
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
  EXPECT_EQ(inv_kin2->getSolverName(), UR_INV_KIN_CHAIN_SOLVER_NAME);
  EXPECT_EQ(inv_kin2->numJoints(), 6);
  EXPECT_EQ(inv_kin2->getBaseLinkName(), base_link_name);
  EXPECT_EQ(inv_kin2->getWorkingFrame(), base_link_name);
  EXPECT_EQ(inv_kin2->getTipLinkNames().size(), 1);
  EXPECT_EQ(inv_kin2->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(inv_kin2->getJointNames(), joint_names);

  runInvKinTest(*inv_kin2, fwd_kin, pose, tip_link_name, seed);
}

TEST(TesseractKinematicsUnit, UR10InvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.75;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.75;

  double shoulder_offset{ 0.220941 };
  double elbow_offset{ -0.1719 };

  runURKinematicsTests(UR10Parameters, shoulder_offset, elbow_offset, pose);
}

TEST(TesseractKinematicsUnit, UR5InvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.5;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.5;

  double shoulder_offset{ 0.13585 };
  double elbow_offset{ -0.1197 };

  runURKinematicsTests(UR5Parameters, shoulder_offset, elbow_offset, pose);
}

TEST(TesseractKinematicsUnit, UR3InvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.25;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.25;

  double shoulder_offset{ 0.1198 };
  double elbow_offset{ -0.0925 };

  runURKinematicsTests(UR3Parameters, shoulder_offset, elbow_offset, pose);
}

TEST(TesseractKinematicsUnit, UR10eInvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.75;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.75;

  double shoulder_offset{ 0.176 };
  double elbow_offset{ -0.137 };

  runURKinematicsTests(UR10eParameters, shoulder_offset, elbow_offset, pose);
}

TEST(TesseractKinematicsUnit, UR5eInvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.5;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.5;

  double shoulder_offset{ 0.138 };
  double elbow_offset{ -0.131 };

  runURKinematicsTests(UR5eParameters, shoulder_offset, elbow_offset, pose);
}

TEST(TesseractKinematicsUnit, UR3eInvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.25;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.25;

  double shoulder_offset{ 0.120 };
  double elbow_offset{ -0.093 };

  runURKinematicsTests(UR3eParameters, shoulder_offset, elbow_offset, pose);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
