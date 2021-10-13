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

void runURKinematicsTests(const tesseract_kinematics::URParameters& params, const Eigen::Isometry3d& pose)
{
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(6);

  // Setup test
  auto scene_graph = getSceneGraphUR(params);

  std::string manip_name = "manip";
  std::string base_link_name = "base_link";
  std::string tip_link_name = "tool0";
  std::vector<std::string> joint_names{ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

  tesseract_kinematics::KDLFwdKinChain fwd_kin;
  fwd_kin.init(*scene_graph, base_link_name, tip_link_name, manip_name);

  auto inv_kin = std::make_unique<tesseract_kinematics::URInvKin>();
  EXPECT_FALSE(inv_kin->checkInitialized());
  bool status = inv_kin->init(manip_name, params, base_link_name, tip_link_name, joint_names);

  EXPECT_TRUE(status);
  EXPECT_TRUE(inv_kin->checkInitialized());
  EXPECT_EQ(inv_kin->getName(), manip_name);
  EXPECT_EQ(inv_kin->getSolverName(), "URInvKin");
  EXPECT_EQ(inv_kin->numJoints(), 6);
  EXPECT_EQ(inv_kin->getBaseLinkName(), base_link_name);
  EXPECT_EQ(inv_kin->getWorkingFrames().size(), 1);
  EXPECT_EQ(inv_kin->getWorkingFrames()[0], base_link_name);
  EXPECT_EQ(inv_kin->getTipLinkNames().size(), 1);
  EXPECT_EQ(inv_kin->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(inv_kin->getJointNames(), joint_names);

  runInvKinTest(*inv_kin, fwd_kin, pose, base_link_name, tip_link_name, seed);

  // Check cloned
  tesseract_kinematics::InverseKinematics::Ptr inv_kin2 = inv_kin->clone();
  EXPECT_TRUE(inv_kin2 != nullptr);
  EXPECT_EQ(inv_kin2->getName(), manip_name);
  EXPECT_EQ(inv_kin2->getSolverName(), "URInvKin");
  EXPECT_EQ(inv_kin2->numJoints(), 6);
  EXPECT_EQ(inv_kin2->getBaseLinkName(), base_link_name);
  EXPECT_EQ(inv_kin2->getWorkingFrames().size(), 1);
  EXPECT_EQ(inv_kin2->getWorkingFrames()[0], base_link_name);
  EXPECT_EQ(inv_kin2->getTipLinkNames().size(), 1);
  EXPECT_EQ(inv_kin2->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(inv_kin2->getJointNames(), joint_names);

  runInvKinTest(*inv_kin2, fwd_kin, pose, base_link_name, tip_link_name, seed);
}

TEST(TesseractKinematicsUnit, UR10InvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.75;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.75;

  runURKinematicsTests(tesseract_kinematics::UR10Parameters, pose);
}

TEST(TesseractKinematicsUnit, UR5InvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.5;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.5;

  runURKinematicsTests(tesseract_kinematics::UR5Parameters, pose);
}

TEST(TesseractKinematicsUnit, UR3InvKinUnit)  // NOLINT
{
  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0.25;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.25;

  runURKinematicsTests(tesseract_kinematics::UR3Parameters, pose);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
