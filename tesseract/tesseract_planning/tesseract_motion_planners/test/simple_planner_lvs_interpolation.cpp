/**
 * @file simple_planner_fixed_size_interpolation.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date July 23, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <boost/filesystem.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/step_generators/lvs_interpolation.h>

using namespace tesseract;
using namespace tesseract_planning;

bool DEBUG = false;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

class TesseractPlanningSimplePlannerLVSInterpolationUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  ManipulatorInfo manip_info_;
  std::vector<std::string> joint_names_;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;

    manip_info_.manipulator = "manipulator";
    joint_names_ = tesseract_ptr_->getEnvironment()
                       ->getManipulatorManager()
                       ->getFwdKinematicSolver("manipulator")
                       ->getJointNames();
  }
};

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointJoint_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  auto composite = LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.cast_const<MoveInstruction>()->getWaypoint()));
    EXPECT_EQ(c.cast_const<MoveInstruction>()->getProfile(), instr.getProfile());
  }
  const auto* mi = composite.back().cast_const<MoveInstruction>();
  EXPECT_TRUE(wp2.isApprox(mi->getWaypoint().cast_const<StateWaypoint>()->position, 1e-5));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  auto composite_short =
      LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 0.5, 1.57, min_steps);
  EXPECT_EQ(composite_short.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.05;
  auto composite_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), longest_valid_segment_length, 10, 6.28, min_steps);
  double dist = (wp1 - wp2).norm();
  int steps = int(dist / longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(composite_long.size()) > min_steps);
  EXPECT_EQ(composite_long.size(), steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointJoint_Linear)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  auto fwd_kin =
      tesseract_ptr_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr(wp1, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  auto composite = LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.cast_const<MoveInstruction>()->getWaypoint()));
    EXPECT_EQ(c.cast_const<MoveInstruction>()->getProfile(), instr.getProfile());
  }
  const auto* mi = composite.back().cast_const<MoveInstruction>();
  EXPECT_TRUE(wp2.isApprox(mi->getWaypoint().cast_const<StateWaypoint>()->position, 1e-5));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  auto composite_short =
      LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 10, 6.28, min_steps);
  EXPECT_EQ(composite_short.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used when large motion given
  double translation_longest_valid_segment_length = 0.01;
  auto composite_trans_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), 6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  Eigen::Isometry3d p1, p2;
  fwd_kin->calcFwdKin(p1, wp1);
  fwd_kin->calcFwdKin(p2, wp2);
  double trans_dist = (p2.translation() - p1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(composite_trans_long.size()) > min_steps);
  EXPECT_EQ(composite_trans_long.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  auto composite_rot_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 10, rotation_longest_valid_segment_length, min_steps);
  double rot_dist = Eigen::Quaterniond(p1.linear()).angularDistance(Eigen::Quaterniond(p2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(composite_rot_long.size()) > min_steps);
  EXPECT_EQ(composite_rot_long.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointCart_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  auto fwd_kin =
      tesseract_ptr_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2;
  fwd_kin->calcFwdKin(wp2, Eigen::VectorXd::Ones(7));
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  auto composite = LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.cast_const<MoveInstruction>()->getWaypoint()));
    EXPECT_EQ(c.cast_const<MoveInstruction>()->getProfile(), instr.getProfile());
  }
  const auto* mi = composite.back().cast_const<MoveInstruction>();
  const Eigen::VectorXd& last_position = mi->getWaypoint().cast_const<StateWaypoint>()->position;
  Eigen::Isometry3d final_pose;
  fwd_kin->calcFwdKin(final_pose, last_position);
  EXPECT_TRUE(wp2.isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  auto composite_short =
      LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 0.5, 1.57, min_steps);
  EXPECT_EQ(composite_short.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  auto composite_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), longest_valid_segment_length, 10, 6.28, min_steps);
  EXPECT_TRUE(static_cast<int>(composite_long.size()) > min_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointCart_Linear)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  auto fwd_kin =
      request.tesseract->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2;
  fwd_kin->calcFwdKin(wp2, Eigen::VectorXd::Ones(7));

  PlanInstruction instr(wp1, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  auto composite = LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.cast_const<MoveInstruction>()->getWaypoint()));
    EXPECT_EQ(c.cast_const<MoveInstruction>()->getProfile(), instr.getProfile());
  }
  const auto* mi = composite.back().cast_const<MoveInstruction>();
  const Eigen::VectorXd& last_position = mi->getWaypoint().cast_const<StateWaypoint>()->position;
  Eigen::Isometry3d final_pose;
  fwd_kin->calcFwdKin(final_pose, last_position);
  EXPECT_TRUE(wp2.isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  auto composite_short =
      LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 10, 6.28, min_steps);
  EXPECT_EQ(composite_short.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  auto composite_trans_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), 6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  Eigen::Isometry3d p1;
  fwd_kin->calcFwdKin(p1, wp1);
  double trans_dist = (wp2.translation() - p1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(composite_trans_long.size()) > min_steps);
  EXPECT_EQ(composite_trans_long.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  auto composite_rot_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 10, rotation_longest_valid_segment_length, min_steps);
  double rot_dist = Eigen::Quaterniond(p1.linear()).angularDistance(Eigen::Quaterniond(wp2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(composite_rot_long.size()) > min_steps);
  EXPECT_EQ(composite_rot_long.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartJoint_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  auto fwd_kin =
      tesseract_ptr_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  CartesianWaypoint wp1;
  fwd_kin->calcFwdKin(wp1, Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  auto composite = LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.cast_const<MoveInstruction>()->getWaypoint()));
    EXPECT_EQ(c.cast_const<MoveInstruction>()->getProfile(), instr.getProfile());
  }
  const auto* mi = composite.back().cast_const<MoveInstruction>();
  EXPECT_TRUE(wp2.isApprox(mi->getWaypoint().cast_const<StateWaypoint>()->position, 1e-5));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  auto composite_short =
      LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 10, 6.28, min_steps);
  EXPECT_EQ(composite_short.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  auto composite_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), longest_valid_segment_length, 10, 6.28, min_steps);
  EXPECT_TRUE(static_cast<int>(composite_long.size()) > min_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartJoint_Linear)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  auto fwd_kin =
      tesseract_ptr_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  CartesianWaypoint wp1;
  fwd_kin->calcFwdKin(wp1, Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr(wp1, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  auto composite = LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.cast_const<MoveInstruction>()->getWaypoint()));
    EXPECT_EQ(c.cast_const<MoveInstruction>()->getProfile(), instr.getProfile());
  }
  const auto* mi = composite.back().cast_const<MoveInstruction>();
  EXPECT_TRUE(wp2.isApprox(mi->getWaypoint().cast_const<StateWaypoint>()->position, 1e-5));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  auto composite_short =
      LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 10, 6.28, min_steps);
  EXPECT_EQ(composite_short.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  auto composite_trans_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), 6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  Eigen::Isometry3d p2;
  fwd_kin->calcFwdKin(p2, wp2);
  double trans_dist = (p2.translation() - wp1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(composite_trans_long.size()) > min_steps);
  EXPECT_EQ(composite_trans_long.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  auto composite_rot_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 10, rotation_longest_valid_segment_length, min_steps);
  double rot_dist = Eigen::Quaterniond(wp1.linear()).angularDistance(Eigen::Quaterniond(p2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(composite_rot_long.size()) > min_steps);
  EXPECT_EQ(composite_rot_long.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartCart_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  auto fwd_kin =
      tesseract_ptr_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  CartesianWaypoint wp1;
  fwd_kin->calcFwdKin(wp1, Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2;
  fwd_kin->calcFwdKin(wp2, Eigen::VectorXd::Ones(7));
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  auto composite = LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.cast_const<MoveInstruction>()->getWaypoint()));
    EXPECT_EQ(c.cast_const<MoveInstruction>()->getProfile(), instr.getProfile());
  }
  const auto* mi = composite.back().cast_const<MoveInstruction>();
  const Eigen::VectorXd& last_position = mi->getWaypoint().cast_const<StateWaypoint>()->position;
  Eigen::Isometry3d final_pose = Eigen::Isometry3d::Identity();
  fwd_kin->calcFwdKin(final_pose, last_position);
  EXPECT_TRUE(wp2.isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  auto composite_short =
      LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 0.5, 1.57, min_steps);
  EXPECT_EQ(composite_short.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  auto composite_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), longest_valid_segment_length, 10, 6.28, min_steps);
  EXPECT_TRUE(static_cast<int>(composite_long.size()) > min_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartCart_Linear)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  auto fwd_kin =
      tesseract_ptr_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  CartesianWaypoint wp1;
  fwd_kin->calcFwdKin(wp1, Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2;
  fwd_kin->calcFwdKin(wp2, Eigen::VectorXd::Ones(7));
  PlanInstruction instr(wp1, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  auto composite = LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.cast_const<MoveInstruction>()->getWaypoint()));
    EXPECT_EQ(c.cast_const<MoveInstruction>()->getProfile(), instr.getProfile());
  }
  const auto* mi = composite.back().cast_const<MoveInstruction>();
  const Eigen::VectorXd& last_position = mi->getWaypoint().cast_const<StateWaypoint>()->position;
  Eigen::Isometry3d final_pose = Eigen::Isometry3d::Identity();
  fwd_kin->calcFwdKin(final_pose, last_position);
  EXPECT_TRUE(wp2.isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  auto composite_short =
      LVSInterpolateStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 10, 6.28, min_steps);
  EXPECT_EQ(composite_short.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  auto composite_trans_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), 6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  double trans_dist = (wp2.translation() - wp1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(composite_trans_long.size()) > min_steps);
  EXPECT_EQ(composite_trans_long.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  auto composite_rot_long = LVSInterpolateStateWaypoint(
      wp1, wp2, instr, request, ManipulatorInfo(), 6.28, 10, rotation_longest_valid_segment_length, min_steps);
  double rot_dist = Eigen::Quaterniond(wp1.linear()).angularDistance(Eigen::Quaterniond(wp2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(composite_rot_long.size()) > min_steps);
  EXPECT_EQ(composite_rot_long.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateCartStateWaypoint_JointJoint_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  EXPECT_ANY_THROW(LVSInterpolateCartStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5));
  /// @todo: Update once implemented
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateCartStateWaypoint_JointJoint_Linear)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr(wp1, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  EXPECT_ANY_THROW(LVSInterpolateCartStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5));
  /// @todo: Update once implemented
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateCartStateWaypoint_JointCart_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  EXPECT_ANY_THROW(LVSInterpolateCartStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5));
  /// @todo: Update once implemented
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateCartStateWaypoint_JointCart_Linear)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr(wp1, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  EXPECT_ANY_THROW(LVSInterpolateCartStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5));
  /// @todo: Update once implemented
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateCartStateWaypoint_CartJoint_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Zero(7));
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  EXPECT_ANY_THROW(LVSInterpolateCartStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5));
  /// @todo: Update once implemented
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateCartStateWaypoint_CartJoint_Linear)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Zero(7));
  PlanInstruction instr(wp1, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  EXPECT_ANY_THROW(LVSInterpolateCartStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5));
  /// @todo: Update once implemented
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateCartStateWaypoint_CartCart_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  EXPECT_ANY_THROW(LVSInterpolateCartStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5));
  /// @todo: Update once implemented
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateCartStateWaypoint_CartCart_Linear)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr(wp1, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  EXPECT_ANY_THROW(LVSInterpolateCartStateWaypoint(wp1, wp2, instr, request, ManipulatorInfo(), 3.14, 0.5, 1.57, 5));
  /// @todo: Update once implemented
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
