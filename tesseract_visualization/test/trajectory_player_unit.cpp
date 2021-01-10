/**
 * @file tesseract_process_planners_unit.cpp
 * @brief tests for the process planner generators. Several of the
 * generators are instantiated.
 *
 * @author Levi Armstrong
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/trajectory_player.h>
#include <tesseract_visualization/trajectory_interpolator.h>

TEST(TesseracTrajectoryPlayerUnit, TrajectoryTest)  // NOLINT
{
  using namespace tesseract_visualization;
  using namespace tesseract_planning;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  CompositeInstruction program;

  // Define start instruction
  StateWaypoint wp0(joint_names, Eigen::VectorXd::Zero(6));
  MoveInstruction start_instruction(wp0, MoveInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Define move instructions
  for (long i = 1; i < 10; ++i)
  {
    Eigen::VectorXd p = Eigen::VectorXd::Zero(6);
    p(0) = static_cast<double>(i);
    StateWaypoint swp(joint_names, p);
    swp.time = static_cast<double>(i);
    MoveInstruction move_f(swp, MoveInstructionType::FREESPACE, "DEFAULT");
    program.push_back(move_f);
  }

  TrajectoryPlayer player;
  player.setProgram(program);

  EXPECT_NEAR(player.trajectoryDuration(), 9, 1e-5);
  EXPECT_NEAR(player.currentDuration(), 0, 1e-5);

  for (long i = 0; i < 10; ++i)
  {
    MoveInstruction mi = player.setCurrentDurationByIndex(i);
    const auto* swp = mi.getWaypoint().cast_const<StateWaypoint>();
    EXPECT_NEAR(swp->time, static_cast<double>(i), 1e-5);
    EXPECT_NEAR(swp->position(0), static_cast<double>(i), 1e-5);
  }

  for (long i = 0; i < 10; ++i)
  {
    MoveInstruction mi = player.setCurrentDuration(static_cast<double>(i));
    const auto* swp = mi.getWaypoint().cast_const<StateWaypoint>();
    EXPECT_NEAR(swp->time, static_cast<double>(i), 1e-5);
    EXPECT_NEAR(swp->position(0), static_cast<double>(i), 1e-5);
  }

  {
    MoveInstruction mi = player.setCurrentDurationByIndex(10);
    const auto* swp = mi.getWaypoint().cast_const<StateWaypoint>();
    EXPECT_NEAR(swp->time, 9, 1e-5);
    EXPECT_NEAR(swp->position(0), 9, 1e-5);
  }

  {
    MoveInstruction mi = player.setCurrentDuration(10);
    const auto* swp = mi.getWaypoint().cast_const<StateWaypoint>();
    EXPECT_NEAR(swp->time, 9, 1e-5);
    EXPECT_NEAR(swp->position(0), 9, 1e-5);
  }

  {
    MoveInstruction mi = player.setCurrentDurationByIndex(-1);
    const auto* swp = mi.getWaypoint().cast_const<StateWaypoint>();
    EXPECT_NEAR(swp->time, 0, 1e-5);
    EXPECT_NEAR(swp->position(0), 0, 1e-5);
  }

  {
    MoveInstruction mi = player.setCurrentDuration(-1);
    const auto* swp = mi.getWaypoint().cast_const<StateWaypoint>();
    EXPECT_NEAR(swp->time, 0, 1e-5);
    EXPECT_NEAR(swp->position(0), 0, 1e-5);
  }
}

TEST(TesseracTrajectoryInterpolatorUnit, TrajectoryInterpolatorTest)  // NOLINT
{
  using namespace tesseract_visualization;
  using namespace tesseract_planning;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  CompositeInstruction program;

  // Define start instruction
  StateWaypoint wp0(joint_names, Eigen::VectorXd::Zero(6));
  MoveInstruction start_instruction(wp0, MoveInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Define move instructions
  for (long i = 1; i < 10; ++i)
  {
    Eigen::VectorXd p = Eigen::VectorXd::Zero(6);
    p(0) = static_cast<double>(i);
    StateWaypoint swp(joint_names, p);
    swp.time = static_cast<double>(i);
    MoveInstruction move_f(swp, MoveInstructionType::FREESPACE, "DEFAULT");
    program.push_back(move_f);
  }

  TrajectoryInterpolator interpolator(program);

  EXPECT_EQ(interpolator.getMoveInstructionCount(), 10);

  for (long i = 0; i < 19; ++i)
  {
    MoveInstruction mi = interpolator.getMoveInstruction(static_cast<double>(i) * 0.5);
    EXPECT_TRUE(isStateWaypoint(mi.getWaypoint()));
    const auto* swp = mi.getWaypoint().cast_const<StateWaypoint>();
    EXPECT_NEAR(swp->time, static_cast<double>(i) * 0.5, 1e-5);
    EXPECT_NEAR(swp->position(0), static_cast<double>(i) * 0.5, 1e-5);
  }

  // Test above max duration
  MoveInstruction mi = interpolator.getMoveInstruction(10);
  EXPECT_TRUE(isStateWaypoint(mi.getWaypoint()));
  const auto* swp = mi.getWaypoint().cast_const<StateWaypoint>();
  EXPECT_NEAR(swp->time, 9, 1e-5);
  EXPECT_NEAR(swp->position(0), 9, 1e-5);

  // Test get instruction duration
  for (long i = 0; i < 10; ++i)
  {
    double duration = interpolator.getMoveInstructionDuration(i);
    EXPECT_NEAR(duration, static_cast<double>(i), 1e-5);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
