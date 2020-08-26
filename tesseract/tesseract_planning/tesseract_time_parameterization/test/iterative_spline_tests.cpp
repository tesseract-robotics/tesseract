/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Ken Anderson
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ken Anderson */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/command_language.h>
#include <tesseract_time_parameterization/iterative_spline_parameterization.h>

using namespace tesseract_planning;

// Initialize one-joint, 3 points exactly the same.
CompositeInstruction createRepeatedPointTrajectory()
{
  const int num = 3;
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  CompositeInstruction program;
  for (int i = 0; i < num; i++)
  {
    StateWaypoint swp(joint_names, Eigen::VectorXd::Zero(6));
    swp.position[0] = 1;
    if (i == 0)
      program.setStartInstruction(MoveInstruction(swp, MoveInstructionType::START));
    else
      program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  return program;
}

// Initialize one-joint, straight-line trajectory
CompositeInstruction createStraightTrajectory()
{
  const int num = 10;
  const double max = 2.0;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  CompositeInstruction program;
  for (int i = 0; i < num; i++)
  {
    StateWaypoint swp(joint_names, Eigen::VectorXd::Zero(6));
    swp.position[0] = i * max / num;
    if (i == 0)
      program.setStartInstruction(MoveInstruction(swp, MoveInstructionType::START));
    else
      program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  // leave final velocity/acceleration unset
  StateWaypoint swp(joint_names, Eigen::VectorXd::Zero(6));
  swp.position[0] = max;
  program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));

  return program;
}

TEST(IterativeSplineParameterizationUnit, Solve)  // NOLINT
{
  EXPECT_TRUE(true);
}

TEST(TestTimeParameterization, TestIterativeSpline)
{
  IterativeSplineParameterization time_parameterization(false);
  CompositeInstruction program = createStraightTrajectory();
  std::vector<double> max_velocity = { 2.088, 2.082, 3.27, 3.6, 3.3, 3.078 };
  std::vector<double> max_acceleration = { 1, 1, 1, 1, 1, 1 };
  EXPECT_TRUE(time_parameterization.compute(program, max_velocity, max_acceleration));
  ASSERT_LT(program.back().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>()->time, 5.0);
}

TEST(TestTimeParameterization, TestIterativeSplineAddPoints)
{
  IterativeSplineParameterization time_parameterization(true);
  CompositeInstruction program = createStraightTrajectory();
  std::vector<double> max_velocity = { 2.088, 2.082, 3.27, 3.6, 3.3, 3.078 };
  std::vector<double> max_acceleration = { 1, 1, 1, 1, 1, 1 };
  EXPECT_TRUE(time_parameterization.compute(program, max_velocity, max_acceleration));
  ASSERT_LT(program.back().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>()->time, 5.0);
}

TEST(TestTimeParameterization, TestIterativeSplineDynamicParams)
{
  IterativeSplineParameterization time_parameterization(false);
  CompositeInstruction program = createStraightTrajectory();
  Eigen::VectorXd max_velocity(6);
  max_velocity << 2.088, 2.082, 3.27, 3.6, 3.3, 3.078;
  Eigen::VectorXd max_acceleration(6);
  max_acceleration << 1, 1, 1, 1, 1, 1;
  Eigen::VectorXd max_velocity_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(program.size() + 1));
  Eigen::VectorXd max_acceleration_scaling_factors =
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(program.size() + 1));  // +1 for start instruction

  EXPECT_TRUE(time_parameterization.compute(
      program, max_velocity, max_acceleration, max_velocity_scaling_factors, max_acceleration_scaling_factors));
  EXPECT_LT(program.back().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>()->time, 5.0);

  program = createStraightTrajectory();
  max_velocity_scaling_factors[0] = 0.5;
  max_acceleration_scaling_factors[0] = 0.5;
  EXPECT_TRUE(time_parameterization.compute(
      program, max_velocity, max_acceleration, max_velocity_scaling_factors, max_acceleration_scaling_factors));
}

TEST(TestTimeParameterization, TestRepeatedPoint)
{
  IterativeSplineParameterization time_parameterization(true);
  CompositeInstruction program = createRepeatedPointTrajectory();
  std::vector<double> max_velocity = { 2.088, 2.082, 3.27, 3.6, 3.3, 3.078 };
  std::vector<double> max_acceleration = { 1, 1, 1, 1, 1, 1 };
  EXPECT_TRUE(time_parameterization.compute(program, max_velocity, max_acceleration));
  ASSERT_LT(program.back().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>()->time, 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  return RUN_ALL_TESTS();
}
