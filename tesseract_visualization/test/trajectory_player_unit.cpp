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

void CheckTrajectory(const tesseract_common::JointTrajectory& trajectory, int first, int last)
{
  using namespace tesseract_visualization;
  using namespace tesseract_common;

  TrajectoryPlayer player;
  player.setTrajectory(trajectory);

  EXPECT_NEAR(player.trajectoryDurationStart(), first, 1e-5);
  EXPECT_NEAR(player.trajectoryDurationEnd(), last, 1e-5);
  EXPECT_NEAR(player.currentDuration(), first, 1e-5);

  // Advance by index
  for (long i = first; i <= last; ++i)
  {
    JointState s = player.setCurrentDurationByIndex(i - first);
    EXPECT_NEAR(s.time, static_cast<double>(i), 1e-5);
    EXPECT_NEAR(s.position(0), static_cast<double>(i), 1e-5);
  }

  // Advance by time
  for (long i = first; i <= last; ++i)
  {
    JointState s = player.setCurrentDuration(static_cast<double>(i));
    EXPECT_NEAR(s.time, static_cast<double>(i), 1e-5);
    EXPECT_NEAR(s.position(0), static_cast<double>(i), 1e-5);
  }

  {
    JointState s = player.setCurrentDurationByIndex((last - first) + 1);
    EXPECT_NEAR(s.time, last, 1e-5);
    EXPECT_NEAR(s.position(0), last, 1e-5);
  }

  {
    JointState s = player.setCurrentDuration(last + 1);
    EXPECT_NEAR(s.time, last, 1e-5);
    EXPECT_NEAR(s.position(0), last, 1e-5);
    EXPECT_TRUE(player.isFinished());
  }

  {
    JointState s = player.setCurrentDuration(last + 2);
    EXPECT_NEAR(s.time, last, 1e-5);
    EXPECT_NEAR(s.position(0), last, 1e-5);
    EXPECT_TRUE(player.isFinished());
    player.setCurrentDuration(first);
    EXPECT_FALSE(player.isFinished());
  }

  {
    JointState s = player.setCurrentDurationByIndex(-1);
    EXPECT_NEAR(s.time, first, 1e-5);
    EXPECT_NEAR(s.position(0), first, 1e-5);
  }

  {
    JointState s = player.setCurrentDuration(first - 1);
    EXPECT_NEAR(s.time, first, 1e-5);
    EXPECT_NEAR(s.position(0), first, 1e-5);
    EXPECT_FALSE(player.isFinished());
  }
}

TEST(TesseracTrajectoryPlayerUnit, TrajectoryUntimedTest)  // NOLINT
{
  using namespace tesseract_visualization;
  using namespace tesseract_common;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  JointTrajectory trajectory;
  int first = 0;
  int last = 9;
  double auto_dt = 0.1;  // Time step assigned by interpolator to untimed trajectory

  // Define trajectory without timing
  for (long i = first; i <= last; ++i)
  {
    Eigen::VectorXd p = Eigen::VectorXd::Zero(6);
    p(0) = static_cast<double>(i);
    trajectory.push_back(JointState(joint_names, p));
  }

  TrajectoryPlayer player;
  player.setTrajectory(trajectory);

  EXPECT_NEAR(player.trajectoryDurationStart(), first, 1e-5);
  EXPECT_NEAR(player.trajectoryDurationEnd(), last * auto_dt, 1e-5);
  EXPECT_NEAR(player.currentDuration(), first, 1e-5);

  {
    JointState s = player.setCurrentDuration(0.5);
    EXPECT_NEAR(s.position(0), 0.5 / auto_dt, 1e-5);
  }

  {
    JointState s = player.setCurrentDurationByIndex(6);
    EXPECT_NEAR(s.time, 6 * auto_dt, 1e-5);
  }
}

TEST(TesseracTrajectoryPlayerUnit, TrajectoryTimedTest)  // NOLINT
{
  using namespace tesseract_visualization;
  using namespace tesseract_common;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  JointTrajectory trajectory;
  int first = 0;
  int last = 9;

  // Define trajectory with timing
  trajectory.clear();
  for (long i = first; i <= last; ++i)
  {
    Eigen::VectorXd p = Eigen::VectorXd::Zero(6);
    p(0) = static_cast<double>(i);
    trajectory.push_back(JointState(joint_names, p));
    trajectory.back().time = static_cast<double>(i);
  }

  CheckTrajectory(trajectory, first, last);
}

TEST(TesseracTrajectoryPlayerUnit, TrajectoryNonzeroStartTest)  // NOLINT
{
  using namespace tesseract_visualization;
  using namespace tesseract_common;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  JointTrajectory trajectory;
  int first = 5;
  int last = 14;

  // Define trajectory with timing and a non-zero begin
  for (long i = first; i <= last; ++i)
  {
    Eigen::VectorXd p = Eigen::VectorXd::Zero(6);
    p(0) = static_cast<double>(i);
    trajectory.push_back(JointState(joint_names, p));
    trajectory.back().time = static_cast<double>(i);
  }

  CheckTrajectory(trajectory, first, last);
}

TEST(TesseracTrajectoryInterpolatorUnit, TrajectoryInterpolatorTest)  // NOLINT
{
  using namespace tesseract_visualization;
  using namespace tesseract_common;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  JointTrajectory trajectory;
  double time_scale = 0.5;

  // Define trajectory
  for (long i = 0; i < 10; ++i)
  {
    Eigen::VectorXd p = Eigen::VectorXd::Zero(6);
    p(0) = static_cast<double>(i);
    trajectory.push_back(JointState(joint_names, p));
    trajectory.back().time = static_cast<double>(i) * time_scale;
  }

  TrajectoryInterpolator interpolator(trajectory);

  EXPECT_EQ(interpolator.getStateCount(), 10);

  for (long i = 0; i < 19; ++i)
  {
    JointState s = interpolator.getState(static_cast<double>(i) * 0.5 * time_scale);
    EXPECT_NEAR(s.time, static_cast<double>(i) * 0.5 * time_scale, 1e-5);
    EXPECT_NEAR(s.position(0), static_cast<double>(i) * 0.5, 1e-5);
  }

  // Test above max duration
  JointState s = interpolator.getState(10);
  EXPECT_NEAR(s.time, 9 * time_scale, 1e-5);
  EXPECT_NEAR(s.position(0), 9, 1e-5);

  // Test get instruction duration
  for (long i = 0; i < 10; ++i)
  {
    double duration = interpolator.getStateDuration(i);
    EXPECT_NEAR(duration, static_cast<double>(i) * time_scale, 1e-5);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
