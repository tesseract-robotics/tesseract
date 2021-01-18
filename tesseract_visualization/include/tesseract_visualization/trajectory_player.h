/**
 * @file trajectory_player.h
 * @brief Trajectory player class
 *
 * @author Levi Armstrong
 * @date August 20, 2020
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
#ifndef TESSERACT_VISUALIZATION_TRAJECTORY_PLAYER_H
#define TESSERACT_VISUALIZATION_TRAJECTORY_PLAYER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <chrono>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/trajectory_interpolator.h>

namespace tesseract_visualization
{
/** @brief Enables the ability to play a trajectory provided by the set program */
class TrajectoryPlayer
{
public:
  TrajectoryPlayer() = default;

  /**
   * @brief Set the the trajectory for the trajectory player
   * @param program The trajectory to play
   */
  void setTrajectory(tesseract_common::JointTrajectory trajectory);

  /**
   * @brief Set the scale factor for the play back of the trajectory
   * @param scale The scale playback of the trajectory
   */
  void setScale(double scale);

  /**
   * @brief Set the current time for the player by index of the input trajectoy
   * @param index The input trajectory index for which to set the current time from
   * @return The trajectory state at the input trajectory index
   */
  tesseract_common::JointState setCurrentDurationByIndex(long index);

  /**
   * @brief Set the current time for the player by duration
   * @param duration The duration for which to set the current time from
   * @return The trajectory state at the provided duration
   */
  tesseract_common::JointState setCurrentDuration(double duration);

  /**
   * @brief Get the next move instruction from the player
   * @return The move instruction at the next time interval
   */
  tesseract_common::JointState getNext();

  /**
   * @brief Get move instruction by index
   * @param index The index of the input program to extract the move instruction from
   * @return The move instruction at the input index
   */
  tesseract_common::JointState getByIndex(long index) const;

  /**
   * @brief Get the current duration populated by the last call to getNext()
   * @return The current duration
   */
  double currentDuration() const;

  /**
   * @brief Get the trajectory duration
   * @return The trajectory duration
   */
  double trajectoryDuration() const;

  /**
   * @brief Check if the player has the reached the end of the trajectory
   * @return True if end has been reached, otherwise false.
   */
  bool isFinished() const;

  /**
   * @brief Enable looping playback of the trajectory
   * @param loop True to enable looping play, otherwise single playback.
   */
  void enableLoop(bool loop);

  /**
   * @brief Get if looping playback is enabled
   * @return True if looping playback is enabled otherwise false.
   */
  bool isLoopEnabled() const;

  /** @brief Reset the state of the trajectory player */
  void reset();

  /** @brief The size of the tajectory */
  long size() const;

private:
  TrajectoryInterpolator::UPtr trajectory_{ nullptr };
  double trajectory_duration_{ 0 };
  double current_duration_{ 0 };
  double scale_{ 1 };
  bool loop_{ false };
  bool finished_{ false };

  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

}  // namespace tesseract_visualization

#endif  // TESSERACT_VISUALIZATION_TRAJECTORY_PLAYER_H
