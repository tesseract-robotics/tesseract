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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/trajectory_player.h>

namespace tesseract_visualization
{
void TrajectoryPlayer::setTrajectory(tesseract_common::JointTrajectory trajectory)
{
  // Prepare the new trajectory message
  trajectory_ = std::make_unique<TrajectoryInterpolator>(trajectory);

  // Get the duration
  trajectory_duration_ = trajectory_->getStateDuration(trajectory_->getStateCount() - 1);

  // Reset state
  reset();
}

void TrajectoryPlayer::setScale(double scale) { scale_ = scale; }

tesseract_common::JointState TrajectoryPlayer::setCurrentDurationByIndex(long index)
{
  if (!trajectory_ || trajectory_->empty())
    throw std::runtime_error("Trajectory is empty!");

  if (trajectory_->getStateCount() > 0)
  {
    if (index > 0)
      current_duration_ = trajectory_->getStateDuration(index);
    else
      current_duration_ = 0;
  }

  start_time_ = std::chrono::high_resolution_clock::now() -
                std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                    std::chrono::duration<double>(current_duration_));

  // Compute the interpolated state
  return trajectory_->getState(current_duration_);
}

tesseract_common::JointState TrajectoryPlayer::setCurrentDuration(double duration)
{
  if (!trajectory_ || trajectory_->empty())
    throw std::runtime_error("Trajectory is empty!");

  finished_ = false;
  if (duration > trajectory_duration_)
  {
    current_duration_ = trajectory_duration_;
    finished_ = true;
  }
  else if (duration < 0)
    current_duration_ = 0;
  else
    current_duration_ = duration;

  start_time_ = std::chrono::high_resolution_clock::now() -
                std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                    std::chrono::duration<double>(current_duration_));

  // Compute the interpolated state
  return trajectory_->getState(current_duration_);
}

tesseract_common::JointState TrajectoryPlayer::getNext()
{
  if (!trajectory_ || trajectory_->empty())
    throw std::runtime_error("Trajectory is empty!");

  auto current_time = std::chrono::high_resolution_clock::now();
  current_duration_ = (scale_ * std::chrono::duration<double>(current_time - start_time_).count());

  if (current_duration_ > trajectory_duration_)
  {
    current_duration_ = trajectory_duration_;

    // Compute the interpolated state
    auto mi = trajectory_->getState(current_duration_);

    // Reset the player
    if (loop_)
      reset();
    else
      finished_ = true;

    return mi;
  }

  return trajectory_->getState(current_duration_);
}

tesseract_common::JointState TrajectoryPlayer::getByIndex(long index) const
{
  return trajectory_->getState(trajectory_->getStateDuration(index));
}

double TrajectoryPlayer::currentDuration() const { return current_duration_; }

double TrajectoryPlayer::trajectoryDuration() const { return trajectory_duration_; }

bool TrajectoryPlayer::isFinished() const { return finished_; }

void TrajectoryPlayer::enableLoop(bool loop) { loop_ = loop; }

bool TrajectoryPlayer::isLoopEnabled() const { return loop_; }

void TrajectoryPlayer::reset()
{
  // Reset state associated with trajectory playback
  current_duration_ = 0.0;

  // Get the chrono time
  start_time_ = std::chrono::high_resolution_clock::now();

  // Reset finished
  finished_ = false;
}

long TrajectoryPlayer::size() const { return (trajectory_) ? trajectory_->getStateCount() : 0; }

}  // namespace tesseract_visualization
