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
void TrajectoryPlayer::setProgram(tesseract_planning::CompositeInstruction program)
{
  // Prepare the new trajectory message
  trajectory_ = std::make_unique<TrajectoryInterpolator>(program);

  // Get the duration
  trajectory_duration_ = trajectory_->getMoveInstructionDuration(trajectory_->getMoveInstructionCount() - 1);

  // Reset state associated with trajectory playback
  current_duration_ = 0.0;

  // Get the chrono time
  start_time_ = std::chrono::high_resolution_clock::now();
}

void TrajectoryPlayer::setScale(double scale) { scale_ = scale; }

tesseract_planning::MoveInstruction TrajectoryPlayer::setCurrentDurationByIndex(long index)
{
  using tesseract_planning::MoveInstruction;
  using tesseract_planning::MoveInstructionType;
  using tesseract_planning::NullWaypoint;

  if (!trajectory_ || trajectory_->empty())
    throw std::runtime_error("Trajectory is empty!");

  if (trajectory_->getMoveInstructionCount() > 0)
  {
    if (index > 0)
      current_duration_ = trajectory_->getMoveInstructionDuration(index);
    else
      current_duration_ = 0;
  }

  start_time_ = std::chrono::high_resolution_clock::now() -
                std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                    std::chrono::duration<double>(current_duration_));

  // Compute the interpolated state
  return trajectory_->getMoveInstruction(current_duration_);
}

tesseract_planning::MoveInstruction TrajectoryPlayer::setCurrentDuration(double duration)
{
  using tesseract_planning::MoveInstruction;
  using tesseract_planning::MoveInstructionType;
  using tesseract_planning::NullWaypoint;

  if (!trajectory_ || trajectory_->empty())
    throw std::runtime_error("Trajectory is empty!");

  if (duration > trajectory_duration_)
    current_duration_ = trajectory_duration_;
  else if (duration < 0)
    current_duration_ = 0;
  else
    current_duration_ = duration;

  start_time_ = std::chrono::high_resolution_clock::now() -
                std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                    std::chrono::duration<double>(current_duration_));

  // Compute the interpolated state
  return trajectory_->getMoveInstruction(current_duration_);
}

tesseract_planning::MoveInstruction TrajectoryPlayer::getNext()
{
  using tesseract_planning::MoveInstruction;
  using tesseract_planning::MoveInstructionType;
  using tesseract_planning::NullWaypoint;

  if (!trajectory_ || trajectory_->empty())
    throw std::runtime_error("Trajectory is empty!");

  auto current_time = std::chrono::high_resolution_clock::now();
  current_duration_ = (scale_ * std::chrono::duration<double>(current_time - start_time_).count());

  if (current_duration_ > trajectory_duration_)
  {
    current_duration_ = trajectory_duration_;

    // Compute the interpolated state
    auto mi = trajectory_->getMoveInstruction(current_duration_);

    // Reset the player
    if (loop_)
      reset();
    else
      finished_ = true;

    return mi;
  }

  return trajectory_->getMoveInstruction(current_duration_);
}

tesseract_planning::MoveInstruction TrajectoryPlayer::getByIndex(long index) const
{
  return trajectory_->getMoveInstruction(trajectory_->getMoveInstructionDuration(index));
}

double TrajectoryPlayer::currentDuration() const { return current_duration_; }

double TrajectoryPlayer::trajectoryDuration() const { return trajectory_duration_; }

bool TrajectoryPlayer::isFinished() const { return finished_; }

void TrajectoryPlayer::enableLoop(bool loop) { loop_ = loop; }

bool TrajectoryPlayer::isLoopEnabled() const { return loop_; }

void TrajectoryPlayer::reset()
{
  current_duration_ = 0;
  start_time_ = std::chrono::high_resolution_clock::now();
  finished_ = false;
}

long TrajectoryPlayer::size() const { return trajectory_->getMoveInstructionCount(); }

}  // namespace tesseract_visualization
