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
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>

namespace tesseract_visualization
{
class TrajectoryPlayer
{
public:
  TrajectoryPlayer() = default;

  void setProgram(tesseract_planning::CompositeInstruction program);

  void setScale(double scale);

  tesseract_planning::MoveInstruction setCurrentTime(int index);

  tesseract_planning::MoveInstruction setCurrentDuration(double duration);

  tesseract_planning::MoveInstruction getNext();

  double currentDuration() const;

  bool isFinished() const;

  void setLoop(bool loop);

  bool getLoop() const;

  void reset();

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
