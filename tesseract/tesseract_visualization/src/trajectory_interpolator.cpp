/**
 * @file trajectory_interpolator.cpp
 * @brief Trajectory interpolator class
 *
 * @author Levi Armstrong
 * @date August 20, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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

/* Based on MoveIt code authored by: Ioan Sucan, Adam Leeper */

#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_visualization/trajectory_interpolator.h>

namespace tesseract_visualization
{
static tesseract_planning::locateFilterFn moveFilter = [](const tesseract_planning::Instruction& i,
                                                          const tesseract_planning::CompositeInstruction& /*composite*/,
                                                          bool parent_is_first_composite) {
  if (tesseract_planning::isMoveInstruction(i))
  {
    if (i.cast_const<tesseract_planning::MoveInstruction>()->isStart())
      return (parent_is_first_composite);

    return true;
  }
  return false;
};

TrajectoryInterpolator::TrajectoryInterpolator(tesseract_planning::CompositeInstruction program) : program_(program)
{
  flattened_program_ = tesseract_planning::flatten(program_, moveFilter);
  for (auto& mi : flattened_program_)
    waypoints_.emplace_back(mi.get().cast<tesseract_planning::MoveInstruction>()->getWaypoint());

  double last_time = 0;
  double current_time = 0;
  double total_time = 0;
  for (auto& waypoint : waypoints_)
  {
    auto* swp = waypoint.get().cast<tesseract_planning::StateWaypoint>();
    current_time = swp->time;

    // It is possible for sub composites to start back from zero, this accounts for it
    if (current_time < last_time)
      last_time = current_time;

    double dt = current_time - last_time;
    total_time += dt;
    duration_from_previous_.push_back(dt);
    swp->time = total_time;
    last_time = current_time;
  }
}

void TrajectoryInterpolator::findMoveInstructionIndices(const double& duration,
                                                        int& before,
                                                        int& after,
                                                        double& blend) const
{
  if (duration < 0.0)
  {
    before = 0;
    after = 0;
    blend = 0;
    return;
  }

  // Find indicies
  std::size_t index = 0;
  std::size_t num_points = waypoints_.size();
  double running_duration = 0.0;
  for (; index < num_points; ++index)
  {
    running_duration += duration_from_previous_[index];
    if (running_duration >= duration)
      break;
  }
  before = static_cast<int>(std::max<std::size_t>(index - 1, 0));
  after = static_cast<int>(std::min<std::size_t>(index, num_points - 1));

  // Compute duration blend
  double before_time = running_duration - duration_from_previous_[index];
  if (after == before)
    blend = 1.0;
  else
    blend = (duration - before_time) / duration_from_previous_[index];
}

tesseract_planning::MoveInstruction TrajectoryInterpolator::getMoveInstruction(double request_duration) const
{
  // If there are no waypoints we can't do anything
  if (waypoints_.empty())
    throw std::runtime_error("Invalid duration");

  int before = 0;
  int after = 0;
  double blend = 1.0;
  findMoveInstructionIndices(request_duration, before, after, blend);

  if (before < 0 && after < 0)
    throw std::runtime_error("Invalid duration");

  if (before < 0 && after == 0)
    return *(
        flattened_program_[static_cast<std::size_t>(after)].get().cast_const<tesseract_planning::MoveInstruction>());

  if (before == static_cast<int>(waypoints_.size()) - 1)
    return *(
        flattened_program_[static_cast<std::size_t>(before)].get().cast_const<tesseract_planning::MoveInstruction>());

  if (before >= 0 && after > 0)
  {
    const auto* swp0 =
        waypoints_[static_cast<std::size_t>(before)].get().cast_const<tesseract_planning::StateWaypoint>();
    const auto* swp1 =
        waypoints_[static_cast<std::size_t>(after)].get().cast_const<tesseract_planning::StateWaypoint>();

    tesseract_planning::MoveInstruction output_instruction =
        *(flattened_program_[static_cast<std::size_t>(after)].get().cast_const<tesseract_planning::MoveInstruction>());
    output_instruction.setWaypoint(interpolate(*swp0, *swp1, blend));
    return output_instruction;
  }

  throw std::runtime_error("Invalid duration");
}

double TrajectoryInterpolator::getMoveInstructionDuration(std::size_t index) const
{
  if (waypoints_.empty())
    return 0.0;

  if (index >= waypoints_.size())
    index = waypoints_.size() - 1;

  return waypoints_[index].get().cast_const<tesseract_planning::StateWaypoint>()->time;
}

std::size_t TrajectoryInterpolator::getMoveInstructionCount() const { return waypoints_.size(); }

tesseract_planning::StateWaypoint TrajectoryInterpolator::interpolate(const tesseract_planning::StateWaypoint& start,
                                                                      const tesseract_planning::StateWaypoint& end,
                                                                      double t) const
{
  tesseract_planning::StateWaypoint out;
  out.time = start.time + t;
  out.joint_names = start.joint_names;
  out.position.resize(static_cast<long>(out.joint_names.size()));

  for (long i = 0; i < static_cast<long>(out.joint_names.size()); ++i)
    out.position[i] = start.position[i] + (end.position[i] - start.position[i]) * t;

  return out;
}

bool TrajectoryInterpolator::empty() const { return waypoints_.empty(); };

}  // namespace tesseract_visualization
