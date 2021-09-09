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

#include <tesseract_visualization/trajectory_interpolator.h>

namespace tesseract_visualization
{
TrajectoryInterpolator::TrajectoryInterpolator(tesseract_common::JointTrajectory trajectory)
  : trajectory_(std::move(trajectory))
{
  double last_time = 0;
  double current_time = 0;
  double total_time = 0;
  bool overwrite_dt = false;
  // Check if time is populated
  if (!trajectory_.empty() && (trajectory_.back().time - trajectory_.front().time) < 1e-3)
    overwrite_dt = true;

  bool initial_state = true;

  for (auto& state : trajectory_)
  {
    current_time = state.time;

    // It is possible for sub composites to start back from zero, this accounts for it
    if (current_time < last_time)
      last_time = 0;

    double dt = current_time - last_time;
    if (overwrite_dt)
    {
      if (initial_state)
        dt = 0;
      else
        dt = 0.1;
    }
    initial_state = false;
    total_time += dt;
    duration_from_previous_.push_back(dt);
    state.time = total_time;
    last_time = current_time;
  }
}

void TrajectoryInterpolator::findStateIndices(const double& duration, long& before, long& after, double& blend) const
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
  std::size_t num_points = trajectory_.size();
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
  if ((index == 0) || (after == before))
    blend = 1.0;
  else
    blend = (duration - before_time) / duration_from_previous_[index];
}

tesseract_common::JointState TrajectoryInterpolator::getState(double request_duration) const
{
  // If there are no waypoints we can't do anything
  if (trajectory_.empty())
    throw std::runtime_error("Invalid duration");

  long before = 0;
  long after = 0;
  double blend = 1.0;
  findStateIndices(request_duration, before, after, blend);

  if (before < 0 && after < 0)
    throw std::runtime_error("Invalid duration");

  if (before < 0 && after == 0)
    return trajectory_[static_cast<std::size_t>(after)];

  if (before == static_cast<int>(trajectory_.size()) - 1)
    return trajectory_[static_cast<std::size_t>(before)];

  if (before >= 0 && after > 0)
  {
    const tesseract_common::JointState& swp0 = trajectory_[static_cast<std::size_t>(before)];
    const tesseract_common::JointState& swp1 = trajectory_[static_cast<std::size_t>(after)];
    return interpolate(swp0, swp1, blend);
  }

  throw std::runtime_error("Invalid duration");
}

double TrajectoryInterpolator::getStateDuration(long index) const
{
  if (trajectory_.empty())
    return 0.0;

  int s = static_cast<int>(trajectory_.size());
  if (index >= s)
    index = s - 1;

  return trajectory_[static_cast<std::size_t>(index)].time;
}

long TrajectoryInterpolator::getStateCount() const { return static_cast<long>(trajectory_.size()); }

tesseract_common::JointState TrajectoryInterpolator::interpolate(const tesseract_common::JointState& start,
                                                                 const tesseract_common::JointState& end,
                                                                 double t)
{
  assert(!start.joint_names.empty());
  assert(!end.joint_names.empty());
  assert(start.position.rows() != 0);
  assert(end.position.rows() != 0);
  tesseract_common::JointState out;
  out.time = start.time + t;
  out.joint_names = start.joint_names;
  out.position.resize(static_cast<long>(out.joint_names.size()));

  for (long i = 0; i < static_cast<long>(out.joint_names.size()); ++i)
    out.position[i] = start.position[i] + (end.position[i] - start.position[i]) * t;

  return out;
}

bool TrajectoryInterpolator::empty() const { return trajectory_.empty(); };

}  // namespace tesseract_visualization
