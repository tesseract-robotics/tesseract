/**
 * @file trajectory_interpolator.h
 * @brief Trajectory interpolator class
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

/* Based on MoveIt code authored by: Ioan Sucan, Adam Leeper */

#ifndef TESSERACT_VISUALIZATION_TRAJECTORY_INTERPOLATOR_H
#define TESSERACT_VISUALIZATION_TRAJECTORY_INTERPOLATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/joint_state.h>

namespace tesseract_visualization
{
class TrajectoryInterpolator
{
public:
  using UPtr = std::unique_ptr<TrajectoryInterpolator>;

  TrajectoryInterpolator(tesseract_common::JointTrajectory trajectory);
  virtual ~TrajectoryInterpolator() = default;
  TrajectoryInterpolator(const TrajectoryInterpolator&) = delete;
  TrajectoryInterpolator& operator=(const TrajectoryInterpolator&) = delete;
  TrajectoryInterpolator(TrajectoryInterpolator&&) = delete;
  TrajectoryInterpolator& operator=(TrajectoryInterpolator&&) = delete;

  tesseract_common::JointState getState(double request_duration) const;

  double getStateDuration(long index) const;

  long getStateCount() const;

  bool empty() const;

private:
  tesseract_common::JointTrajectory trajectory_;
  std::vector<double> duration_from_previous_;

  void findStateIndices(const double& duration, long& before, long& after, double& blend) const;

  tesseract_common::JointState interpolate(const tesseract_common::JointState& start,
                                           const tesseract_common::JointState& end,
                                           double t) const;
};
}  // namespace tesseract_visualization
#endif  // TESSERACT_VISUALIZATION_TRAJECTORY_INTERPOLATOR_H
