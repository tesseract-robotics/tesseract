/**
 * @file linear_transition_generator.h
 * @brief generator for a linear transition
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

#ifndef TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H
#define TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <tesseract_motion_planners/core/utils.h>
#include <Eigen/Core>

namespace tesseract_process_planners
{
class LinearTransitionGenerator : public ProcessTransitionGenerator
{
public:
  LinearTransitionGenerator(int step_count) : step_count_(step_count) {}
  ~LinearTransitionGenerator() override = default;
  LinearTransitionGenerator(const LinearTransitionGenerator&) = default;
  LinearTransitionGenerator& operator=(const LinearTransitionGenerator&) = default;
  LinearTransitionGenerator(LinearTransitionGenerator&&) = default;
  LinearTransitionGenerator& operator=(LinearTransitionGenerator&&) = default;

  std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const tesseract_motion_planners::Waypoint::Ptr& start_waypoint,
           const tesseract_motion_planners::Waypoint::Ptr& end_waypoint) const override
  {
    assert(start_waypoint->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
    assert(end_waypoint->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);

    const tesseract_motion_planners::CartesianWaypoint::Ptr& w1 =
        std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(start_waypoint);
    const tesseract_motion_planners::CartesianWaypoint::Ptr& w2 =
        std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(end_waypoint);

    return tesseract_motion_planners::interpolate(*w1, *w2, step_count_);
  }

private:
  int step_count_;
};

}  // namespace tesseract_process_planners

#endif  // TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H
