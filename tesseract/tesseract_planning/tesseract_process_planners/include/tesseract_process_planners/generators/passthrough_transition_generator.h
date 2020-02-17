/**
 * @file passthrough_transition_generator.h
 * @brief generator for transitions that pass through a list of supplied points
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

#ifndef TESSERACT_PLANNING_PASSTHROUGH_TRANSITION_GENERATOR_H
#define TESSERACT_PLANNING_PASSTHROUGH_TRANSITION_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <Eigen/Core>

namespace tesseract_process_planners
{
class PassthroughTransitionGenerator : public ProcessTransitionGenerator
{
public:
  PassthroughTransitionGenerator() = default;
  ~PassthroughTransitionGenerator() override = default;
  PassthroughTransitionGenerator(const PassthroughTransitionGenerator&) = default;
  PassthroughTransitionGenerator& operator=(const PassthroughTransitionGenerator&) = default;
  PassthroughTransitionGenerator(PassthroughTransitionGenerator&&) = default;
  PassthroughTransitionGenerator& operator=(PassthroughTransitionGenerator&&) = default;

  std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const tesseract_motion_planners::Waypoint::Ptr& start_waypoint,
           const tesseract_motion_planners::Waypoint::Ptr& end_waypoint) const override
  {
    std::vector<tesseract_motion_planners::Waypoint::Ptr> transition;
    transition.reserve(2);
    transition.push_back(start_waypoint);
    transition.push_back(end_waypoint);

    return transition;
  }
};

}  // namespace tesseract_process_planners
#endif  // TESSERACT_PLANNING_PASSTHROUGH_TRANSITION_GENERATOR_H
