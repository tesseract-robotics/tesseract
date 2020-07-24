/**
 * @file simple_planner_interpolation_plan_profile.h
 * @brief
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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

#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_INTERPOLATION_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_INTERPOLATION_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_sco/modeling_utils.hpp>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

namespace tesseract_planning
{
class SimplePlannerInterpolationPlanProfile : public SimplePlannerPlanProfile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerInterpolationPlanProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerInterpolationPlanProfile>;

  SimplePlannerInterpolationPlanProfile(int freespace_steps = 10, int cartesian_steps = 10);

  /** @brief apply Sets the function handles based on the number of steps specified */
  void apply();

  const int& getFreespaceSteps();
  void setFreespaceSteps(int freespace_steps);

  const int& getCartesianSteps();
  void setCartesianSteps(int cartesian_steps);

protected:
  int freespace_steps_;
  int cartesian_steps_;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_DEFAULT_PLAN_PROFILE_H
