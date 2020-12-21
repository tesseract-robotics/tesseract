/**
 * @file simple_planner_default_plan_profile.h
 * @brief
 *
 * @author Tyler Marr
 * @date September 16, 2020
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

#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_DEFAULT_LVS_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_DEFAULT_LVS_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_sco/modeling_utils.hpp>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::SimplePlannerDefaultLVSPlanProfile)
#endif  // SWIG

namespace tesseract_planning
{
class SimplePlannerDefaultLVSPlanProfile : public SimplePlannerPlanProfile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerDefaultLVSPlanProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerDefaultLVSPlanProfile>;

  SimplePlannerDefaultLVSPlanProfile(double state_longest_valid_segment_length = 5 * M_PI / 180,
                                     double translation_longest_valid_segment_length = 0.1,
                                     double rotation_longest_valid_segment_length = 5 * M_PI / 180,
                                     int min_steps = 1);

  /** @brief apply Sets the function handles based on the number of steps specified */
  void apply();

  const double& getStateLongestValidSegmentLength();
  void setStateLongestValidSegmentLength(double state_longest_valid_segment_length);

  const double& getTranslationLongestValidSegmentLength();
  void setTranslationLongestValidSegmentLength(double translation_longest_valid_segment_length);

  const double& getRotationLongestValidSegmentLength();
  void setRotationLongestValidSegmentLength(double rotation_longest_valid_segment_length);

  const int& getMinSteps();
  void setMinSteps(int min_steps);

protected:
  double state_longest_valid_segment_length_;
  double translation_longest_valid_segment_length_;
  double rotation_longest_valid_segment_length_;
  int min_steps_;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_DEFAULT_PLAN_PROFILE_H
