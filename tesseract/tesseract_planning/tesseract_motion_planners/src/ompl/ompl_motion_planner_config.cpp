/**
 * @file ompl_motion_planner_config.cpp
 * @brief Tesseract OMPL motion planner config implementation.
 *
 * @author Levi Armstrong
 * @date January 22, 2020
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

#include <tesseract_motion_planners/ompl/ompl_motion_planner_config.h>
#include <tesseract/tesseract.h>

namespace tesseract_planning
{

bool OMPLMotionPlannerConfig::generate()
{
  for (const auto& sub_prob : prob)
    if (!((sub_prob->simple_setup != nullptr) && (sub_prob->tesseract != nullptr) && (sub_prob->manip_fwd_kin != nullptr) && (!sub_prob->planners.empty()) && (sub_prob->extractor != nullptr)))
      return false;

  return true;
}

}  // namespace tesseract_motion_planners
