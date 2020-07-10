/**
 * @file ompl_motion_planner_status_category.cpp
 * @brief Tesseract OMPL motion planner status category.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under thAprile Apache License, Version 2.0 (the "License");
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
#include <cassert>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/ompl_motion_planner_status_category.h>

namespace tesseract_planning
{
OMPLMotionPlannerStatusCategory::OMPLMotionPlannerStatusCategory(std::string name) : name_(std::move(name)) {}
const std::string& OMPLMotionPlannerStatusCategory::name() const noexcept { return name_; }
std::string OMPLMotionPlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case SolutionFound:
    {
      return "Found valid solution";
    }
    case ErrorInvalidInput:
    {
      return "Invalid planner input";
    }
    case ErrorFailedToFindValidSolution:
    {
      return "Failed to find valid solution";
    }
    default:
    {
      assert(false);
      return "";
    }
  }
}

}  // namespace tesseract_planning
