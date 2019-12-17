/**
 * @file descartes_motion_planner_status_category.h
 * @brief Tesseract ROS Descartes planner status category
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_MOTION_PLANNER_STATUS_CATEGORY_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_MOTION_PLANNER_STATUS_CATEGORY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/status_code.h>

namespace tesseract_motion_planners
{
/**
 * @brief The Descartes motion planner status category
 *
 * It contains both successfull and error status codes.

 */
class DescartesMotionPlannerStatusCategory : public tesseract_common::StatusCategory
{
public:
  DescartesMotionPlannerStatusCategory(std::string name);
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    IsConfigured = 1,
    SolutionFound = 0,
    ErrorIsNotConfigured = -1,
    ErrorFailedToParseConfig = -2,
    ErrorFailedToBuildGraph = -3,
    ErrorFailedToFindValidSolution = -4,
    ErrorFoundValidSolutionInCollision = -5
  };

private:
  std::string name_; /**< @brief The name of the status category */
};
}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_MOTION_PLANNER_STATUS_CATEGORY_H
