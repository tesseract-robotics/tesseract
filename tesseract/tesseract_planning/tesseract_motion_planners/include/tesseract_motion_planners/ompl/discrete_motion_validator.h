/**
 * @file discrete_motion_validator.h
 * @brief Tesseract OMPL planner discrete collision check between two states
 *
 * @author Jonathan Meyer, Levi Armstrong
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_DISCRETE_MOTION_VALIDATOR_H
#define TESSERACT_MOTION_PLANNERS_OMPL_DISCRETE_MOTION_VALIDATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/MotionValidator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/** @brief Continuous collision check between two states */
class DiscreteMotionValidator : public ompl::base::MotionValidator
{
public:
  DiscreteMotionValidator(const ompl::base::SpaceInformationPtr& space_info);

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

  bool checkMotion(const ompl::base::State* s1,
                   const ompl::base::State* s2,
                   std::pair<ompl::base::State*, double>& lastValid) const override;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_DISCRETE_MOTION_VALIDATOR_H
