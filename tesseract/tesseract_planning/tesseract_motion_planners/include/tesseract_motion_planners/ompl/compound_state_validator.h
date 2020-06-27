/**
 * @file compound_state_validator.h
 * @brief Tesseract OMPL planner OMPL compund state validator
 *
 * @author Levi Armstrong
 * @date February 17, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_COMPOUND_STATE_VALIDATOR_H
#define TESSERACT_MOTION_PLANNERS_COMPOUND_STATE_VALIDATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/** @brief Create a single state validity checker from multiple */
class CompoundStateValidator : public ompl::base::StateValidityChecker
{
public:
  CompoundStateValidator();
  CompoundStateValidator(ompl::base::StateValidityCheckerPtr validator);
  CompoundStateValidator(ompl::base::StateValidityCheckerFn validator);

  bool isValid(const ompl::base::State* state) const override;

  void addStateValidator(ompl::base::StateValidityCheckerPtr validator);
  void addStateValidator(ompl::base::StateValidityCheckerFn validator);

private:
  std::vector<ompl::base::StateValidityCheckerPtr> cache_;
  std::vector<ompl::base::StateValidityCheckerFn> validators_;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_COMPOUND_STATE_VALIDATOR_H
