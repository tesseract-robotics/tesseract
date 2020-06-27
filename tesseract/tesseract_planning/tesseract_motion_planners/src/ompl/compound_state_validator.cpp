/**
 * @file compound_state_validator.cpp
 * @brief Tesseract OMPL planner OMPL compound state validator
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/SpaceInformation.h>
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/compound_state_validator.h>

namespace tesseract_planning
{
CompoundStateValidator::CompoundStateValidator() : StateValidityChecker(nullptr) {}

CompoundStateValidator::CompoundStateValidator(ompl::base::StateValidityCheckerPtr validator)
  : StateValidityChecker(nullptr)
{
  addStateValidator(validator);
}

CompoundStateValidator::CompoundStateValidator(ompl::base::StateValidityCheckerFn validator)
  : StateValidityChecker(nullptr)
{
  addStateValidator(validator);
}

bool CompoundStateValidator::isValid(const ompl::base::State* state) const
{
  for (const auto& fn : validators_)
    if (!fn(state))
      return false;

  return true;
}

void CompoundStateValidator::addStateValidator(ompl::base::StateValidityCheckerPtr validator)
{
  auto fn = [validator](const ompl::base::State* state) { return validator->isValid(state); };

  cache_.push_back(std::move(validator));
  validators_.push_back(fn);
}

void CompoundStateValidator::addStateValidator(ompl::base::StateValidityCheckerFn validator)
{
  validators_.push_back(std::move(validator));
}

}  // namespace tesseract_planning
