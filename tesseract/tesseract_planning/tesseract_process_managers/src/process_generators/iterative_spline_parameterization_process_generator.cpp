/**
 * @file iterative_spline_parameterization_process_generator.cpp
 * @brief Perform iterative spline time parameterization
 *
 * @author Levi Armstrong
 * @date August 11. 2020
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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>

namespace tesseract_planning
{
IterativeSplineParameterizationProcessGenerator::IterativeSplineParameterizationProcessGenerator(bool add_points,
                                                                                                 std::string name)
  : name_(std::move(name)), solver_(add_points)
{
}

const std::string& IterativeSplineParameterizationProcessGenerator::getName() const { return name_; }

std::function<void()> IterativeSplineParameterizationProcessGenerator::generateTask(ProcessInput input)
{
  return [=]() { process(input); };
}

std::function<int()> IterativeSplineParameterizationProcessGenerator::generateConditionalTask(ProcessInput input)
{
  return [=]() { return conditionalProcess(input); };
}

int IterativeSplineParameterizationProcessGenerator::conditionalProcess(ProcessInput input) const
{
  if (abort_)
    return 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  if (!isCompositeInstruction(*(input.results)))
  {
    CONSOLE_BRIDGE_logError("Input results to iterative spline parameterization must be a composite instruction");
    return 0;
  }

  auto* ci = input.results->cast<CompositeInstruction>();
  const ManipulatorInfo& manip_info = ci->getManipulatorInfo();
  const auto fwd_kin = input.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manip_info.manipulator);

  if (!solver_.compute(*ci,
                       fwd_kin->getLimits().velocity_limits,
                       fwd_kin->getLimits().acceleration_limits,
                       max_velocity_scaling_factor,
                       max_acceleration_scaling_factor))
  {
    CONSOLE_BRIDGE_logInform("Failed to perform iterative spline time parameterization!");
    return 0;
  }

  CONSOLE_BRIDGE_logDebug("Iterative spline time parameterization succeeded");
  return 1;
}

void IterativeSplineParameterizationProcessGenerator::process(ProcessInput input) const { conditionalProcess(input); }

bool IterativeSplineParameterizationProcessGenerator::getAbort() const { return abort_; }
void IterativeSplineParameterizationProcessGenerator::setAbort(bool abort) { abort_ = abort; }

}  // namespace tesseract_planning
