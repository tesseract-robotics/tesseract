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

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/utils/flatten_utils.h>
#include <tesseract_time_parameterization/iterative_spline_parameterization.h>

namespace tesseract_planning
{
IterativeSplineParameterizationProfile::IterativeSplineParameterizationProfile(double max_velocity_scaling_factor,
                                                                               double max_acceleration_scaling_factor)
  : max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
{
}

IterativeSplineParameterizationProcessGenerator::IterativeSplineParameterizationProcessGenerator(bool add_points,
                                                                                                 std::string name)
  : name_(std::move(name)), solver_(add_points)
{
  // Register default profile
  composite_profiles["DEFAULT"] = std::make_shared<IterativeSplineParameterizationProfile>();
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
  Instruction* input_results = input.getResults();
  if (!isCompositeInstruction(*input_results))
  {
    CONSOLE_BRIDGE_logError("Input results to iterative spline parameterization must be a composite instruction");
    return 0;
  }

  auto* ci = input_results->cast<CompositeInstruction>();
  const ManipulatorInfo& manip_info = ci->getManipulatorInfo();
  const auto fwd_kin =
      input.tesseract->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);

  // Get Plan Profile
  std::string profile = ci->getProfile();
  profile = getProfileString(profile, name_, input.composite_profile_remapping);
  auto cur_composite_profile = getProfile<IterativeSplineParameterizationProfile>(
      profile, composite_profiles, std::make_shared<IterativeSplineParameterizationProfile>());
  if (!cur_composite_profile)
    cur_composite_profile = std::make_shared<IterativeSplineParameterizationProfile>();

  // Create data structures for checking for plan profile overrides
  auto flattened = flatten(*ci, moveFilter);
  if (flattened.empty())
  {
    CONSOLE_BRIDGE_logWarn("Iterative spline time parameterization found no MoveInstructions to process");
    return 1;
  }

  Eigen::VectorXd velocity_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                             cur_composite_profile->max_velocity_scaling_factor;
  Eigen::VectorXd acceleration_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                                 cur_composite_profile->max_acceleration_scaling_factor;

  // Loop over all PlanInstructions
  for (Eigen::Index idx = 0; idx < static_cast<Eigen::Index>(flattened.size()); idx++)
  {
    profile = flattened[static_cast<std::size_t>(idx)].get().cast_const<MoveInstruction>()->getProfile();

    // Check for remapping of the plan profile
    std::string remap = getProfileString(profile, name_, input.plan_profile_remapping);
    auto cur_composite_profile = getProfile<IterativeSplineParameterizationProfile>(
        profile, composite_profiles, std::make_shared<IterativeSplineParameterizationProfile>());

    // If there is a move profile associated with it, override the parameters
    auto it = move_profiles.find(profile);
    if (it != move_profiles.end())
    {
      velocity_scaling_factors[idx] = it->second->max_velocity_scaling_factor;
      acceleration_scaling_factors[idx] = it->second->max_acceleration_scaling_factor;
    }
  }

  // Solve using parameters
  if (!solver_.compute(*ci,
                       fwd_kin->getLimits().velocity_limits,
                       fwd_kin->getLimits().acceleration_limits,
                       velocity_scaling_factors,
                       acceleration_scaling_factors))
  {
    CONSOLE_BRIDGE_logInform("Failed to perform iterative spline time parameterization for process input: %s!",
                             input_results->getDescription().c_str());
    return 0;
  }

  CONSOLE_BRIDGE_logDebug("Iterative spline time parameterization succeeded");
  return 1;
}

void IterativeSplineParameterizationProcessGenerator::process(ProcessInput input) const { conditionalProcess(input); }

bool IterativeSplineParameterizationProcessGenerator::getAbort() const { return abort_; }
void IterativeSplineParameterizationProcessGenerator::setAbort(bool abort) { abort_ = abort; }

}  // namespace tesseract_planning
