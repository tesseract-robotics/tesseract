/**
 * @file trajopt_planner.cpp
 * @brief Tesseract ROS Trajopt planner
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <tesseract_environment/core/utils.h>
#include <boost/date_time/posix_time/posix_time.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_motion_planners/core/utils.h>

using namespace trajopt;

namespace tesseract_planning
{
TrajOptMotionPlannerStatusCategory::TrajOptMotionPlannerStatusCategory(std::string name) : name_(std::move(name)) {}
const std::string& TrajOptMotionPlannerStatusCategory::name() const noexcept { return name_; }
std::string TrajOptMotionPlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case SolutionFound:
    {
      return "Found valid solution";
    }
    case InvalidInput:
    {
      return "Input to planner is invalid. Check that instructions and seed are compatible";
    }
    case FailedToFindValidSolution:
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

TrajOptMotionPlanner::TrajOptMotionPlanner(std::string name)
  : MotionPlanner(std::move(name)), status_category_(std::make_shared<const TrajOptMotionPlannerStatusCategory>(name_))
{
}

bool TrajOptMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

void TrajOptMotionPlanner::clear()
{
  params = sco::BasicTrustRegionSQPParameters();
  callbacks.clear();
}

tesseract_common::StatusCode TrajOptMotionPlanner::solve(const PlannerRequest& request,
                                                         PlannerResponse& response,
                                                         bool verbose)
{
  if (!checkUserInput(request) || !problem_generator)
  {
    response.status = tesseract_common::StatusCode(TrajOptMotionPlannerStatusCategory::InvalidInput, status_category_);
    return response.status;
  }
  auto problem = std::make_shared<trajopt::TrajOptProb>(problem_generator(request, plan_profiles, composite_profiles));

  // Set Log Level
  if (verbose)
    util::gLogLevel = util::LevelInfo;
  else
    util::gLogLevel = util::LevelWarn;

  // Create optimizer
  sco::BasicTrustRegionSQP opt(problem);
  opt.setParameters(params);
  opt.initialize(trajToDblVec(problem->GetInitTraj()));

  // Add all callbacks
  for (const sco::Optimizer::Callback& callback : callbacks)
  {
    opt.addCallback(callback);
  }

  // Optimize
  auto tStart = boost::posix_time::second_clock::local_time();
  opt.optimize();
  CONSOLE_BRIDGE_logInform("planning time: %.3f", (boost::posix_time::second_clock::local_time() - tStart).seconds());
  if (opt.results().status != sco::OptStatus::OPT_CONVERGED)
  {
    response.status =
        tesseract_common::StatusCode(TrajOptMotionPlannerStatusCategory::FailedToFindValidSolution, status_category_);
    return response.status;
  }

  // Get the results
  tesseract_common::TrajArray trajectory = getTraj(opt.x(), problem->GetVars());

  // Flatten the results to make them easier to process
  auto results_flattened = FlattenToPattern(response.results, request.instructions);
  auto instructions_flattened = Flatten(request.instructions);

  // Loop over the flattened results and add them to response if the input was a plan instruction
  Eigen::Index result_index = 0;
  for (std::size_t plan_index = 0; plan_index < results_flattened.size(); plan_index++)
  {
    if (instructions_flattened.at(plan_index).get().isPlan())
    {
      // This instruction corresponds to a composite. Set all results in that composite to the results
      auto* move_instructions = results_flattened[plan_index].get().cast<CompositeInstruction>();
      for (auto& instruction : *move_instructions)
        instruction.cast<MoveInstruction>()->setPosition(trajectory.row(result_index++));
    }
  }

  response.status = tesseract_common::StatusCode(TrajOptMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

bool TrajOptMotionPlanner::checkUserInput(const PlannerRequest& request) const
{
  // Check that parameters are valid
  if (request.tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In TrajOptPlannerUniversalConfig: tesseract is a required parameter and has not been set");
    return false;
  }

  // Check that parameters are valid
  auto manipulators = request.tesseract->getFwdKinematicsManagerConst()->getAvailableFwdKinematicsManipulators();
  if (std::find(manipulators.begin(), manipulators.end(), request.manipulator) == manipulators.end())
  {
    CONSOLE_BRIDGE_logError("In TrajOptPlannerUniversalConfig: manipulator is a required parameter and is not found in "
                            "the list of available manipulators");
    return false;
  }

  if (request.instructions.empty())
  {
    CONSOLE_BRIDGE_logError("TrajOptPlannerUniversalConfig requires at least 2 instructions");
    return false;
  }

  return true;
}

}  // namespace tesseract_planning
