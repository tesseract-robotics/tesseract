/**
 * @file ompl_motion_planner.cpp
 * @brief Tesseract OMPL motion planner
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
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/ompl_motion_planner_status_category.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>

#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
bool checkStartState(const ompl::base::ProblemDefinitionPtr& prob_def,
                     const Eigen::Ref<const Eigen::VectorXd>& state,
                     const OMPLStateExtractor& extractor)
{
  if (!(prob_def->getStartStateCount() >= 1))
    return false;

  for (unsigned i = 0; i < prob_def->getStartStateCount(); ++i)
    if (extractor(prob_def->getStartState(i)).isApprox(state, 1e-5))
      return true;

  return false;
}

bool checkGoalState(const ompl::base::ProblemDefinitionPtr& prob_def,
                    const Eigen::Ref<const Eigen::VectorXd>& state,
                    const OMPLStateExtractor& extractor)
{
  ompl::base::GoalPtr goal = prob_def->getGoal();
  if (goal->getType() == ompl::base::GoalType::GOAL_STATE)
  {
    return extractor(prob_def->getGoal()->as<ompl::base::GoalState>()->getState()).isApprox(state, 1e-5);
  }
  else if (goal->getType() == ompl::base::GoalType::GOAL_STATES)
  {
    auto goal_states = prob_def->getGoal()->as<ompl::base::GoalStates>();
    for (unsigned i = 0; i < goal_states->getStateCount(); ++i)
      if (extractor(goal_states->getState(i)).isApprox(state, 1e-5))
        return true;
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("checkGoalStates: Unsupported Goal Type!");
    return true;
  }
  return false;
}

/** @brief Construct a basic planner */
OMPLMotionPlanner::OMPLMotionPlanner(std::string name)
  : MotionPlanner(std::move(name)), status_category_(std::make_shared<const OMPLMotionPlannerStatusCategory>(name_))
{
}

bool OMPLMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

tesseract_common::StatusCode OMPLMotionPlanner::solve(const PlannerRequest& request,
                                                      PlannerResponse& response,
                                                      bool verbose) const
{
  if (!checkUserInput(request))
  {
    response.status =
        tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }
  std::vector<OMPLProblem::Ptr> problem;
  if (request.data)
  {
    problem = *std::static_pointer_cast<std::vector<OMPLProblem::Ptr>>(request.data);
  }
  else
  {
    if (!problem_generator)
    {
      CONSOLE_BRIDGE_logError("OMPLPlanner does not have a problem generator specified.");
      response.status =
          tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
      return response.status;
    }

    try
    {
      problem = problem_generator(name_, request, plan_profiles);
    }
    catch (std::exception& e)
    {
      CONSOLE_BRIDGE_logError("OMPLPlanner failed to generate problem: %s.", e.what());
      response.status =
          tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
      return response.status;
    }

    response.data = std::make_shared<std::vector<OMPLProblem::Ptr>>(problem);
  }

  // If the verbose set the log level to debug.
  if (verbose)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  /// @todo: Need to expand this to support multiple motion plans leveraging taskflow
  assert(problem.size() == 1);
  for (auto& p : problem)
  {
    auto parallel_plan = std::make_shared<ompl::tools::ParallelPlan>(p->simple_setup->getProblemDefinition());

    for (const auto& planner : p->planners)
      parallel_plan->addPlanner(planner->create(p->simple_setup->getSpaceInformation()));

    ompl::base::PlannerStatus status;
    if (!p->optimize)
    {
      // Solve problem. Results are stored in the response
      // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
      // and finishes at the end state.
      status = parallel_plan->solve(p->planning_time, 1, static_cast<unsigned>(p->max_solutions), false);
    }
    else
    {
      ompl::time::point end = ompl::time::now() + ompl::time::seconds(p->planning_time);
      const ompl::base::ProblemDefinitionPtr& pdef = p->simple_setup->getProblemDefinition();
      while (ompl::time::now() < end)
      {
        // Solve problem. Results are stored in the response
        // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
        // and finishes at the end state.
        ompl::base::PlannerStatus localResult =
            parallel_plan->solve(std::max(ompl::time::seconds(end - ompl::time::now()), 0.0),
                                 1,
                                 static_cast<unsigned>(p->max_solutions),
                                 false);
        if (localResult)
        {
          if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
            status = localResult;

          if (!pdef->hasOptimizationObjective())
          {
            CONSOLE_BRIDGE_logDebug("Terminating early since there is no optimization objective specified");
            break;
          }

          ompl::base::Cost obj_cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective());
          CONSOLE_BRIDGE_logDebug("Motion Objective Cost: %f", obj_cost.value());

          if (pdef->getOptimizationObjective()->isSatisfied(obj_cost))
          {
            CONSOLE_BRIDGE_logDebug("Terminating early since solution path satisfies the optimization objective");
            break;
          }

          if (pdef->getSolutionCount() >= static_cast<std::size_t>(p->max_solutions))
          {
            CONSOLE_BRIDGE_logDebug("Terminating early since %u solutions were generated", p->max_solutions);
            break;
          }
        }
      }
    }

    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
      response.status = tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorFailedToFindValidSolution,
                                                     status_category_);
      return response.status;
    }

    if (p->simplify)
    {
      p->simple_setup->simplifySolution();
    }
    else
    {
      // Interpolate the path if it shouldn't be simplified and there are currently fewer states than requested
      auto num_output_states = static_cast<unsigned>(p->n_output_states);
      if (p->simple_setup->getSolutionPath().getStateCount() < num_output_states)
      {
        p->simple_setup->getSolutionPath().interpolate(num_output_states);
      }
      else
      {
        // Now try to simplify the trajectory to get it under the requested number of output states
        // The interpolate function only executes if the current number of states is less than the requested
        p->simple_setup->simplifySolution();
        if (p->simple_setup->getSolutionPath().getStateCount() < num_output_states)
          p->simple_setup->getSolutionPath().interpolate(num_output_states);
      }
    }

    // Get the results
    tesseract_common::TrajArray trajectory = p->getTrajectory();

    assert(checkStartState(p->simple_setup->getProblemDefinition(), trajectory.row(0), p->extractor));
    assert(checkGoalState(p->simple_setup->getProblemDefinition(), trajectory.bottomRows(1).transpose(), p->extractor));

    // Flatten the results to make them easier to process
    response.results = request.seed;
    std::vector<std::reference_wrapper<Instruction>> results_flattened =
        flattenProgramToPattern(response.results, request.instructions);
    std::vector<std::reference_wrapper<const Instruction>> instructions_flattened =
        flattenProgram(request.instructions);

    // Loop over the flattened results and add them to response if the input was a plan instruction
    Eigen::Index result_index = 0;
    for (std::size_t idx = 0; idx < instructions_flattened.size(); idx++)
    {
      // If plan_index is zero then this should be the start instruction
      assert((idx == 0) ? isPlanInstruction(instructions_flattened.at(idx).get()) : true);
      assert((idx == 0) ? isMoveInstruction(results_flattened[idx].get()) : true);
      if (isPlanInstruction(instructions_flattened.at(idx).get()))
      {
        // This instruction corresponds to a composite. Set all results in that composite to the results
        const auto* plan_instruction = instructions_flattened.at(idx).get().cast_const<PlanInstruction>();
        if (plan_instruction->isStart())
        {
          assert(idx == 0);
          assert(isMoveInstruction(results_flattened[idx].get()));
          auto* move_instruction = results_flattened[idx].get().cast<MoveInstruction>();
          move_instruction->getWaypoint().cast<StateWaypoint>()->position = trajectory.row(result_index++);
        }
        else
        {
          auto* move_instructions = results_flattened[idx].get().cast<CompositeInstruction>();
          for (auto& instruction : *move_instructions)
            instruction.cast<MoveInstruction>()->getWaypoint().cast<StateWaypoint>()->position =
                trajectory.row(result_index++);
        }
      }
    }

    response.status = tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::SolutionFound, status_category_);
    return response.status;
  }

  return response.status;
}

void OMPLMotionPlanner::clear() { parallel_plan_ = nullptr; }

bool OMPLMotionPlanner::checkUserInput(const PlannerRequest& request) const
{
  // Check that parameters are valid
  if (request.tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In TrajOptPlannerUniversalConfig: tesseract is a required parameter and has not been set");
    return false;
  }

  if (request.instructions.empty())
  {
    CONSOLE_BRIDGE_logError("TrajOptPlannerUniversalConfig requires at least one instruction");
    return false;
  }

  return true;
}

}  // namespace tesseract_planning
