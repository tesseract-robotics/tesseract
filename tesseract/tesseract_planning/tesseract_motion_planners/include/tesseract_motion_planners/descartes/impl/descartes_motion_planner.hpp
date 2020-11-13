/**
 * @file descartes_motion_planner.hpp
 * @brief Tesseract ROS Descartes planner
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
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP

#include <tesseract/tesseract.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/descartes_light.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_samplers/samplers/railed_cartesian_point_sampler.h>
#include <descartes_samplers/samplers/railed_axial_symmetric_sampler.h>
#include <descartes_samplers/samplers/axial_symmetric_sampler.h>
#include <descartes_samplers/samplers/cartesian_point_sampler.h>
#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>
#include <descartes_samplers/evaluators/timing_edge_evaluator.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
DescartesMotionPlanner<FloatType>::DescartesMotionPlanner()
  : status_category_(std::make_shared<const DescartesMotionPlannerStatusCategory>(name_))
{
  plan_profiles[DEFAULT_PROFILE_KEY] = std::make_shared<DescartesDefaultPlanProfile<FloatType>>();
}

template <typename FloatType>
const std::string& DescartesMotionPlanner<FloatType>::getName() const
{
  return name_;
}

template <typename FloatType>
tesseract_common::StatusCode DescartesMotionPlanner<FloatType>::solve(const PlannerRequest& request,
                                                                      PlannerResponse& response,
                                                                      const bool /*verbose*/) const
{
  std::shared_ptr<DescartesProblem<FloatType>> problem;
  if (request.data)
  {
    problem = std::static_pointer_cast<DescartesProblem<FloatType>>(request.data);
  }
  else
  {
    if (!problem_generator)
    {
      CONSOLE_BRIDGE_logError("DescartesMotionPlanner does not have a problem generator specified.");
      response.status =
          tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
      return response.status;
    }

    try
    {
      problem = problem_generator(name_, request, plan_profiles);
    }
    catch (std::exception& e)
    {
      CONSOLE_BRIDGE_logError("DescartesMotionPlanner failed to generate problem: %s.", e.what());
      response.status =
          tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
      return response.status;
    }

    response.data = problem;
  }

  descartes_light::Solver<FloatType> graph_builder(problem->manip_inv_kin->numJoints());
  if (!graph_builder.build(problem->samplers, problem->edge_evaluators, problem->num_threads))
  {
    //    CONSOLE_BRIDGE_logError("Failed to build vertices");
    //    for (const auto& i : graph_builder.getFailedVertices())
    //      response.failed_waypoints.push_back(config_->waypoints[i]);

    //    // Copy the waypoint if it is not already in the failed waypoints list
    //    std::copy_if(config_->waypoints.begin(),
    //                 config_->waypoints.end(),
    //                 std::back_inserter(response.succeeded_waypoints),
    //                 [&response](const Waypoint::ConstPtr wp) {
    //                   return std::find(response.failed_waypoints.begin(), response.failed_waypoints.end(), wp) ==
    //                          response.failed_waypoints.end();
    //                 });

    response.status =
        tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorFailedToBuildGraph, status_category_);
    return response.status;
  }
  //  // No failed waypoints
  //  response.succeeded_waypoints = config_->waypoints;
  //  response.failed_waypoints.clear();

  // Search for edges
  std::vector<FloatType> solution;
  if (!graph_builder.search(solution))
  {
    CONSOLE_BRIDGE_logError("Search for graph completion failed");
    response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorFailedToFindValidSolution,
                                                   status_category_);
    return response.status;
  }

  // Flatten the results to make them easier to process
  response.results = request.seed;
  auto results_flattened = flattenProgramToPattern(response.results, request.instructions);
  auto instructions_flattened = flattenProgram(request.instructions);

  // Loop over the flattened results and add them to response if the input was a plan instruction
  Eigen::Index dof = problem->manip_fwd_kin->numJoints();
  Eigen::Index result_index = 0;
  for (std::size_t idx = 0; idx < instructions_flattened.size(); idx++)
  {
    // If idx is zero then this should be the start instruction
    assert((idx == 0) ? isPlanInstruction(instructions_flattened.at(idx).get()) : true);
    assert((idx == 0) ? isMoveInstruction(results_flattened[idx].get()) : true);
    if (isPlanInstruction(instructions_flattened.at(idx).get()))
    {
      const auto* plan_instruction = instructions_flattened.at(idx).get().cast_const<PlanInstruction>();
      if (plan_instruction->isStart())
      {
        assert(idx == 0);
        assert(isMoveInstruction(results_flattened[idx].get()));
        auto* move_instruction = results_flattened[idx].get().cast<MoveInstruction>();

        Eigen::Map<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> temp(solution.data() + dof * result_index++, dof);
        move_instruction->getWaypoint().cast<StateWaypoint>()->position = temp.template cast<double>();
      }
      else if (plan_instruction->isLinear())
      {
        // This instruction corresponds to a composite. Set all results in that composite to the results
        assert(isCompositeInstruction(results_flattened[idx].get()));
        auto* move_instructions = results_flattened[idx].get().cast<CompositeInstruction>();
        for (auto& instruction : *move_instructions)
        {
          Eigen::Map<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> temp(solution.data() + dof * result_index++,
                                                                             dof);
          instruction.cast<MoveInstruction>()->getWaypoint().cast<StateWaypoint>()->position =
              temp.template cast<double>();
        }
      }
      else if (plan_instruction->isFreespace())
      {
        assert(result_index > 0);
        // Because descartes does not support freespace it just includes the plan instruction waypoint so we will
        // fill out the results with a joint interpolated trajectory.
        Eigen::Map<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> start(solution.data() + dof * (result_index - 1),
                                                                            dof);
        Eigen::Map<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> stop(solution.data() + dof * result_index++, dof);

        // This instruction corresponds to a composite. Set all results in that composite to the results
        assert(isCompositeInstruction(results_flattened[idx].get()));
        auto* move_instructions = results_flattened[idx].get().cast<CompositeInstruction>();

        Eigen::MatrixXd temp = interpolate(
            start.template cast<double>(), stop.template cast<double>(), static_cast<int>(move_instructions->size()));

        assert(temp.cols() == static_cast<long>(move_instructions->size()) + 1);
        for (std::size_t i = 0; i < move_instructions->size(); ++i)
          (*move_instructions)[i].cast<MoveInstruction>()->getWaypoint().cast<StateWaypoint>()->position =
              temp.col(static_cast<long>(i) + 1);
      }
      else
      {
        throw std::runtime_error("Unsupported Plan Instruction Type!");
      }
    }
  }

  response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

template <typename FloatType>
bool DescartesMotionPlanner<FloatType>::checkUserInput(const PlannerRequest& request)
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

template <typename FloatType>
bool DescartesMotionPlanner<FloatType>::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

template <typename FloatType>
void DescartesMotionPlanner<FloatType>::clear()
{
}

template <typename FloatType>
MotionPlanner::Ptr DescartesMotionPlanner<FloatType>::clone() const
{
  return std::make_shared<DescartesMotionPlanner<FloatType>>();
}

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP
