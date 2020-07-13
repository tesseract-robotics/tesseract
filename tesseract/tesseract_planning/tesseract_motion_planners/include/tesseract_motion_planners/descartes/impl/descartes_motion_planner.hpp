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
#include <descartes_samplers/evaluators/distance_edge_evaluator.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
DescartesMotionPlanner<FloatType>::DescartesMotionPlanner(std::string name)
  : MotionPlanner(name), status_category_(std::make_shared<const DescartesMotionPlannerStatusCategory>(name))
{
}

template <typename FloatType>
tesseract_common::StatusCode DescartesMotionPlanner<FloatType>::solve(const PlannerRequest& request,
                                                                      PlannerResponse& response,
                                                                      const bool /*verbose*/)
{
  if (!problem_generator)
  {
    CONSOLE_BRIDGE_logError("DescartesMotionPlanner does not have a problem generator specified.");
    response.status =
        tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }
  DescartesProblem<FloatType> problem = problem_generator(request, plan_profiles);

  descartes_light::Solver<FloatType> graph_builder(problem.manip_inv_kin->numJoints());
  if (!graph_builder.build(problem.samplers, problem.timing_constraints, problem.edge_evaluators, problem.num_threads))
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
  std::vector<std::reference_wrapper<Instruction>> results_flattened =
      FlattenToPattern(response.results, request.instructions);
  std::vector<std::reference_wrapper<const Instruction>> instructions_flattened = Flatten(request.instructions);

  // Loop over the flattened results and add them to response if the input was a plan instruction
  Eigen::Index dof = problem.manip_fwd_kin->numJoints();
  Eigen::Index result_index = 0;
  for (std::size_t plan_index = 0; plan_index < results_flattened.size(); plan_index++)
  {
    if (instructions_flattened.at(plan_index).get().isPlan())
    {
      // This instruction corresponds to a composite. Set all results in that composite to the results
      auto* move_instructions = results_flattened[plan_index].get().cast<CompositeInstruction>();
      for (auto& instruction : *move_instructions)
      {
        Eigen::Map<Eigen::Matrix<FloatType, -1, 1>> temp(solution.data() + dof * result_index, dof);
        instruction.cast<MoveInstruction>()->setPosition(temp.template cast<double>());
        result_index++;
      }
    }
  }

  response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

template <typename FloatType>
bool DescartesMotionPlanner<FloatType>::checkUserInput(const PlannerRequest& /*request*/)
{
  // TODO: copy from trajopt
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

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP
