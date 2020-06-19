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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>

namespace tesseract_planning
{
/** @brief Construct a basic planner */
OMPLMotionPlanner::OMPLMotionPlanner(std::string name)
  : MotionPlanner(std::move(name))
  , config_(nullptr)
  , status_category_(std::make_shared<const OMPLMotionPlannerStatusCategory>(name_))
{
}

bool OMPLMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

tesseract_common::StatusCode OMPLMotionPlanner::solve(PlannerResponse& response,
                                                      PostPlanCheckType check_type,
                                                      bool verbose)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  // If the verbose set the log level to debug.
  if (verbose)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // TODO: LEVI need to expand this to support multiple motion planns leveraging taskflow
  assert(config_->prob.size() == 1);
  for (auto& p : config_->prob)
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
      response.status =
          tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorFailedToFindValidSolution, status_category_);
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

    tesseract_common::TrajArray traj = p->getTrajectory();

    assert(p->simple_setup->getProblemDefinition()->getStartStateCount() == 1);
    assert(p->extractor(p->simple_setup->getProblemDefinition()->getStartState(0))
               .transpose()
               .isApprox(traj.row(0), 1e-5));
    assert(p->extractor(p->simple_setup->getProblemDefinition()->getGoal()->as<ompl::base::GoalState>()->getState())
            .transpose()
            .isApprox(traj.bottomRows(1), 1e-5));

    //  // Check and report collisions
    //  continuous_contact_manager_->setContactDistanceThreshold(0.0);

    bool valid = true;
    //  {
    //    auto env = config_->prob.tesseract->getEnvironmentConst();
    //    auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
    //        env->getSceneGraph(), config_->prob.manip_fwd_kin->getActiveLinkNames(), config_->prob.env_state->link_transforms);
    //    auto discrete_contact_manager = env->getDiscreteContactManager();
    //    discrete_contact_manager->setActiveCollisionObjects(adj_map->getActiveLinkNames());
    //    discrete_contact_manager->setContactDistanceThreshold(0.0);

    //    tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();

    //    // TODO: LEVI
    ////    validator_ = std::make_shared<TrajectoryValidator>(
    ////        continuous_contact_manager_, discrete_contact_manager, config_->prob.longest_valid_segment_length, verbose);
    ////    valid = validator_->trajectoryValid(traj, check_type, *state_solver, kin_->getJointNames());
    //  }

    //  // Set the contact distance back to original incase solve was called again.
    ////  continuous_contact_manager_->setContactDistanceThreshold(config_->prob.collision_safety_margin); TODO: LEVI

    // Send response
    response.joint_trajectory.trajectory = traj;
    response.joint_trajectory.joint_names = p->manip_fwd_kin->getJointNames();
    if (!valid)
    {
      response.status = tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision,
                                                     status_category_);
    }
    else
    {
      response.status = tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::SolutionFound, status_category_);
      CONSOLE_BRIDGE_logInform("%s, final trajectory is collision free", name_.c_str());
    }

    return response.status;
  }
}

void OMPLMotionPlanner::clear()
{
  request_ = PlannerRequest();
  config_ = nullptr;
  continuous_contact_manager_ = nullptr;
  parallel_plan_ = nullptr;
}

tesseract_common::StatusCode OMPLMotionPlanner::isConfigured() const
{
  if (config_ == nullptr) // || continuous_contact_manager_ == nullptr)
    return tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorIsNotConfigured, status_category_);

  return tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::IsConfigured, status_category_);
}

bool OMPLMotionPlanner::setConfiguration(OMPLMotionPlannerConfig::Ptr config)
{
  // Reset state
  clear();

  config_ = std::move(config);
  if (!config_->generate())
  {
    config_ = nullptr;
    return false;
  }

//  const tesseract_environment::Environment::ConstPtr& env = config_->prob.tesseract->getEnvironmentConst();
//  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
//  // to determine all active links.
//  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
//      env->getSceneGraph(), config_->prob.manip_fwd_kin->getActiveLinkNames(), config_->prob.env_state->link_transforms);

//  continuous_contact_manager_ = env->getContinuousContactManager();
//  continuous_contact_manager_->setActiveCollisionObjects(adj_map->getActiveLinkNames());
////  continuous_contact_manager_->setContactDistanceThreshold(config_->collision_safety_margin); TODO: LEVI

//  parallel_plan_ = std::make_shared<ompl::tools::ParallelPlan>(config_->prob.simple_setup->getProblemDefinition());

//  for (const auto& planner : config_->prob.planners)
//    parallel_plan_->addPlanner(planner->create(config_->prob.simple_setup->getSpaceInformation()));

  return true;
}

}  // namespace tesseract_motion_planners
