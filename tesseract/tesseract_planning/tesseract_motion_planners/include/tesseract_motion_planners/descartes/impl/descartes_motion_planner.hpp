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
#ifndef TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_HPP
#define TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_HPP

#include <tesseract/tesseract.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/core/waypoint.h>
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

namespace tesseract_motion_planners
{
template <typename FloatType>
DescartesMotionPlanner<FloatType>::DescartesMotionPlanner(std::string name)
  : MotionPlanner(name)
  , config_(nullptr)
  , status_category_(std::make_shared<const DescartesMotionPlannerStatusCategory>(name))
{
}

template <typename FloatType>
bool DescartesMotionPlanner<FloatType>::setConfiguration(const DescartesMotionPlannerConfig<FloatType>& config)
{
  // Check that parameters are valid
  if (config.tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In %s: tesseract is a required parameter and has not been set", name_.c_str());
    return false;
  }

  if (config.edge_evaluator == nullptr)
  {
    CONSOLE_BRIDGE_logError("In %s: edge_evaluator is a required parameter and has not been set", name_.c_str());
    return false;
  }

  if (config.timing_constraint.empty())
  {
    CONSOLE_BRIDGE_logError("In %s: timing_constraint is a required parameter and has not been set", name_.c_str());
    return false;
  }

  if (config.waypoints.empty())
  {
    CONSOLE_BRIDGE_logError("In %s: waypoints is a required parameter and has not been set", name_.c_str());
    return false;
  }

  if (config.samplers.empty())
  {
    CONSOLE_BRIDGE_logError("In %s: waypoints is a required parameter and has not been set", name_.c_str());
    return false;
  }

  if ((config.timing_constraint.size() != config.waypoints.size()) ||
      (config.timing_constraint.size() != config.samplers.size()))
  {
    CONSOLE_BRIDGE_logError("In %s: waypoints, timing_constraint and samplers must be the same size", name_.c_str());
    return false;
  }

  config_ = std::make_shared<DescartesMotionPlannerConfig<FloatType>>(config);
  return true;
}

template <typename FloatType>
tesseract_common::StatusCode DescartesMotionPlanner<FloatType>::solve(PlannerResponse& response, const bool verbose)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  auto tStart = boost::posix_time::second_clock::local_time();

  const auto dof = config_->joint_names.size();
  descartes_light::Solver<FloatType> graph_builder(dof);
  if (!graph_builder.build(
          config_->samplers, config_->timing_constraint, config_->edge_evaluator, config_->num_threads))
  {
    CONSOLE_BRIDGE_logError("Failed to build vertices");
    for (const auto& i : graph_builder.getFailedVertices())
      response.failed_waypoints.push_back(config_->waypoints[i]);

    // Copy the waypoint if it is not already in the failed waypoints list
    std::copy_if(config_->waypoints.begin(),
                 config_->waypoints.end(),
                 std::back_inserter(response.succeeded_waypoints),
                 [&response](const Waypoint::ConstPtr wp) {
                   return std::find(response.failed_waypoints.begin(), response.failed_waypoints.end(), wp) ==
                          response.failed_waypoints.end();
                 });

    response.status =
        tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorFailedToBuildGraph, status_category_);
    return response.status;
  }
  // No failed waypoints
  response.succeeded_waypoints = config_->waypoints;
  response.failed_waypoints.clear();

  // Search for edges
  std::vector<FloatType> solution;
  if (!graph_builder.search(solution))
  {
    CONSOLE_BRIDGE_logError("Search for graph completion failed");
    response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorFailedToFindValidSolution,
                                                   status_category_);
    return response.status;
  }

  response.joint_trajectory.joint_names = config_->joint_names;
  response.joint_trajectory.trajectory.resize(static_cast<long>(config_->waypoints.size()), static_cast<long>(dof));
  for (size_t r = 0; r < config_->waypoints.size(); ++r)
    for (size_t c = 0; c < dof; ++c)
      response.joint_trajectory.trajectory(static_cast<long>(r), static_cast<long>(c)) = solution[(r * dof) + c];

  // Check and report collisions
  std::vector<tesseract_collision::ContactResultMap> collisions;
  auto manager = config_->tesseract->getEnvironmentConst()->getContinuousContactManager();
  manager->setActiveCollisionObjects(config_->active_link_names);
  manager->setContactDistanceThreshold(0);
  collisions.clear();
  bool found = tesseract_environment::checkTrajectory(collisions,
                                                      *manager,
                                                      *config_->tesseract->getEnvironmentConst()->getStateSolver(),
                                                      response.joint_trajectory.joint_names,
                                                      response.joint_trajectory.trajectory,
                                                      tesseract_collision::ContactTestType::FIRST,
                                                      verbose);

  CONSOLE_BRIDGE_logInform("Descartes planning time: %.3f",
                           (boost::posix_time::second_clock::local_time() - tStart).seconds());

  if (found)
  {
    response.status = tesseract_common::StatusCode(
        DescartesMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision, status_category_);
    return response.status;
  }

  CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
  response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
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
  request_ = PlannerRequest();
  config_ = nullptr;
}

template <typename FloatType>
tesseract_common::StatusCode DescartesMotionPlanner<FloatType>::isConfigured() const
{
  if (config_ != nullptr)
    return tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::IsConfigured, status_category_);

  return tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorIsNotConfigured, status_category_);
}

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_HPP
