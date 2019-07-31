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
#include <jsoncpp/json/json.h>
#include <ros/console.h>
#include <descartes_light/descartes_light.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_samplers/samplers/railed_cartesian_point_sampler.h>
#include <descartes_samplers/samplers/railed_axial_symmetric_sampler.h>
#include <descartes_samplers/samplers/axial_symmetric_sampler.h>
#include <descartes_samplers/samplers/cartesian_point_sampler.h>
#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>
#include <descartes_samplers/evaluators/distance_edge_evaluator.h>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>

namespace tesseract_motion_planners
{
DescartesMotionPlannerStatusCategory::DescartesMotionPlannerStatusCategory(std::string name) : name_(name) {}
const std::string& DescartesMotionPlannerStatusCategory::name() const noexcept { return name_; }
std::string DescartesMotionPlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case IsConfigured:
    {
      return "Is Configured";
    }
    case SolutionFound:
    {
      return "Found valid solution";
    }
    case IsNotConfigured:
    {
      return "Planner is not configured, must call setConfiguration prior to calling solve.";
    }
    case FailedToParseConfig:
    {
      return "Failed to parse config data";
    }
    case FailedToBuildGraph:
    {
      return "Failed to build graph";
    }
    case FailedToFindValidSolution:
    {
      return "Failed to search graph";
    }
    case FoundValidSolutionInCollision:
    {
      return "Found valid solution, but is in collision";
    }
    default:
    {
      assert (false);
      return "";
    }
  }
}


template<typename FloatType>
DescartesMotionPlanner<FloatType>::DescartesMotionPlanner(std::string name) : MotionPlanner(name), config_(nullptr), status_category_(std::make_shared<const DescartesMotionPlannerStatusCategory>(name))
{
}

template<typename FloatType>
bool DescartesMotionPlanner<FloatType>::setConfiguration(const DescartesMotionPlannerConfig<FloatType> &config)
{
  // Check that parameters are valid
  if (config.tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In %s: tesseract is a required parameter and has not been set", name_.c_str());
    return false;
  }

  if (config.contact_checker == nullptr)
  {
    CONSOLE_BRIDGE_logError("In %s: contact_checker is a required parameter and has not been set", name_.c_str());
    return false;
  }

  if (config.kinematics == nullptr)
  {
    CONSOLE_BRIDGE_logError("In %s: kinematics is a required parameter and has not been set", name_.c_str());
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

  if ((config.timing_constraint.size() != config.waypoints.size()) || (config.timing_constraint.size() != config.samplers.size()))
  {
    CONSOLE_BRIDGE_logError("In %s: waypoints, timing_constraint and samplers must be the same size", name_.c_str());
    return false;
  }

  config_ = std::make_shared<DescartesMotionPlannerConfig<FloatType>>(config);
  return true;
}

template<typename FloatType>
tesseract_common::StatusCode DescartesMotionPlanner<FloatType>::solve(PlannerResponse& response)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  ros::Time tStart = ros::Time::now();

  const auto dof = config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator)->numJoints();
  descartes_light::Solver<FloatType> graph_builder(dof);
  if (!graph_builder.build(config_->samplers, config_->timing_constraint, config_->edge_evaluator))
  {
    CONSOLE_BRIDGE_logError("Failed to build vertices");
    for (const auto& i : graph_builder.getFailedVertices())
    {
      const Waypoint::Ptr& wp = config_->waypoints[i];
      if (wp->getType() == WaypointType::CARTESIAN_WAYPOINT)
      {
        CartesianWaypoint::ConstPtr cwp = std::static_pointer_cast<const CartesianWaypoint>(wp);
        config_->kinematics->analyzeIK(cwp->getTransform().cast<FloatType>());
      }
    }

    response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::FailedToBuildGraph, status_category_);
    return response.status;
  }

  // Search for edges
  std::vector<FloatType> solution;
  if (!graph_builder.search(solution))
  {
    CONSOLE_BRIDGE_logError("Search for graph completion failed");
    response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::FailedToFindValidSolution, status_category_);
    return response.status;
  }

  response.joint_trajectory.joint_names = config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator)->getJointNames();
  response.joint_trajectory.trajectory.resize(static_cast<long>(config_->waypoints.size()), dof);
  for (size_t r = 0; r < config_->waypoints.size(); ++r)
    for (size_t c = 0; c < dof; ++c)
      response.joint_trajectory.trajectory(static_cast<long>(r), static_cast<long>(c)) = solution[(r * dof) + c];

  // Check and report collisions
  std::vector<tesseract_collision::ContactResultMap> collisions;
  tesseract_collision::ContinuousContactManager::Ptr manager = config_->tesseract->getEnvironmentConst()->getContinuousContactManager();
  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(config_->tesseract->getEnvironmentConst()->getSceneGraph(),
                                                                                        config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator)->getLinkNames(),
                                                                                        config_->tesseract->getEnvironmentConst()->getCurrentState()->transforms);
  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);
  collisions.clear();
  bool found = tesseract_environment::checkTrajectory(*manager, *config_->tesseract->getEnvironmentConst(), response.joint_trajectory.joint_names, response.joint_trajectory.trajectory, collisions);

  CONSOLE_BRIDGE_logInform("Descartes planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (found)
  {
    response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::FoundValidSolutionInCollision, status_category_);
    return response.status;
  }

  CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
  response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

template<typename FloatType>
bool DescartesMotionPlanner<FloatType>::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

template<typename FloatType>
void DescartesMotionPlanner<FloatType>::clear()
{
  request_ = PlannerRequest();
  config_ = nullptr;
}

template<typename FloatType>
tesseract_common::StatusCode DescartesMotionPlanner<FloatType>::isConfigured() const
{
  if (config_ != nullptr)
    return tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::IsConfigured, status_category_);
  else
    return tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::IsNotConfigured, status_category_);
}

}
#endif // TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_HPP
