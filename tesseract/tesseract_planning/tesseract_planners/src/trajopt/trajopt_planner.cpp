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
#include <jsoncpp/json/json.h>
#include <ros/console.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <tesseract_environment/core/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_planners/trajopt/trajopt_planner.h>

using namespace trajopt;

namespace tesseract_planners
{
bool TrajOptPlanner::solve(PlannerResponse& response)
{
  Json::Value root;
  Json::Reader reader;
  if (request_.config_format == "json")
  {
    bool parse_success = reader.parse(request_.config.c_str(), root);
    if (!parse_success)
    {
      ROS_FATAL("Failed to pass valid json file in the request");
      response.status_code = -2;
      response.status_description = status_code_map_[-2];
      return false;
    }
  }
  else
  {
    ROS_FATAL("Invalid config format: %s. Only json format is currently "
              "support for this planner.",
              request_.config_format.c_str());
    response.status_code = -1;
    response.status_description = status_code_map_[-1];
    return false;
  }

  TrajOptProbPtr prob = ConstructProblem(root, request_.tesseract);
  return solve(response, prob);
}

bool TrajOptPlanner::solve(PlannerResponse& response, const TrajOptPlannerConfig& config)
{
  // Create optimizer
  sco::BasicTrustRegionSQP opt(config.prob);
  opt.setParameters(config.params);
  opt.initialize(trajToDblVec(config.prob->GetInitTraj()));

  // Add all callbacks
  for (const sco::Optimizer::Callback& callback : config.callbacks)
  {
    opt.addCallback(callback);
  }

  // Optimize
  ros::Time tStart = ros::Time::now();
  opt.optimize();
  ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());

  // Check and report collisions
  std::vector<tesseract_collision::ContactResultMap> collisions;
  tesseract_collision::ContinuousContactManagerPtr continuous_manager = config.prob->GetEnv()->getContinuousContactManager();
  tesseract_environment::AdjacencyMapPtr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(config.prob->GetEnv()->getSceneGraph(),
                                                                                                               config.prob->GetKin()->getActiveLinkNames(),
                                                                                                               config.prob->GetEnv()->getCurrentState()->transforms);

  continuous_manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  continuous_manager->setContactDistanceThreshold(0);
  collisions.clear();
  bool found = checkTrajectory(*continuous_manager, *config.prob->GetEnv(), config.prob->GetKin()->getJointNames(), getTraj(opt.x(), config.prob->GetVars()), collisions);

  // Do a discrete check until continuous collision checking is updated to do dynamic-dynamic checking
  tesseract_collision::DiscreteContactManagerPtr discrete_manager = config.prob->GetEnv()->getDiscreteContactManager();
  discrete_manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  discrete_manager->setContactDistanceThreshold(0);
  collisions.clear();

  found = found || checkTrajectory(*discrete_manager, *config.prob->GetEnv(), config.prob->GetKin()->getJointNames(), getTraj(opt.x(), config.prob->GetVars()), collisions);

  // Send response
  response.trajectory = getTraj(opt.x(), config.prob->GetVars());
  response.joint_names = config.prob->GetKin()->getJointNames();
  if (opt.results().status == sco::OptStatus::OPT_PENALTY_ITERATION_LIMIT ||
      opt.results().status == sco::OptStatus::OPT_FAILED || opt.results().status == sco::OptStatus::INVALID)
  {
    response.status_code = -3;
    response.status_description = status_code_map_[-3] + ": " + sco::statusToString(opt.results().status);
  }
  else if (found)
  {
    response.status_code = -4;
    response.status_description = status_code_map_[-4];
  }
  else
  {
    ROS_INFO("Final trajectory is collision free");
    response.status_code = 0;
    response.status_description = status_code_map_[0] + ": " + sco::statusToString(opt.results().status);
  }

  return (response.status_code >= 0);
}

bool TrajOptPlanner::terminate() { return false; }
void TrajOptPlanner::clear() { request_ = PlannerRequest(); }
}  // namespace tesseract_planners
