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

using namespace trajopt;

namespace tesseract_motion_planners
{
TrajOptMotionPlanner::TrajOptMotionPlanner(const std::string& name ):
    config_(nullptr)
{
  name_ = name;

  // Success Status Codes
  status_code_map_[0] = "Found valid solution";

  // TODO: These should be tied to enumeration ints and returned through an getLastErrorMsg() method
  // Error Status Codes
  status_code_map_[-1] = "Invalid config data format";
  status_code_map_[-2] = "Failed to parse config data";
  status_code_map_[-3] = "Failed to find valid solution";
  status_code_map_[-4] = "Found valid solution, but is in collision";
}

bool TrajOptMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

void TrajOptMotionPlanner::clear()
{
  request_ = PlannerRequest();
  config_ = nullptr;
}

bool TrajOptMotionPlanner::solve(PlannerResponse& response)
{
  if(isConfigured())
  {
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return false;
  }

  // Create optimizer
  sco::BasicTrustRegionSQP opt(config_->prob);
  opt.setParameters(config_->params);
  opt.initialize(trajToDblVec(config_->prob->GetInitTraj()));

  // Add all callbacks
  for (const sco::Optimizer::Callback& callback : config_->callbacks)
  {
    opt.addCallback(callback);
  }

  // Optimize
  auto tStart = boost::posix_time::second_clock::local_time();
  opt.optimize();
  CONSOLE_BRIDGE_logInform("planning time: %.3f", (boost::posix_time::second_clock::local_time() - tStart).seconds());

  // Check and report collisions
  std::vector<tesseract_collision::ContactResultMap> collisions;
  tesseract_collision::ContinuousContactManagerPtr continuous_manager = config_->prob->GetEnv()->getContinuousContactManager();
  tesseract_environment::AdjacencyMapPtr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      config_->prob->GetEnv()->getSceneGraph(),
      config_->prob->GetKin()->getActiveLinkNames(),
      config_->prob->GetEnv()->getCurrentState()->transforms);

  continuous_manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  continuous_manager->setContactDistanceThreshold(0);
  collisions.clear();
  bool found = checkTrajectory(*continuous_manager, *config_->prob->GetEnv(),
                               config_->prob->GetKin()->getJointNames(),
                               getTraj(opt.x(), config_->prob->GetVars()), collisions);

  // Do a discrete check until continuous collision checking is updated to do dynamic-dynamic checking
  tesseract_collision::DiscreteContactManagerPtr discrete_manager = config_->prob->GetEnv()->getDiscreteContactManager();
  discrete_manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  discrete_manager->setContactDistanceThreshold(0);
  collisions.clear();

  found = found || checkTrajectory(*discrete_manager, *config_->prob->GetEnv(),
                                   config_->prob->GetKin()->getJointNames(),
                                   getTraj(opt.x(), config_->prob->GetVars()),
                                   collisions);

  // Send response
  response.trajectory = getTraj(opt.x(), config_->prob->GetVars());
  response.joint_names = config_->prob->GetKin()->getJointNames();
  if (opt.results().status == sco::OptStatus::OPT_PENALTY_ITERATION_LIMIT ||
      opt.results().status == sco::OptStatus::OPT_FAILED || opt.results().status == sco::OptStatus::INVALID)
  {
    response.status_code = -3;
    response.status_description = status_code_map_.at(-3) + ": " + sco::statusToString(opt.results().status);
  }
  else if (found)
  {
    response.status_code = -4;
    response.status_description = status_code_map_.at(-4);
  }
  else
  {
    CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
    response.status_code = 0;
    response.status_description = status_code_map_.at(0) + ": " + sco::statusToString(opt.results().status);
  }

  return (response.status_code >= 0);
}

bool TrajOptMotionPlanner::isConfigured() const
{
  return config_ != nullptr;
}

bool TrajOptMotionPlanner::setConfiguration(const TrajOptPlannerConfig& config)
{
  config_ = std::make_shared<TrajOptPlannerConfig>(config);
  return true;
}

} // namespace tesseract_motion_planners
