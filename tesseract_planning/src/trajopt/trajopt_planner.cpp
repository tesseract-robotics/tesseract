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
#include <jsoncpp/json/json.h>
#include <ros/console.h>
#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;

namespace tesseract
{
namespace tesseract_planning
{
bool TrajoptPlanner::solve(PlannerResponse& response)
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

  TrajOptProbPtr prob = ConstructProblem(root, request_.env);
  return solve(prob, response);
}

bool TrajoptPlanner::solve(trajopt::TrajOptProbPtr prob, PlannerResponse& response)
{
  BasicTrustRegionSQP opt(prob);
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  ros::Time tStart = ros::Time::now();
  opt.optimize();
  ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());

  std::vector<tesseract::ContactResultMap> collisions;
  ContinuousContactManagerBasePtr manager = prob->GetEnv()->getContinuousContactManager();

  collisions.clear();
  bool found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), getTraj(opt.x(), prob->GetVars()), collisions);

  if (found)
  {
    ROS_INFO("Final trajectory is in collision");
  }
  else
  {
    ROS_INFO("Final trajectory is collision free");
  }

  response.trajectory = getTraj(opt.x(), prob->GetVars());
  response.status_code = opt.results().status;
  response.joint_names = prob->GetEnv()->getJointNames();
  response.status_description = sco::statusToString(opt.results().status);
  return true;
}

bool TrajoptPlanner::terminate() { return false; }
void TrajoptPlanner::clear() { request_ = PlannerRequest(); }

}  // namespace tesseract_planning
}  // namespace tesseract
