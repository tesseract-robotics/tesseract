/**
 * @file trajopt_planner.h
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
#ifndef TESSERACT_PLANNING_TRAJOPT_PLANNER_H
#define TESSERACT_PLANNING_TRAJOPT_PLANNER_H

#include <tesseract_planning/basic_planner.h>
#include <trajopt/problem_description.hpp>
#include <tesseract_ros/ros_basic_plotting.h>

namespace tesseract
{
namespace tesseract_planning
{
class TrajoptPlanner : public BasicPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajoptPlanner()
  {
    name_ = "TRAJOPT";

    // Error Status Codes
    status_code_map_[-1] = "Invalid config data format";
    status_code_map_[-2] = "Failed to parse config data";
    status_code_map_[-3] = "";

    // Converge Status Codes
  }

  bool solve(PlannerResponse& response) override;

  bool solve(trajopt::TrajOptProbPtr prob, PlannerResponse& response);

  bool solve(trajopt::TrajOptProbPtr prob, PlannerResponse& response, trajopt::BasicTrustRegionSQPParameters& params);

  bool solve(trajopt::TrajOptProbPtr prob,
             PlannerResponse& response,
             trajopt::BasicTrustRegionSQPParameters& params,
             trajopt::Optimizer::Callback& callback);

  bool solve(trajopt::TrajOptProbPtr prob, PlannerResponse& response, trajopt::Optimizer::Callback& callback);

  bool solve(trajopt::TrajOptProbPtr prob,
             PlannerResponse& response,
             std::vector<trajopt::Optimizer::Callback>& callbacks);

  /**
   * @brief solve - Sets up optimizer and solves a SQP problem
   * @param prob - trajopt problem to be solved
   * @param response - the results of the optimization. Primary output is the optimized joint trajectory
   * @param params - SQP parameters for the optimization. If not given, defaults will be used
   * @param callbacks - Callbacks to be called on each iteration, e.g. plotting, write to file, etc.
   * @return
   */
  bool solve(trajopt::TrajOptProbPtr prob,
             PlannerResponse& response,
             trajopt::BasicTrustRegionSQPParameters& params,
             std::vector<trajopt::Optimizer::Callback>& callbacks);

  bool terminate() override;

  void clear() override;
};
}  // namespace tesseract_planning
}  // namespace tesseract
#endif  // TESSERACT_PLANNING_TRAJOPT_PLANNER_H
