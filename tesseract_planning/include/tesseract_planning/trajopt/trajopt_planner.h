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

#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_planning/basic_planner.h>
#include <tesseract_ros/ros_basic_plotting.h>

namespace tesseract
{
namespace tesseract_planning
{
struct TrajOptPlannerConfig
{
  TrajOptPlannerConfig(trajopt::TrajOptProbPtr prob) : prob(prob) {}
  virtual ~TrajOptPlannerConfig() {}
  /** @brief Trajopt problem to be solved (Required) */
  trajopt::TrajOptProbPtr prob;

  /** @brief Optimization parameters to be used (Optional) */
  sco::BasicTrustRegionSQPParameters params;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  std::vector<sco::Optimizer::Callback> callbacks;
};

class TrajOptPlanner : public BasicPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajOptPlanner()
  {
    name_ = "TRAJOPT";

    // Error Status Codes
    status_code_map_[-1] = "Invalid config data format";
    status_code_map_[-2] = "Failed to parse config data";
    status_code_map_[-3] = "";

    // Converge Status Codes
  }

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @return true if optimization complete
   */
  bool solve(PlannerResponse& response) override;

  /**
   * @brief Sets up the optimizer and solves the SQP problem with no callbacks and default parameters
   *
   * Note: This does not use the request information because everything is provided by config parameter
   *
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param config Trajopt planner config
   * @return true if optimization complete
   */
  bool solve(PlannerResponse& response, const TrajOptPlannerConfig& config);

  bool terminate() override;

  void clear() override;
};
}  // namespace tesseract_planning
}  // namespace tesseract
#endif  // TESSERACT_PLANNING_TRAJOPT_PLANNER_H
