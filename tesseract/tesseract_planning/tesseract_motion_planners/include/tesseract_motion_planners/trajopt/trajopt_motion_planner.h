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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>

namespace tesseract_motion_planners
{
class TrajOptMotionPlannerStatusCategory;

class TrajOptMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajOptMotionPlanner(std::string name = "TRAJOPT");

  ~TrajOptMotionPlanner() override = default;
  TrajOptMotionPlanner(const TrajOptMotionPlanner&) = default;
  TrajOptMotionPlanner& operator=(const TrajOptMotionPlanner&) = default;
  TrajOptMotionPlanner(TrajOptMotionPlanner&&) = default;
  TrajOptMotionPlanner& operator=(TrajOptMotionPlanner&&) = default;

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(TrajOptPlannerConfig::Ptr config);

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param check_type The type of collision check to perform on the planned trajectory
   * @param verbose Boolean indicating whether logging information about the motion planning solution should be printed
   * to console
   * @return true if optimization complete
   */
  tesseract_common::StatusCode solve(PlannerResponse& response,
                                     PostPlanCheckType check_type = PostPlanCheckType::DISCRETE_CONTINUOUS_COLLISION,
                                     bool verbose = false) override;

  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  tesseract_common::StatusCode isConfigured() const override;

protected:
  TrajOptPlannerConfig::Ptr config_;
  std::shared_ptr<const TrajOptMotionPlannerStatusCategory> status_category_; /** @brief The plannsers status codes */
};

class TrajOptMotionPlannerStatusCategory : public tesseract_common::StatusCategory
{
public:
  TrajOptMotionPlannerStatusCategory(std::string name);
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    IsConfigured = 1,
    SolutionFound = 0,
    IsNotConfigured = -1,
    FailedToParseConfig = -2,
    FailedToFindValidSolution = -3,
    FoundValidSolutionInCollision = -4
  };

private:
  std::string name_;
};

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_PLANNING_TRAJOPT_PLANNER_H
