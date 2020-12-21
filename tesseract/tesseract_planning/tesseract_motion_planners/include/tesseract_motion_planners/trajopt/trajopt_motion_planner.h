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
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::TrajOptMotionPlanner)
%shared_ptr(tesseract_planning::TrajOptMotionPlannerStatusCategory)
#endif  // SWIG

namespace tesseract_planning
{
class TrajOptMotionPlannerStatusCategory;

using TrajOptProblemGeneratorFn =
    std::function<std::shared_ptr<trajopt::ProblemConstructionInfo>(const std::string&,
                                                                    const PlannerRequest&,
                                                                    const TrajOptPlanProfileMap&,
                                                                    const TrajOptCompositeProfileMap&,
                                                                    const TrajOptSolverProfileMap&)>;

class TrajOptMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajOptMotionPlanner();

  ~TrajOptMotionPlanner() override = default;
  TrajOptMotionPlanner(const TrajOptMotionPlanner&) = delete;
  TrajOptMotionPlanner& operator=(const TrajOptMotionPlanner&) = delete;
  TrajOptMotionPlanner(TrajOptMotionPlanner&&) = delete;
  TrajOptMotionPlanner& operator=(TrajOptMotionPlanner&&) = delete;

  const std::string& getName() const override;

  TrajOptProblemGeneratorFn problem_generator;

  /**
   * @brief The available solver profiles
   * @details This is used to look up solver parameters by the planner
   */
  TrajOptSolverProfileMap solver_profiles;

  /**
   * @brief The available composite profiles
   * @details Composite instruction is a way to namespace or organize your planning problem. The composite instruction
   * has a profile which is used for applying multy waypoint costs and constraints like joint smoothing, collision
   * avoidance, and velocity smoothing.
   */
  TrajOptCompositeProfileMap composite_profiles;

  /**
   * @brief The available plan profiles
   * @details Plan instruction profiles are used to control waypoint specific information like fixed waypoint,
   * toleranced waypoint, corner distance waypoint, etc.
   */
  TrajOptPlanProfileMap plan_profiles;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  std::vector<sco::Optimizer::Callback> callbacks;

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param check_type The type of collision check to perform on the planned trajectory
   * @param verbose Boolean indicating whether logging information about the motion planning solution should be printed
   * to console
   * @return true if optimization complete
   */
  tesseract_common::StatusCode solve(const PlannerRequest& request,
                                     PlannerResponse& response,
                                     bool verbose = false) const override;

  bool checkUserInput(const PlannerRequest& request) const;

  bool terminate() override;

  void clear() override;

  MotionPlanner::Ptr clone() const override;

protected:
#ifndef SWIG
  std::string name_{ "TRAJOPT" };
  std::shared_ptr<const TrajOptMotionPlannerStatusCategory> status_category_; /** @brief The planners status codes */
#endif
};

class TrajOptMotionPlannerStatusCategory : public tesseract_common::StatusCategory
{
public:
  TrajOptMotionPlannerStatusCategory(std::string name);
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    SolutionFound = 0,
    ErrorInvalidInput = -1,
    FailedToFindValidSolution = -3,
  };

private:
  std::string name_;
};

}  // namespace tesseract_planning
#endif  // TESSERACT_PLANNING_TRAJOPT_PLANNER_H
