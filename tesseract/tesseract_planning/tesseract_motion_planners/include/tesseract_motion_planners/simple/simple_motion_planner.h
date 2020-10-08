/**
 * @file simple_motion_planner.h
 * @brief The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all
 * of the PlanInstructions and then calls the appropriate function from the profile. These functions do not depend on
 * the seed, so this may be used to initialize the seed appropriately using e.g. linear interpolation.
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/visibility_control.h>

namespace tesseract_planning
{
class SimpleMotionPlannerStatusCategory;

/**
 * @brief The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all
 * of the PlanInstructions and then calls the appropriate function from the profile. These functions do not depend on
 * the seed, so this may be used to initialize the seed appropriately using e.g. linear interpolation.
 */
class TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC SimpleMotionPlanner : public MotionPlanner
{
public:
  using Ptr = std::shared_ptr<SimpleMotionPlanner>;
  using ConstPtr = std::shared_ptr<const SimpleMotionPlanner>;

  /** @brief Construct a basic planner */
  SimpleMotionPlanner(std::string name = "SIMPLE_PLANNER");

  ~SimpleMotionPlanner() override = default;
  SimpleMotionPlanner(const SimpleMotionPlanner&) = default;
  SimpleMotionPlanner& operator=(const SimpleMotionPlanner&) = default;
  SimpleMotionPlanner(SimpleMotionPlanner&&) = default;
  SimpleMotionPlanner& operator=(SimpleMotionPlanner&&) = default;

  /**
   * @brief The available composite profiles
   *
   * Composite instruction is a way to namespace or organize your planning problem. The composite instruction has a
   * profile which is used for applying multy waypoint costs and constraints like joint smoothing, collision avoidance,
   * and velocity smoothing.
   */
  SimplePlannerCompositeProfileMap composite_profiles;

  /**
   * @brief The available plan profiles
   *
   * Plan instruction profiles are used to control waypoint specific information like fixed waypoint, toleranced
   * waypoint, corner distance waypoint, etc.
   */
  SimplePlannerPlanProfileMap plan_profiles;

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

protected:
  std::shared_ptr<const SimpleMotionPlannerStatusCategory> status_category_; /** @brief The planners status codes */

  MoveInstruction getStartInstruction(const PlannerRequest& request,
                                      const tesseract_environment::EnvState::ConstPtr& current_state,
                                      const tesseract_kinematics::ForwardKinematics::Ptr& fwd_kin) const;

  CompositeInstruction processCompositeInstruction(const CompositeInstruction& instructions,
                                                   const Waypoint& initial_start_waypoint,
                                                   const PlannerRequest& request) const;
};

class TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC SimpleMotionPlannerStatusCategory
  : public tesseract_common::StatusCategory
{
public:
  SimpleMotionPlannerStatusCategory(std::string name);
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
#endif  // TESSERACT_PLANNING_SIMPLE_PLANNER_H
