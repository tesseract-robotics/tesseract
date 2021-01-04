/**
 * @file planner.h
 * @brief Planner Interface Class.
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
#ifndef TESSERACT_MOTION_PLANNERS_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/status_code.h>
#include <tesseract_motion_planners/core/types.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::MotionPlanner)
#endif  // SWIG

namespace tesseract_planning
{
class MotionPlanner
{
public:
  using Ptr = std::shared_ptr<MotionPlanner>;
  using ConstPtr = std::shared_ptr<const MotionPlanner>;
  /** @brief Construct a basic planner */
  MotionPlanner() = default;
  virtual ~MotionPlanner() = default;
  MotionPlanner(const MotionPlanner&) = delete;
  MotionPlanner& operator=(const MotionPlanner&) = delete;
  MotionPlanner(MotionPlanner&&) = delete;
  MotionPlanner& operator=(MotionPlanner&&) = delete;

  /** @brief Get the name of this planner */
  virtual const std::string& getName() const = 0;

  /**
   * @brief Solve the planner request problem
   * @param request The planning request
   * @param response The results from the planner
   * @param check_type The type of validation check to be performed on the planned trajectory
   * @param verbose Flag for printing more detailed planning information
   * @return A code indicating the status of the planned trajectory
   */
  virtual tesseract_common::StatusCode solve(const PlannerRequest& request,
                                             PlannerResponse& response,
                                             bool verbose = false) const = 0;

  /**
   * @brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if
   * solve() is not running (returns true).
   */
  virtual bool terminate() = 0;

  /** @brief Clear the data structures used by the planner */
  virtual void clear() = 0;

  /** @brief Clone the motion planner */
  virtual MotionPlanner::Ptr clone() const = 0;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_PLANNING_PLANNER_H
