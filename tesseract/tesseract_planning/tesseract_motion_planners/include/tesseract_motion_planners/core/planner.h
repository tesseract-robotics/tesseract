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

namespace tesseract_motion_planners
{
class MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  MotionPlanner(std::string name) : name_(std::move(name)) {}
  virtual ~MotionPlanner() = default;
  MotionPlanner(const MotionPlanner&) = default;
  MotionPlanner& operator=(const MotionPlanner&) = default;
  MotionPlanner(MotionPlanner&&) = default;
  MotionPlanner& operator=(MotionPlanner&&) = default;

  /** @brief Get the name of this planner */
  const std::string& getName() const { return name_; }
  /** @brief Get the planner request */
  const PlannerRequest& getRequest() const { return request_; }
  /** @brief Set the planner request for this context */
  void setRequest(const PlannerRequest& request) { request_ = request; }

  /** @brief Solve the planner request problem */
  virtual tesseract_common::StatusCode solve(PlannerResponse& res, bool verbose = false) = 0;

  /**
   * @brief checks if the planner is configured for planning
   * @return True if configured, false otherwise
   */
  virtual tesseract_common::StatusCode isConfigured() const = 0;

  /**
   * @brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if
   * solve() is not running (returns true).
   */
  virtual bool terminate() = 0;

  /** @brief Clear the data structures used by the planner */
  virtual void clear() = 0;

protected:
  std::string name_;       /**< @brief The name of this planner */
  PlannerRequest request_; /**< @brief The planner request information */
};
}  // namespace tesseract_motion_planners
#endif  // TESSERACT_PLANNING_PLANNER_H
