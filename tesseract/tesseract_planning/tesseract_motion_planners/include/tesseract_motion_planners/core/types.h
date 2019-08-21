/**
 * @file planner_types.h
 * @brief Planner types.
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
#ifndef TESSERACT_MOTION_PLANNERS_PLANNER_TYPES_H
#define TESSERACT_MOTION_PLANNERS_PLANNER_TYPES_H

#include <tesseract/tesseract.h>
#include <tesseract_common/status_code.h>
#include <tesseract_common/types.h>
#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_motion_planners
{
struct PlannerRequest
{
  std::string name;                                      /**< @brief The name of the planner to use */
  tesseract::Tesseract::ConstPtr tesseract;              /**< @brief Tesseract */
  tesseract_environment::EnvState::ConstPtr start_state; /**< @brief The start state to use for planning */
  std::string config;                                    /**< @brief The configuration to use (json file) */
  std::string config_format;                             /**< @brief The file extension used to parse config */
};

struct PlannerResponse
{
  tesseract_common::JointTrajectory joint_trajectory; /**< @brief The joint trajectory */
  tesseract_common::StatusCode status;                /**< @brief The status information */
  std::vector<Waypoint::Ptr> succeeded_waypoints;     /**< @brief Waypoints for which the planner succeeded */
  std::vector<Waypoint::Ptr> failed_waypoints;        /**< @brief Waypoints for which the planner failed */
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_PLANNING_PLANNER_TYPES_H
