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

#include <tesseract_environment/core/environment.h>
#include <tesseract_common/status_code.h>
#include <tesseract_common/types.h>
#include <tesseract_command_language/command_language.h>

namespace tesseract_planning
{
/**
 * This used to store planner specific profile mapping with the request
 *
 * For example say you have a profile named Raster in your command language. Say you have multiple Raster profiles
 * for descartes {Raster, Raster1, Rester2}. This allows you to remap the meaning of Raster in the command language to
 * say Raster2 for the specific planner Descartes by Map<Descartes, Map<Raster, Raster1>>.
 */
using PlannerProfileRemapping = std::unordered_map<std::string, std::unordered_map<std::string, std::string>>;

struct PlannerRequest
{
  std::string name;                                    /**< @brief The name of the process manager to use */
  tesseract_environment::Environment::ConstPtr env;    /**< @brief The environment */
  tesseract_environment::EnvState::ConstPtr env_state; /**< @brief The start state to use for planning */

  /**
   * @brief The program instruction
   * This must containt a minimum of two move instruction the first move instruction is the start state
   */
  CompositeInstruction instructions;
  /**
   * @brief This should be a one to one match with the instructions where the PlanInstruction is replaced with a
   * CompositeInstruction of MoveInstructions.
   */
  CompositeInstruction seed;

  /**
   * @brief This allows the remapping of the Plan Profile identified in the command language to a specific profile for a
   * given motion planner.
   */
  PlannerProfileRemapping plan_profile_remapping;

  /**
   * @brief This allows the remapping of the Composite Profile identified in the command language to a specific profile
   * for a given motion planner.
   */
  PlannerProfileRemapping composite_profile_remapping;

  /**
   * @brief data Planner specific data. For planners included in Tesseract_planning this is the planner problem that
   * will be used if it is not null
   */
  std::shared_ptr<void> data;
};

struct PlannerResponse
{
  CompositeInstruction results;
  tesseract_common::StatusCode status;                                     /**< @brief The status information */
  std::vector<std::reference_wrapper<Instruction>> succeeded_instructions; /**< @brief Waypoints for which the planner
                                                                        succeeded */
  std::vector<std::reference_wrapper<Instruction>> failed_instructions;    /**< @brief Waypoints for which the planner
                                                                              failed */
  /**
   * @brief data Planner specific data. For planners included in Tesseract_planning this is the planner problem that was
   * solved
   */
  std::shared_ptr<void> data;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PLANNING_PLANNER_TYPES_H
