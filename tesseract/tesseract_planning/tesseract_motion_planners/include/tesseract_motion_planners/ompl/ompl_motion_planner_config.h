/**
 * @file ompl_motion_planner_config.h
 * @brief Tesseract OMPL motion planner config.
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PLANNER_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PLANNER_CONFIG_H

#include <tesseract_motion_planners/ompl/ompl_problem.h>

namespace tesseract_planning
{
/** @brief The OMPLMotionPlannerConfig struct */
struct OMPLMotionPlannerConfig
{
  using Ptr = std::shared_ptr<OMPLMotionPlannerConfig>;
  using ConstPtr = std::shared_ptr<const OMPLMotionPlannerConfig>;

  /**
   * @brief Generates the OMPL problem and saves the result internally
   * @return True on success, false on failure
   */
  virtual bool generate();

  /** @brief The OMPL problem to solve */
  std::vector<OMPLProblem::UPtr> prob;

};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PLANNER_CONFIG_H
