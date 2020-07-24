/**
 * @file default_raster_processes.h
 * @brief Default processes for the raster process manager
 *
 * @author Matthew Powelson
 * @date July 16. 2020
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
#ifndef TESSERACT_PROCESS_MANAGER_DEFAULT_RASTER_PROCESSES_H
#define TESSERACT_PROCESS_MANAGER_DEFAULT_RASTER_PROCESSES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_process_managers/process_generators/random_process_generator.h>
#include <tesseract_process_managers/process_generators/validators/random_validator.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>

namespace tesseract_planning
{
inline std::vector<ProcessGenerator::Ptr> defaultRasterProcesses()
{
  // Setup Interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");
  interpolator->plan_profiles["FREESPACE"] =
      std::make_shared<SimplePlannerDefaultPlanProfile>();  // TODO: switch this for interpolator plan profile once the
                                                            // step generators have been implemented
  auto interpolator_generator = std::make_shared<MotionPlannerProcessGenerator>(interpolator);
  interpolator_generator->validators.emplace_back(&randomValidator);

  // Setup processes
  auto descartes_planner = std::make_shared<DescartesMotionPlanner<double>>();
  descartes_planner->problem_generator = &DefaultDescartesProblemGenerator<double>;
  descartes_planner->plan_profiles["RASTER"] = std::make_shared<DescartesDefaultPlanProfileD>();

  auto descartes = std::make_shared<MotionPlannerProcessGenerator>(descartes_planner);
  descartes->planner = descartes_planner;

  return std::vector<ProcessGenerator::Ptr>{ interpolator_generator, descartes };
}

}  // namespace tesseract_planning

#endif
