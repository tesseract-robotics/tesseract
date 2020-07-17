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

#include <tesseract_process_managers/process_generators/random_process_generator.h>
#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>

namespace tesseract_planning
{
inline std::vector<ProcessGenerator::Ptr> defaultRasterProcesses()
{
  // Setup processes
  auto descartes = std::make_shared<RandomProcessGenerator>();
  descartes->name = "descartes";

  return std::vector<ProcessGenerator::Ptr>{ descartes };
}

}  // namespace tesseract_planning

#endif
