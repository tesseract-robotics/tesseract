/**
 * @file default_process_planners.h
 * @brief The default process planners provided by Tesseract
 *
 * @author Levi Armstrong
 * @date August 18, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_DEFAULT_PROCESS_PLANNERS_H
#define TESSERACT_PROCESS_MANAGERS_DEFAULT_PROCESS_PLANNERS_H

#include <tesseract_process_managers/core/taskflow_generator.h>

namespace tesseract_planning
{
/** @brief Create TrajOpt Process Pipeline */
TaskflowGenerator::UPtr createTrajOptGenerator();

/** @brief Create OMPL Process Pipeline */
TaskflowGenerator::UPtr createOMPLGenerator();

/** @brief Create Descartes Process Pipeline */
TaskflowGenerator::UPtr createDescartesGenerator();

/** @brief Create Cartesian Process Pipeline */
TaskflowGenerator::UPtr createCartesianGenerator();

/** @brief Create Freespace Process Pipeline */
TaskflowGenerator::UPtr createFreespaceGenerator();

/** @brief Create Raster Process Pipeline */
TaskflowGenerator::UPtr createRasterGenerator();

/** @brief Create Raster Only Process Pipeline */
TaskflowGenerator::UPtr createRasterOnlyGenerator();

/** @brief Create Raster Global Process Pipeline */
TaskflowGenerator::UPtr createRasterGlobalGenerator();

/** @brief Create Raster with Dual Transitions Process Pipeline */
TaskflowGenerator::UPtr createRasterDTGenerator();

/** @brief Create Raster with Approach and Departures Process Pipeline */
TaskflowGenerator::UPtr createRasterWAADGenerator();

/** @brief Create Raster with Approaches, Departures and Dual Transitions Process Pipeline */
TaskflowGenerator::UPtr createRasterWAADDTGenerator();

/** @brief Create Raster Only Global Process Pipeline */
TaskflowGenerator::UPtr createRasterOnlyGlobalGenerator();

/** @brief Create Raster with Cartesian Transitions Process Pipeline */
TaskflowGenerator::UPtr createRasterCTGenerator();

/** @brief Create Raster Only with Cartesian Transitions Process Pipeline */
TaskflowGenerator::UPtr createRasterOnlyCTGenerator();

/** @brief Create Raster with Cartesian Transitions and Dual Transitions Process Pipeline */
TaskflowGenerator::UPtr createRasterCTDTGenerator();

/** @brief Create Raster with Cartesian Transitions with Approaches and Departures Process Pipeline */
TaskflowGenerator::UPtr createRasterCTWAADGenerator();

/** @brief Create Raster with Cartesian Transitions with Approaches, Departures and Dual Transitions Process Pipeline */
TaskflowGenerator::UPtr createRasterCTWAADDTGenerator();

/** @brief Create Raster Global with Cartesian Transitions Process Pipeline */
TaskflowGenerator::UPtr createRasterGlobalCTGenerator();

/** @brief Create Raster Only Global with Cartesian Transitions Process Pipeline */
TaskflowGenerator::UPtr createRasterOnlyGlobalCTGenerator();
}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_DEFAULT_PROCESS_PLANNERS_H
