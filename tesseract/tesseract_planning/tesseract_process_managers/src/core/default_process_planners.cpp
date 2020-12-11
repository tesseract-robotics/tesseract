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

#include <tesseract_process_managers/core/default_process_planners.h>
#include <tesseract_process_managers/taskflow_generators/cartesian_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/descartes_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/freespace_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/ompl_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_dt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_dt_taskflow.h>

namespace tesseract_planning
{
TaskflowGenerator::UPtr createTrajOptGenerator()
{
  TrajOptTaskflowParams params;
  return std::make_unique<TrajOptTaskflow>(params);
}

TaskflowGenerator::UPtr createOMPLGenerator()
{
  OMPLTaskflowParams params;
  return std::make_unique<OMPLTaskflow>(params);
}

TaskflowGenerator::UPtr createDescartesGenerator()
{
  DescartesTaskflowParams params;
  return std::make_unique<DescartesTaskflow>(params);
}

TaskflowGenerator::UPtr createCartesianGenerator()
{
  CartesianTaskflowParams params;
  return std::make_unique<CartesianTaskflow>(params);
}

TaskflowGenerator::UPtr createFreespaceGenerator()
{
  FreespaceTaskflowParams params;
  return std::make_unique<FreespaceTaskflow>(params);
}

TaskflowGenerator::UPtr createRasterGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(fparams);
  TaskflowGenerator::UPtr transition_task = std::make_unique<FreespaceTaskflow>(fparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cparams);

  return std::make_unique<RasterTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterOnlyGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams tparams;
  TaskflowGenerator::UPtr transition_task = std::make_unique<FreespaceTaskflow>(tparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cparams);

  return std::make_unique<RasterOnlyTaskflow>(std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterGlobalGenerator()
{
  DescartesTaskflowParams global_params;
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  TaskflowGenerator::UPtr global_task = std::make_unique<DescartesTaskflow>(global_params);

  FreespaceTaskflowParams freespace_params;
  freespace_params.type = FreespaceTaskflowType::TRAJOPT_FIRST;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(freespace_params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<FreespaceTaskflow>(freespace_params);

  TrajOptTaskflowParams raster_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<TrajOptTaskflow>(raster_params);

  return std::make_unique<RasterGlobalTaskflow>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterDTGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(fparams);
  TaskflowGenerator::UPtr transition_task = std::make_unique<FreespaceTaskflow>(fparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cparams);

  return std::make_unique<RasterDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterWAADGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(fparams);
  TaskflowGenerator::UPtr transition_task = std::make_unique<FreespaceTaskflow>(fparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cparams);

  return std::make_unique<RasterWAADTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterWAADDTGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(fparams);
  TaskflowGenerator::UPtr transition_task = std::make_unique<FreespaceTaskflow>(fparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cparams);

  return std::make_unique<RasterWAADDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterOnlyGlobalGenerator()
{
  DescartesTaskflowParams global_params;
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  TaskflowGenerator::UPtr global_task = std::make_unique<DescartesTaskflow>(global_params);

  FreespaceTaskflowParams transition_params;
  transition_params.type = FreespaceTaskflowType::TRAJOPT_FIRST;
  TaskflowGenerator::UPtr transition_task = std::make_unique<FreespaceTaskflow>(transition_params);

  TrajOptTaskflowParams raster_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<TrajOptTaskflow>(raster_params);

  return std::make_unique<RasterOnlyGlobalTaskflow>(
      std::move(global_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterCTGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams freespace_params;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(freespace_params);

  // Create Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cartesian_params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<CartesianTaskflow>(cartesian_params);

  return std::make_unique<RasterTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterOnlyCTGenerator()
{
  // Create Transition and Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cartesian_params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<CartesianTaskflow>(cartesian_params);

  return std::make_unique<RasterOnlyTaskflow>(std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterCTDTGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams freespace_params;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(freespace_params);

  // Create Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cartesian_params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<CartesianTaskflow>(cartesian_params);

  return std::make_unique<RasterDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterCTWAADGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams freespace_params;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(freespace_params);

  // Create Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cartesian_params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<CartesianTaskflow>(cartesian_params);

  return std::make_unique<RasterWAADTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterCTWAADDTGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams freespace_params;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(freespace_params);

  // Create Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<CartesianTaskflow>(cartesian_params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<CartesianTaskflow>(cartesian_params);

  return std::make_unique<RasterWAADDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterGlobalCTGenerator()
{
  DescartesTaskflowParams global_params;
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  TaskflowGenerator::UPtr global_task = std::make_unique<DescartesTaskflow>(global_params);

  FreespaceTaskflowParams freespace_params;
  freespace_params.type = FreespaceTaskflowType::TRAJOPT_FIRST;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(freespace_params);

  TrajOptTaskflowParams raster_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<TrajOptTaskflow>(raster_params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<TrajOptTaskflow>(raster_params);

  return std::make_unique<RasterGlobalTaskflow>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterOnlyGlobalCTGenerator()
{
  DescartesTaskflowParams global_params;
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  TaskflowGenerator::UPtr global_task = std::make_unique<DescartesTaskflow>(global_params);

  TrajOptTaskflowParams raster_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<TrajOptTaskflow>(raster_params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<TrajOptTaskflow>(raster_params);

  return std::make_unique<RasterOnlyGlobalTaskflow>(
      std::move(global_task), std::move(transition_task), std::move(raster_task));
}
}  // namespace tesseract_planning
