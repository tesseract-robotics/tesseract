/**
 * @file raster_only_global_taskflow.h
 * @brief Plans raster paths
 *
 * @author Matthew Powelson
 * @date July 15, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_RASTER_ONLY_GLOBAL_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_RASTER_ONLY_GLOBAL_TASKFLOW_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <vector>
#include <thread>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/taskflow_generator.h>

namespace tesseract_planning
{
/**
 * @brief This class provides a taskflow for a raster process.
 *
 * Given a TaskInput in the correct format, it handles the creation of the process dependencies and uses Taskflow to
 * execute them efficiently in a parallel based on those dependencies.
 *
 * The required format is below.
 * for skipping of rasters without replanning. This logic must be handled in the execute process.
 *
 * Composite
 * {
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 * }
 */
class RasterOnlyGlobalTaskflow : public TaskflowGenerator
{
public:
  using UPtr = std::unique_ptr<RasterOnlyGlobalTaskflow>;

  RasterOnlyGlobalTaskflow(TaskflowGenerator::UPtr global_taskflow_generator,
                           TaskflowGenerator::UPtr transition_taskflow_generator,
                           TaskflowGenerator::UPtr raster_taskflow_generator,
                           std::string name = "RasterOnlyGlobalTaskflow");

  ~RasterOnlyGlobalTaskflow() override = default;
  RasterOnlyGlobalTaskflow(const RasterOnlyGlobalTaskflow&) = delete;
  RasterOnlyGlobalTaskflow& operator=(const RasterOnlyGlobalTaskflow&) = delete;
  RasterOnlyGlobalTaskflow(RasterOnlyGlobalTaskflow&&) = delete;
  RasterOnlyGlobalTaskflow& operator=(RasterOnlyGlobalTaskflow&&) = delete;

  const std::string& getName() const override;

  TaskflowContainer generateTaskflow(TaskInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb) override;

private:
  TaskflowGenerator::UPtr global_taskflow_generator_;
  TaskflowGenerator::UPtr transition_taskflow_generator_;
  TaskflowGenerator::UPtr raster_taskflow_generator_;
  std::string name_;

  static void globalPostProcess(TaskInput input);

  /**
   * @brief Checks that the TaskInput is in the correct format.
   * @param input TaskInput to be checked
   * @return True if in the correct format
   */
  bool checkTaskInput(const TaskInput& input) const;
};

}  // namespace tesseract_planning

#endif
