/**
 * @file raster_dt_taskflow.h
 * @brief Plans raster paths with dual transitions
 *
 * @author Levi Armstrong
 * @date August 28, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_RASTER_DT_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_RASTER_DT_TASKFLOW_H

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
 * @brief This class provides a taskflow generator for a raster process.
 *
 * Given a TaskInput in the correct format, it handles the creation of the process dependencies and uses Taskflow to
 * execute them efficiently in a parallel based on those dependencies.
 *
 * The required format is below. Note that a transition is planned from both the start and end of each raster to allow
 * for skipping of rasters without replanning. This logic must be handled in the execute process.
 *
 * Composite
 * {
 *   Composite - from start
 *   Composite - Raster segment
 *   Unordered Composite - Transitions
 *   {
 *     Composite - Transition from end
 *     Composite - Transition to start
 *   }
 *   Composite - Raster segment
 *   Unordered Composite - Transitions
 *   {
 *     Composite - Transition from end
 *     Composite - Transition to start
 *   }
 *   Composite - Raster segment
 *   Composite - to end
 * }
 */
class RasterDTTaskflow : public TaskflowGenerator
{
public:
  using UPtr = std::unique_ptr<RasterDTTaskflow>;

  RasterDTTaskflow(TaskflowGenerator::UPtr freespace_taskflow_generator,
                   TaskflowGenerator::UPtr transition_taskflow_generator,
                   TaskflowGenerator::UPtr raster_taskflow_generator,
                   std::string name = "RasterDTTaskflow");
  ~RasterDTTaskflow() override = default;
  RasterDTTaskflow(const RasterDTTaskflow&) = delete;
  RasterDTTaskflow& operator=(const RasterDTTaskflow&) = delete;
  RasterDTTaskflow(RasterDTTaskflow&&) = delete;
  RasterDTTaskflow& operator=(RasterDTTaskflow&&) = delete;

  const std::string& getName() const override;

  TaskflowContainer generateTaskflow(TaskInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb) override;

private:
  TaskflowGenerator::UPtr freespace_taskflow_generator_;
  TaskflowGenerator::UPtr transition_taskflow_generator_;
  TaskflowGenerator::UPtr raster_taskflow_generator_;
  std::string name_;

  /**
   * @brief Checks that the TaskInput is in the correct format.
   * @param input TaskInput to be checked
   * @return True if in the correct format
   */
  bool checkTaskInput(const TaskInput& input) const;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_RASTER_DT_TASKFLOW_H
