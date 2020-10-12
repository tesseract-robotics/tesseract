/**
 * @file taskflow_generator.h
 * @brief Taskflow generator
 *
 * @author Matthew Powelson
 * @date July 15. 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_TASKFLOW_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_TASKFLOW_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_input.h>

// Forward Declare
namespace tf
{
class Taskflow;
}

namespace tesseract_planning
{
/** @brief Base class for generating a taskflow */
class TaskflowGenerator
{
public:
  using UPtr = std::unique_ptr<TaskflowGenerator>;

  TaskflowGenerator() = default;
  virtual ~TaskflowGenerator() = default;
  TaskflowGenerator(const TaskflowGenerator&) = delete;
  TaskflowGenerator& operator=(const TaskflowGenerator&) = delete;
  TaskflowGenerator(TaskflowGenerator&&) = delete;
  TaskflowGenerator& operator=(TaskflowGenerator&&) = delete;

  /**
   * @brief Get the name of the task flow
   * @return The name
   */
  virtual const std::string& getName() const = 0;

  /**
   * @brief Generates unique taskflows for using as a component (e.g. with taskflow.composed_of)
   * @param instruction Instructions passed to this instance of the taskflow
   * @return Reference to taskflow. TaskflowGenerator should maintain ownership
   */
  virtual tf::Taskflow& generateTaskflow(ProcessInput instruction,
                                         std::function<void()> done_cb,
                                         std::function<void()> error_cb) = 0;

  /** @brief Abort the active taskflow */
  virtual void abort() = 0;

  /** @brief Reset to initial state, but does not clear stored task */
  virtual void reset() = 0;

  /** @brief Clear all stored task and calls reset() */
  virtual void clear() = 0;
};

}  // namespace tesseract_planning

#endif
