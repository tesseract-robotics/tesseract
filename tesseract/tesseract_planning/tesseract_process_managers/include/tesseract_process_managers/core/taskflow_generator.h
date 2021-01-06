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

#include <tesseract_process_managers/core/types.h>
#include <tesseract_process_managers/core/task_input.h>
#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/taskflow_container.h>

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
   * @param done_cb A user defined callback
   * @param error_cb A user defined callback
   * @return A taskflow container which must stay around during execution of taskflow
   */
  virtual TaskflowContainer generateTaskflow(TaskInput instruction,
                                             TaskflowVoidFn done_cb,
                                             TaskflowVoidFn error_cb) = 0;

  //  /**
  //   * @brief Generate a series of task assigned to the provided taskflow but not connected
  //   * @details The task generated must be attached to other tasks in the taskflow outside this function.
  //   * Use only the input and outputs tasks for attaching.
  //   * @param taskflow The taskflow to use for generating new tasks
  //   * @param instruction Instructions passed to this instance of the taskflow
  //   * @param done_cb A user defined callback
  //   * @param error_cb A user defined callback
  //   * @return A taskflow container which must stay around during execution of taskflow
  //   */
  //  virtual TaskflowContainer generateTaskflow(tf::Taskflow& taskflow,
  //                                             TaskInput instruction,
  //                                             TaskflowVoidFn done_cb,
  //                                             TaskflowVoidFn error_cb) = 0;
};

}  // namespace tesseract_planning

#endif
