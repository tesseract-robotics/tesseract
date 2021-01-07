/**
 * @file taskflow_container.h
 * @brief Taskflow container is used to contain everything that must stay around until it finishes.
 *
 * @author Levi Armstrong
 * @date Decemebr 15, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_TASKFLOW_CONTAINER_H
#define TESSERACT_PROCESS_MANAGERS_TASKFLOW_CONTAINER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <taskflow/taskflow.hpp>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>

namespace tesseract_planning
{
/**
 * @brief Taskflow Container
 *
 * @details Taskflow composition does not allow connecting to tasks within another taskflow.
 * This is to support similar functionality where all tasks are added to the highest level taskflow
 * and stores all tasks from a generator and the identified input and output tasks for composition
 *
 *  - Input Task: input
 *  - Output Tasks:
 *    - On Error: outputs[0]
 *    - On Success: outputs[1]
 */
struct TaskflowContainer
{
  /**
   * @brief If not a nullptr this should be added using taskflow composition.
   * @details If a nullptr then input and outputs may be used.
   */
  std::unique_ptr<tf::Taskflow> taskflow;

  /**
   * @brief The input task associated with a taskflow
   * @warning If taskflow is not a nullptr, then input is associated with the above taskflow and connections cannot span
   * multiple taskflows
   */
  tf::Task input;

  /**
   * @brief The output task associated with a taskflow
   * @warning If taskflow is not a nullptr, then input is associated with the above taskflow and connections cannot span
   * multiple taskflows
   */
  std::vector<tf::Task> outputs;

  /**
   * @brief TaskflowContainer's associated with the taskflow
   * @details This must stay around during execution of taskflow
   */
  std::vector<TaskflowContainer> containers;

  /**
   * @brief TaskGenerator's associated with the taskflow
   * @details This must stay around during execution of taskflow
   */
  std::vector<TaskGenerator::UPtr> generators;

  /** @brief Clear the Taskflow Container */
  void clear();
};
}  // namespace tesseract_planning
#endif  // TESSERACT_PROCESS_MANAGERS_TASKFLOW_CONTAINER_H
