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
#include <vector>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/process_input.h>
#include <tesseract_process_managers/core/process_generator.h>

// Forward Declare
namespace tf
{
class Taskflow;
}

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
   * @brief ProcessGenerator's associated with the taskflow
   * @details This must stay around during execution of taskflow
   */
  std::vector<ProcessGenerator::UPtr> generators;

  /** @brief Clear the Taskflow Container */
  void clear()
  {
    taskflow->clear();
    taskflow = nullptr;
    outputs.clear();

    for (auto& c : containers)
      c.clear();

    containers.clear();
    generators.clear();
  }
};

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
  virtual TaskflowContainer generateTaskflow(ProcessInput instruction,
                                             std::function<void()> done_cb,
                                             std::function<void()> error_cb) = 0;

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
  //                                             ProcessInput instruction,
  //                                             std::function<void()> done_cb,
  //                                             std::function<void()> error_cb) = 0;
};

/**
 * @brief The default success task to be used
 * @param name The name
 * @param message A detailed message
 * @param user_callback A user callback function
 */
inline void successTask(const ProcessInput& /*instruction*/,
                        const std::string& name,
                        const std::string& message,
                        const std::function<void()>& user_callback = nullptr)
{
  CONSOLE_BRIDGE_logInform("%s Successful: %s", name.c_str(), message.c_str());
  if (user_callback)
    user_callback();
}

/**
 * @brief The default failure task to be used
 * @details This will call the abort function of the ProcessInput provided
 * @param name The name
 * @param message A detailed message
 * @param user_callback A user callback function
 */
inline void failureTask(ProcessInput instruction,
                        const std::string& name,
                        const std::string& message,
                        const std::function<void()>& user_callback = nullptr)
{
  // Call abort on the process input
  instruction.abort();

  // Print an error if this is the first failure
  CONSOLE_BRIDGE_logError("%s Failure: %s", name.c_str(), message.c_str());
  if (user_callback)
    user_callback();
}

/**
 * @brief Check if composite is empty along with children composites
 * @param composite The composite to check
 * @return True if empty otherwise false
 */
inline bool isCompositeEmpty(const CompositeInstruction& composite)
{
  if (composite.empty())
    return true;

  for (const auto& i : composite)
  {
    if (isCompositeInstruction(i))
    {
      const auto* sub_composite = i.cast_const<CompositeInstruction>();
      if (isCompositeEmpty(*sub_composite))
        return true;
    }
  }

  return false;
}

/**
 * @brief Check if the input has a seed
 * @details It checks if a any composite instruction is empty in the results data structure
 * @param input The process input
 * @return One if seed exists, otherwise zero
 */
inline int hasSeedTask(ProcessInput input)
{
  if (input.has_seed)
    return 1;

  assert(isCompositeInstruction(*(input.getResults())));
  if (isCompositeInstruction(*(input.getResults())))
  {
    const auto* composite = input.getResults()->cast_const<CompositeInstruction>();
    if (isCompositeEmpty(*composite))
    {
      CONSOLE_BRIDGE_logDebug("Seed is empty!");
      return 0;
    }
  }
  return 1;
}

}  // namespace tesseract_planning

#endif
