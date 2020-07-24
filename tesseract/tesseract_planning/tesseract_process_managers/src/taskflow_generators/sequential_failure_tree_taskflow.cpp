/**
 * @file sequential_failure_tree_taskflow.h
 * @brief Creates a taskflow that sequentially calls processes until one succeeds
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <console_bridge/console.h>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/taskflow_generators/sequential_failure_tree_taskflow.h>

namespace tesseract_planning
{
SequentialFailureTreeTaskflow::SequentialFailureTreeTaskflow(std::vector<ProcessGenerator::Ptr> processes,
                                                             std::string name)
  : name(std::move(name)), processes_(std::move(processes))
{
}

tf::Taskflow& SequentialFailureTreeTaskflow::generateTaskflow(ProcessInput input,
                                                              std::function<void()> done_cb,
                                                              std::function<void()> error_cb)
{
  return generateTaskflow(input, null_instruction, null_instruction, done_cb, error_cb);
}

tf::Taskflow& SequentialFailureTreeTaskflow::generateTaskflow(ProcessInput input,
                                                              const Instruction& start_instruction,
                                                              std::function<void()> done_cb,
                                                              std::function<void()> error_cb)
{
  return generateTaskflow(input, start_instruction, null_instruction, done_cb, error_cb);
}

tf::Taskflow& SequentialFailureTreeTaskflow::generateTaskflow(ProcessInput input,
                                                              const Instruction& start_instruction,
                                                              const Instruction& end_instruction,
                                                              std::function<void()> done_cb,
                                                              std::function<void()> error_cb)
{
  // Create and store the taskflow
  auto taskflow = std::make_shared<tf::Taskflow>(name);
  sequential_failure_trees_.push_back(taskflow);

  // Add "Done" task
  std::size_t done_task_idx = process_tasks_.size();
  if (done_cb)
    process_tasks_.push_back(taskflow->emplace(done_cb).name("Done Callback"));
  else
    process_tasks_.push_back(
        taskflow->emplace([&]() { std::cout << "Done SequentialFailureTreeTaskflow\n"; }).name("Done Callback"));

  // Add "Error" task
  std::size_t error_task_idx = process_tasks_.size();
  if (error_cb)
    process_tasks_.push_back(taskflow->emplace(error_cb).name("Error Callback"));
  else
    process_tasks_.push_back(
        taskflow->emplace([&]() { std::cout << "Error SequentialFailureTreeTaskflow\n"; }).name("Error Callback"));

  // Generate process tasks using each process generator
  std::size_t first_task_idx = process_tasks_.size();
  for (auto& process : processes_)
  {
    tf::Task task = taskflow->emplace(process->generateConditionalTask(input, start_instruction, end_instruction))
                        .name(process->name);
    process_tasks_.push_back(task);
  }

  // Apply the fail-active logic - sequentially calling the planners until one succeeds
  for (std::size_t i = first_task_idx; i < process_tasks_.size() - 1; i++)
  {
    // If the process succeeds, go to the validator. Otherwise, proceed to the next process
    process_tasks_[i].precede(process_tasks_[i + 1], process_tasks_[done_task_idx]);
  }

  // If the last planner fails, call the error callback
  process_tasks_.back().precede(process_tasks_[error_task_idx], process_tasks_[done_task_idx]);

  return *taskflow;
}

void SequentialFailureTreeTaskflow::registerProcess(const ProcessGenerator::Ptr& process)
{
  processes_.push_back(process);
}
}  // namespace tesseract_planning
