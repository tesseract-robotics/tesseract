/**
 * @file sequential_taskflow.cpp
 * @brief Creates a taskflow that sequentially calls processes
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

#include <tesseract_process_managers/taskflow_generators/sequential_taskflow.h>

namespace tesseract_planning
{
SequentialTaskflow::SequentialTaskflow(SequentialProcesses processes, std::string name)
  : processes_(std::move(processes)), name_(std::move(name))
{
}

const std::string& SequentialTaskflow::getName() const { return name_; }

tf::Taskflow& SequentialTaskflow::generateTaskflow(ProcessInput input,
                                                   std::function<void()> done_cb,
                                                   std::function<void()> error_cb)
{
  // Create and store the taskflow
  auto taskflow = std::make_shared<tf::Taskflow>(name_);
  sequential_trees_.push_back(taskflow);

  // Add "Done" task
  std::size_t done_task_idx = process_tasks_.size();
  if (done_cb)
    process_tasks_.push_back(taskflow->emplace(done_cb).name("Done Callback"));
  else
    process_tasks_.push_back(
        taskflow->emplace([&]() { std::cout << "Done SequentialTaskflow\n"; }).name("Done Callback"));

  // Add "Error" task
  std::size_t error_task_idx = process_tasks_.size();
  if (error_cb)
    process_tasks_.push_back(taskflow->emplace(error_cb).name("Error Callback"));
  else
    process_tasks_.push_back(
        taskflow->emplace([&]() { std::cout << "Error SequentialTaskflow\n"; }).name("Error Callback"));

  // Generate process tasks using each process generator
  std::size_t first_task_idx = process_tasks_.size();
  for (auto& process : processes_)
  {
    switch (process.second)
    {
      case SequentialTaskType::TASK:
      {
        tf::Task task = taskflow->placeholder();
        task.work(process.first->generateTask(input, task.hash_value()));
        task.name(process.first->getName());
        process_tasks_.push_back(task);
        break;
      }
      case SequentialTaskType::CONDITIONAL_EXIT_ON_FAILURE:
      case SequentialTaskType::CONDITIONAL_EXIT_ON_SUCCESS:
      {
        tf::Task task = taskflow->placeholder();
        task.work(process.first->generateConditionalTask(input, task.hash_value()));
        task.name(process.first->getName());
        process_tasks_.push_back(task);
        break;
      }
    }
  }

  // Apply the fail-active logic - sequentially calling the planners until one succeeds
  for (std::size_t i = first_task_idx; i < process_tasks_.size(); i++)
  {
    bool is_last = (i == (process_tasks_.size() - 1));
    switch (processes_[i - first_task_idx].second)
    {
      case SequentialTaskType::TASK:
      {
        if (is_last)
          process_tasks_[i].precede(process_tasks_[done_task_idx]);
        else
          process_tasks_[i].precede(process_tasks_[i + 1]);

        break;
      }
      case SequentialTaskType::CONDITIONAL_EXIT_ON_FAILURE:
      {
        if (is_last)
        {
          // If the last planner fails, call the error callback
          process_tasks_[i].precede(process_tasks_[error_task_idx], process_tasks_[done_task_idx]);
        }
        else
        {
          // If the process succeeds, go to the next process. Otherwise, got to error task
          process_tasks_[i].precede(process_tasks_[error_task_idx], process_tasks_[i + 1]);
        }
        break;
      }
      case SequentialTaskType::CONDITIONAL_EXIT_ON_SUCCESS:
      {
        if (is_last)
        {
          // If the last planner fails, call the error callback
          process_tasks_[i].precede(process_tasks_[error_task_idx], process_tasks_[done_task_idx]);
        }
        else
        {
          // If the process succeeds, go to the done task. Otherwise, got to the next task
          process_tasks_[i].precede(process_tasks_[i + 1], process_tasks_[done_task_idx]);
        }
        break;
      }
    }
  }

  return *taskflow;
}

void SequentialTaskflow::abort()
{
  abort_ = true;
  for (auto& gen : processes_)
    gen.first->setAbort(true);
}

void SequentialTaskflow::reset()
{
  abort_ = false;
  for (auto& gen : processes_)
    gen.first->setAbort(false);
}

void SequentialTaskflow::clear()
{
  reset();
  sequential_trees_.clear();
  process_tasks_.clear();
}

void SequentialTaskflow::registerProcess(ProcessGenerator::UPtr process, SequentialTaskType task_type)
{
  processes_.emplace_back(std::move(process), task_type);
}
}  // namespace tesseract_planning
