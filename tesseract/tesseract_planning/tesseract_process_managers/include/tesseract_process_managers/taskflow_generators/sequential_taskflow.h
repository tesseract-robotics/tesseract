/**
 * @file sequential_taskflow.h
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
#ifndef TESSERACT_PROCESS_MANAGERS_SEQUENTIAL_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_SEQUENTIAL_TASKFLOW_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/taskflow_generator.h>
#include <tesseract_process_managers/process_generator.h>
#include <tesseract_process_managers/visibility_control.h>

namespace tesseract_planning
{
enum class SequentialTaskType : int
{
  TASK = 0,
  CONDITIONAL_EXIT_ON_SUCCESS = 1,
  CONDITIONAL_EXIT_ON_FAILURE = 2
};

using SequentialProcesses = std::vector<std::pair<ProcessGenerator::UPtr, SequentialTaskType>>;

/** @brief This class generates taskflows for a sequential failure tree. Each process is executed in order until one
 * succeeds. Between each process, the validator tasks are executed (if not empty). For a process to succeed, the
 * process itself must succeed and all of the validators must succeed*/
class TESSERACT_PROCESS_MANAGERS_PUBLIC SequentialTaskflow : public TaskflowGenerator
{
public:
  using UPtr = std::unique_ptr<SequentialTaskflow>;

  SequentialTaskflow() = default;
  ~SequentialTaskflow() override = default;
  SequentialTaskflow(SequentialProcesses processes, std::string name = "SequentialTaskflow");
  SequentialTaskflow(const SequentialTaskflow&) = delete;
  SequentialTaskflow& operator=(const SequentialTaskflow&) = delete;
  SequentialTaskflow(SequentialTaskflow&&) = delete;
  SequentialTaskflow& operator=(SequentialTaskflow&&) = delete;

  const std::string& getName() const override;

  tf::Taskflow& generateTaskflow(ProcessInput input,
                                 std::function<void()> done_cb,
                                 std::function<void()> error_cb) override;

  void abort() override;

  void reset() override;

  void clear() override;

  /**
   * @brief Add another process that will be added to the taskflow
   * @param process Process added to the taskflow
   */
  void registerProcess(ProcessGenerator::UPtr process, SequentialTaskType task_type);

private:
  /** @brief If true, all tasks return immediately. Workaround for https://github.com/taskflow/taskflow/issues/201 */
  std::atomic<bool> abort_{ false };

  SequentialProcesses processes_;
  std::vector<std::shared_ptr<tf::Taskflow>> sequential_trees_;
  std::vector<tf::Task> process_tasks_;
  std::string name_;
};

}  // namespace tesseract_planning

#endif
