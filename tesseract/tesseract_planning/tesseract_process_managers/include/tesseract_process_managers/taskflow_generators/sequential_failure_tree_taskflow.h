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
#ifndef TESSERACT_PROCESS_MANAGERS_SEQUENTIAL_FAILURE_TREE_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_SEQUENTIAL_FAILURE_TREE_TASKFLOW_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/taskflow_generator.h>
#include <tesseract_process_managers/process_generator.h>

namespace tesseract_planning
{
/** @brief This class generates taskflows for a sequential failure tree. Each process is executed in order until one
 * succeeds. Between each process, the validator tasks are executed (if not empty). For a process to succeed, the
 * process itself must succeed and all of the validators must succeed*/
class SequentialFailureTreeTaskflow : public TaskflowGenerator
{
public:
  SequentialFailureTreeTaskflow() = default;
  SequentialFailureTreeTaskflow(std::vector<ProcessGenerator::Ptr> processes,
                                std::string name = "SequentialFailureTreeTaskflow");

  tf::Taskflow& generateTaskflow(ProcessInput input,
                                 std::function<void()> done_cb,
                                 std::function<void()> error_cb) override;

  tf::Taskflow& generateTaskflow(ProcessInput input,
                                 const Instruction& start_instruction,
                                 std::function<void()> done_cb,
                                 std::function<void()> error_cb);

  tf::Taskflow& generateTaskflow(ProcessInput input,
                                 const Instruction& start_instruction,
                                 const Instruction& end_instruction,
                                 std::function<void()> done_cb,
                                 std::function<void()> error_cb);
  /**
   * @brief Add another process that will be added to the taskflow
   * @param process Process added to the taskflow
   */
  void registerProcess(const ProcessGenerator::Ptr& process);

  std::string name;

private:
  std::vector<ProcessGenerator::Ptr> processes_;
  std::vector<ProcessGenerator::Ptr> validators_;
  std::vector<std::shared_ptr<tf::Taskflow>> sequential_failure_trees_;
  std::vector<tf::Task> process_tasks_;

  Instruction null_instruction{ NullInstruction() };
};

}  // namespace tesseract_planning

#endif
