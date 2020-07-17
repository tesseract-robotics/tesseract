/**
 * @file freespace_process_manager.h
 * @brief Plans freespace paths
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
#ifndef TESSERACT_PROCESS_MANAGER_FREESPACE_PROCESS_MANAGER_H
#define TESSERACT_PROCESS_MANAGER_FREESPACE_PROCESS_MANAGER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_manager.h>
#include <tesseract_process_managers/process_input.h>
#include <tesseract_process_managers/process_generator.h>
#include <tesseract_process_managers/taskflow_generators/sequential_failure_tree_taskflow.h>

namespace tesseract_planning
{
/**
 * @brief This class provides a process manager for a freespace process
 *
 * Given a ProcessInput in the correct format, it handles the creation of the process dependencies and uses Taskflow to
 * execute them efficiently in a parallel based on those dependencies.
 *
 * The required format is below.
 *
 * Composite
 * {
 *   ...
 * }
 */
class FreespaceProcessManager : public ProcessManager
{
public:
  using Ptr = std::shared_ptr<FreespaceProcessManager>;
  using ConstPtr = std::shared_ptr<const FreespaceProcessManager>;

  FreespaceProcessManager();

  bool init(ProcessInput input) override;

  bool execute() override;

  bool terminate() override;

  bool clear() override;

  /**
   * @brief Process generators used to create the freespace planning taskflow. If empty, defaultFreespaceProcesses will
   * be used
   */
  std::vector<ProcessGenerator::Ptr> process_generators;

private:
  void successCallback();
  void failureCallback();
  bool success;

  SequentialFailureTreeTaskflow taskflow_generator;
  tf::Executor executor;
  tf::Taskflow taskflow;
  std::vector<tf::Task> freespace_tasks;
};

}  // namespace tesseract_planning

#endif
