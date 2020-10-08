/**
 * @file simple_process_manager.h
 * @brief Plans simple paths aka. a single composite containing no composite instruction.
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
#ifndef TESSERACT_PROCESS_MANAGERS_SIMPLE_PROCESS_MANAGER_H
#define TESSERACT_PROCESS_MANAGERS_SIMPLE_PROCESS_MANAGER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <vector>
#include <thread>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_manager.h>
#include <tesseract_process_managers/taskflow_generator.h>
#include <tesseract_process_managers/visibility_control.h>

namespace tesseract_planning
{
/**
 * @brief This class provides a process manager for a simple process.
 *
 * Given a ProcessInput in the correct format, it handles the creation of the process dependencies and uses Taskflow to
 * execute them efficiently in a parallel based on those dependencies.
 *
 * The required format is below.
 *
 * Composite
 * {
 *   ... Must not contain any child composite instructions
 * }
 */
class TESSERACT_PROCESS_MANAGERS_PUBLIC SimpleProcessManager : public ProcessManager
{
public:
  using Ptr = std::shared_ptr<SimpleProcessManager>;
  using ConstPtr = std::shared_ptr<const SimpleProcessManager>;

  SimpleProcessManager(TaskflowGenerator::UPtr taskflow_generator, std::size_t n = std::thread::hardware_concurrency());
  ~SimpleProcessManager() override = default;
  SimpleProcessManager(const SimpleProcessManager&) = delete;
  SimpleProcessManager& operator=(const SimpleProcessManager&) = delete;
  SimpleProcessManager(SimpleProcessManager&&) = delete;
  SimpleProcessManager& operator=(SimpleProcessManager&&) = delete;

  bool init(ProcessInput input) override;

  bool execute() override;

  bool terminate() override;

  bool clear() override;

  void enableDebug(bool enabled) override;

  void enableProfile(bool enabled) override;

private:
  void successCallback(std::string message);
  void failureCallback(std::string message);
  bool success_{ false };
  bool debug_{ false };
  bool profile_{ false };

  TaskflowGenerator::UPtr taskflow_generator_;
  tf::Executor executor_;
  tf::Taskflow taskflow_;
  std::vector<tf::Task> simple_tasks_;
};

}  // namespace tesseract_planning

#endif
