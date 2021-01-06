/**
 * @file taskflow_interface.h
 * @brief Process Inteface
 *
 * @author Levi Armstrong
 * @date December 8. 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_taskflow_interface_H
#define TESSERACT_PROCESS_MANAGERS_taskflow_interface_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <atomic>
#include <map>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_info.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::TaskflowInterface)
#endif  // SWIG

namespace tesseract_planning
{
/**
 * @brief This is a thread safe class used for aborting a process along with checking if a process was succesful
 * @details If a process failed then the process has been abort by some child process
 */
class TaskflowInterface
{
public:
  using Ptr = std::shared_ptr<TaskflowInterface>;

  /**
   * @brief Check if the process was aborted
   * @return True if aborted, otherwise false
   */
  bool isAborted() const;

  /**
   * @brief Check if the process finished without error
   * @return True if the process was not aborted, otherwise false
   */
  bool isSuccessful() const;

  /** @brief Abort the process associated with this interface */
  void abort();

  /**
   * @brief Get TaskInfo for a specific task by unique ID
   * @param index Unique ID assigned the task from taskflow
   * @return The TaskInfo associated with this task
   */
  TaskInfo::ConstPtr getTaskInfo(const std::size_t& index) const;

  /**
   * @brief Get the entire stored map of TaskInfos
   * @return The map of TaskInfos stored by unique ID
   */
  std::map<std::size_t, TaskInfo::ConstPtr> getTaskInfoMap() const;

  /**
   * @brief Not meant to be used by users. Exposes TaskInfoContainer so that the
   * @return Threadsafe TaskInfo container
   */
  TaskInfoContainer::Ptr getTaskInfoContainer() const;

protected:
  std::atomic<bool> abort_{ false };

  /** @brief Threadsafe container for TaskInfos */
  TaskInfoContainer::Ptr task_infos_{ std::make_shared<TaskInfoContainer>() };
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_taskflow_interface_H
