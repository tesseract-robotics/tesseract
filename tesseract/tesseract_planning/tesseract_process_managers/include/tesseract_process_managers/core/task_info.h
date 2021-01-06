/**
 * @file task_info.h
 * @brief Process Info
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
#ifndef TESSERACT_PROCESS_MANAGERS_task_info_H
#define TESSERACT_PROCESS_MANAGERS_task_info_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <shared_mutex>
#include <map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifdef SWIG
%shared_ptr(tesseract_planning::TaskInfo)
%template(TaskInfoMap) std::map<std::size_t, std::shared_ptr<const tesseract_planning::TaskInfo>>;
%shared_ptr(tesseract_planning::TaskInfoContainer)
#endif  // SWIG

namespace tesseract_planning
{
/** Stores information about a Task */
class TaskInfo
{
public:
  using Ptr = std::shared_ptr<TaskInfo>;
  using ConstPtr = std::shared_ptr<const TaskInfo>;

  TaskInfo(std::size_t unique_id, std::string name = "");
  virtual ~TaskInfo() = default;
  TaskInfo(const TaskInfo&) = default;
  TaskInfo& operator=(const TaskInfo&) = default;
  TaskInfo(TaskInfo&&) = default;
  TaskInfo& operator=(TaskInfo&&) = default;

  /** @brief Value returned from the Task on completion */
  int return_value;

  /** @brief Unique ID generated for the Task by Taskflow */
  std::size_t unique_id;

  std::string task_name;

  std::string message;
};

/** @brief A threadsafe container for TaskInfos */
struct TaskInfoContainer
{
  using Ptr = std::shared_ptr<TaskInfoContainer>;
  using ConstPtr = std::shared_ptr<const TaskInfoContainer>;

  void addTaskInfo(TaskInfo::ConstPtr task_info);

  TaskInfo::ConstPtr operator[](std::size_t index) const;

  /** @brief Get a copy of the task_info_map_ in case it gets resized*/
  std::map<std::size_t, TaskInfo::ConstPtr> getTaskInfoMap() const;

private:
  mutable std::shared_mutex mutex_;
  std::map<std::size_t, TaskInfo::ConstPtr> task_info_map_;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_task_info_H
