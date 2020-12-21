/**
 * @file process_info.h
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_INFO_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_INFO_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <shared_mutex>
#include <map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifdef SWIG
%shared_ptr(tesseract_planning::ProcessInfo)
#endif  // SWIG

namespace tesseract_planning
{
class ProcessInfo
{
public:
  using Ptr = std::shared_ptr<ProcessInfo>;
  using ConstPtr = std::shared_ptr<const ProcessInfo>;

  ProcessInfo(std::size_t unique_id, std::string name = "");
  virtual ~ProcessInfo() = default;
  ProcessInfo(const ProcessInfo&) = default;
  ProcessInfo& operator=(const ProcessInfo&) = default;
  ProcessInfo(ProcessInfo&&) = default;
  ProcessInfo& operator=(ProcessInfo&&) = default;

  int return_value;

  std::size_t unique_id;

  std::string process_name;

  std::string message;
};
}  // namespace tesseract_planning

#ifdef SWIG
%template(ProcessInfoMap) std::map<std::size_t, std::shared_ptr<const tesseract_planning::ProcessInfo>>;
#endif

namespace tesseract_planning
{
/** @brief A threadsafe container for ProcessInfos */
struct ProcessInfoContainer
{
  void addProcessInfo(ProcessInfo::ConstPtr process_info);

  ProcessInfo::ConstPtr operator[](std::size_t index) const;

  /** @brief Get a copy of the process_info_vec in case it gets resized*/
  std::map<std::size_t, ProcessInfo::ConstPtr> getProcessInfoMap() const;

private:
  mutable std::shared_mutex mutex_;
  std::map<std::size_t, ProcessInfo::ConstPtr> process_info_map_;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_INFO_H
