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

#include <tesseract_process_managers/core/process_info.h>

namespace tesseract_planning
{
ProcessInfo::ProcessInfo(std::size_t unique_id, std::string name) : unique_id(unique_id), message(std::move(name)) {}

void ProcessInfoContainer::addProcessInfo(ProcessInfo::ConstPtr process_info)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  process_info_map_[process_info->unique_id] = std::move(process_info);
}

ProcessInfo::ConstPtr ProcessInfoContainer::operator[](std::size_t index) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return process_info_map_.at(index);
}

std::map<std::size_t, ProcessInfo::ConstPtr> ProcessInfoContainer::getProcessInfoMap() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return process_info_map_;
}

}  // namespace tesseract_planning
