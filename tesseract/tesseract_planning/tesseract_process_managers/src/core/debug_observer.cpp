/**
 * @file debug_observer.cpp
 * @brief Debug Taskflow Observer
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
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/debug_observer.h>

namespace tesseract_planning
{
DebugObserver::DebugObserver(const std::string& name) { std::cout << "Constructing observer " << name << '\n'; }

void DebugObserver::set_up(size_t num_workers)
{
  std::cout << "Setting up observer with " << num_workers << " workers\n";
}

void DebugObserver::on_entry(size_t w, tf::TaskView tv)
{
  std::ostringstream oss;
  oss << "worker " << w << " ready to run " << tv.name() << '\n';
  std::cout << oss.str();
}

void DebugObserver::on_exit(size_t w, tf::TaskView tv)
{
  std::ostringstream oss;
  oss << "worker " << w << " finished running " << tv.name() << '\n';
  std::cout << oss.str();
}
}  // namespace tesseract_planning
