/**
 * @file process_planning_future.cpp
 * @brief A process planning future
 *
 * @author Levi Armstrong
 * @date August 18, 2020
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
#include <tesseract_process_managers/core/process_planning_future.h>

namespace tesseract_planning
{
void ProcessPlanningFuture::clear()
{
  interface = nullptr;
  input = nullptr;
  results = nullptr;
  global_manip_info = nullptr;
  plan_profile_remapping = nullptr;
  composite_profile_remapping = nullptr;
  taskflow_container.clear();
}

bool ProcessPlanningFuture::ready() const
{
  return (process_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready);
}

void ProcessPlanningFuture::wait() const { process_future.wait(); }

std::future_status ProcessPlanningFuture::waitFor(const std::chrono::duration<double>& duration) const
{
  return process_future.wait_for(duration);
}

std::future_status
ProcessPlanningFuture::waitUntil(const std::chrono::time_point<std::chrono::high_resolution_clock>& abs) const
{
  return process_future.wait_until(abs);
}
}  // namespace tesseract_planning
