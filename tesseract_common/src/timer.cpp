/**
 * @file timer.cpp
 * @brief Simple timer class using chrono
 *
 * @author Levi Armstrong
 * @date February 2, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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

#include <tesseract_common/timer.h>

namespace tesseract_common
{
void Timer::start()
{
  start_time_ = Clock::now();
  running_ = true;
}

void Timer::stop()
{
  end_time_ = Clock::now();
  running_ = false;
}

double Timer::elapsedMilliseconds() const
{
  if (running_)
    return std::chrono::duration<double, std::milli>(Clock::now() - start_time_).count();

  return std::chrono::duration<double, std::milli>(end_time_ - start_time_).count();
}

double Timer::elapsedSeconds() const { return (elapsedMilliseconds() / 1000.0); }

}  // namespace tesseract_common
