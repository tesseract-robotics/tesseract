/**
 * @file timer.cpp
 * @brief Simple timer class using chrono and thread
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
Timer::~Timer() { stop(); }

void Timer::start(const std::function<void()>& callback, std::chrono::steady_clock::duration interval)
{
  running_ = true;
  timer_thread_ = std::thread([this, callback, interval]() {
    auto next_time = std::chrono::steady_clock::now();
    while (running_)
    {
      next_time += interval;
      callback();  // Call the provided function
      std::this_thread::sleep_until(next_time);
    }
  });
}

void Timer::stop()
{
  running_ = false;
  if (timer_thread_.joinable())
    timer_thread_.join();
}

}  // namespace tesseract_common
