/**
 * @file timer.h
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
#ifndef TESSERACT_COMMON_TIMER_H
#define TESSERACT_COMMON_TIMER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <chrono>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/** @brief A simple timer class leveraging chrono high resolution clock */
class Timer
{
  using Clock = std::chrono::high_resolution_clock;

public:
  /** @brief Start the timer */
  void start()
  {
    start_time_ = Clock::now();
    running_ = true;
  }

  /** @brief Stop the timer */
  void stop()
  {
    end_time_ = Clock::now();
    running_ = false;
  }

  /**
   * @brief Get the elapsed time in milliseconds
   * @details If timer is actively running it will use Clock::now() as the end time
   * @return The elapsed time in milliseconds
   */
  double elapsedMilliseconds() const
  {
    if (running_)
      return std::chrono::duration<double, std::milli>(Clock::now() - start_time_).count();

    return std::chrono::duration<double, std::milli>(end_time_ - start_time_).count();
  }

  /**
   * @brief Get the elapsed time in seconds
   * @details If timer is actively running it will use Clock::now() as the end time
   * @return The elapsed time in seconds
   */
  double elapsedSeconds() const { return (elapsedMilliseconds() / 1000.0); }

private:
  std::chrono::time_point<Clock> start_time_;
  std::chrono::time_point<Clock> end_time_;
  bool running_{ false };
};
}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_TIMER_H
