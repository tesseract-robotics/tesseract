/**
 * @file stopwatch.h
 * @brief Simple stopwatch class using chrono
 *
 * @author Levi Armstrong
 * @date February 2, 2021
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
#ifndef TESSERACT_COMMON_STOPWATCH_H
#define TESSERACT_COMMON_STOPWATCH_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <chrono>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract::common
{
/** @brief A simple stopwatch class leveraging chrono high resolution clock */
class Stopwatch
{
  using Clock = std::chrono::steady_clock;

public:
  /** @brief Start the timer */
  void start();

  /** @brief Stop the timer */
  void stop();

  /**
   * @brief Get the elapsed time in milliseconds
   * @details If timer is actively running it will use Clock::now() as the end time
   * @return The elapsed time in milliseconds
   */
  double elapsedMilliseconds() const;

  /**
   * @brief Get the elapsed time in seconds
   * @details If timer is actively running it will use Clock::now() as the end time
   * @return The elapsed time in seconds
   */
  double elapsedSeconds() const;

private:
  std::chrono::time_point<Clock> start_time_;
  std::chrono::time_point<Clock> end_time_;
  bool running_{ false };
};
}  // namespace tesseract::common
#endif  // TESSERACT_COMMON_TIMER_H
