/**
 * @file debug_observer.h
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
#ifndef TESSERACT_PROCESS_MANAGERS_DEBUG_OBSERVER_H
#define TESSERACT_PROCESS_MANAGERS_DEBUG_OBSERVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <taskflow/taskflow.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/**
 * @brief This is currently simple debug observer which prints a message when each of the functions below are called.
 */
class DebugObserver : public tf::ObserverInterface
{
public:
  using Ptr = std::shared_ptr<DebugObserver>;
  using ConstPtr = std::shared_ptr<const DebugObserver>;

  /**
   * @brief Constructor
   * @param name The name given to the Observer which also gets printed out.
   */
  DebugObserver(const std::string& name);

  void set_up(size_t num_workers) final;

  void on_entry(size_t w, tf::TaskView tv) final;

  void on_exit(size_t w, tf::TaskView tv) final;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_DEBUG_OBSERVER_H
