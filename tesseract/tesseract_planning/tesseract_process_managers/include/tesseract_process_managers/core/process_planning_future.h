/**
 * @file process_planning_future.h
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_FUTURE_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_FUTURE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/process_interface.h>
#include <tesseract_process_managers/core/taskflow_generator.h>

#include <tesseract_motion_planners/core/types.h>

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/manipulator_info.h>

namespace tesseract_planning
{
/**
 * @brief This contains the result for the process planning request
 * @details Must check the status before access the results to know if available.
 * @note This must not go out of scope until the process has finished
 */
struct ProcessPlanningFuture
{
  /** @brief This is the future return from taskflow executor.run, used to check if process has finished */
  std::future<void> process_future;

  /** @brief This is used to abort the associated process and check if the process was successful */
  ProcessInterface::Ptr interface;

  /** @brief The stored input to the process */
  std::unique_ptr<Instruction> input;

  /** @brief The results to the process */
  std::unique_ptr<Instruction> results;

  /** @brief The stored global manipulator info */
  std::unique_ptr<const ManipulatorInfo> global_manip_info;

  /** @brief The stored plan profile remapping */
  std::unique_ptr<const PlannerProfileRemapping> plan_profile_remapping;

  /** @brief The stored composite profile remapping */
  std::unique_ptr<const PlannerProfileRemapping> composite_profile_remapping;

  /** @brief The taskflow container returned from the TaskflowGenerator that must remain during taskflow execution */
  TaskflowContainer taskflow_container;

  /** @brief Clear all content */
  void clear()
  {
    interface = nullptr;
    input = nullptr;
    results = nullptr;
    global_manip_info = nullptr;
    plan_profile_remapping = nullptr;
    composite_profile_remapping = nullptr;
    taskflow_container.clear();
  }

  /**
   * @brief This checks if the process has finished
   * @return True if the process finished, otherwise false
   */
  bool ready() const { return (process_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready); }

  /** @brief Wait until the process has finished */
  void wait() const { process_future.wait(); }

  /**
   * @brief Check if a process has finished for a given duration
   * @return The future status
   */
  template <typename R, typename P>
  std::future_status waitFor(const std::chrono::duration<R, P>& duration) const
  {
    return process_future.wait_for(duration);
  }

  /**
   * @brief Check if a process has finished up to a given time point
   * @return The future status
   */
  template <typename C, typename D>
  std::future_status waitUntil(const std::chrono::time_point<C, D>& abs) const
  {
    return process_future.wait_until(abs);
  }
};
}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_FUTURE_H
