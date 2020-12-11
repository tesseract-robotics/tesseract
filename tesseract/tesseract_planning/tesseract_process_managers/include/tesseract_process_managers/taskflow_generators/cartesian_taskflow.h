/**
 * @file cartesian_taskflow.h
 * @brief Cartesian Graph Taskflow
 *
 * @author Levi Armstrong
 * @date August 27, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_CARTESIAN_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_CARTESIAN_TASKFLOW_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <vector>
#include <thread>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/taskflow_generator.h>
#include <tesseract_motion_planners/core/profile_dictionary.h>

namespace tesseract_planning
{
struct CartesianTaskflowParams
{
  bool enable_post_contact_discrete_check{ false };
  bool enable_post_contact_continuous_check{ true };
  bool enable_time_parameterization{ true };
};

class CartesianTaskflow : public TaskflowGenerator
{
public:
  using UPtr = std::unique_ptr<CartesianTaskflow>;

  CartesianTaskflow(CartesianTaskflowParams params, std::string name = "CartesianTaskflow");
  ~CartesianTaskflow() override = default;
  CartesianTaskflow(const CartesianTaskflow&) = delete;
  CartesianTaskflow& operator=(const CartesianTaskflow&) = delete;
  CartesianTaskflow(CartesianTaskflow&&) = delete;
  CartesianTaskflow& operator=(CartesianTaskflow&&) = delete;

  const std::string& getName() const override;

  TaskflowContainer generateTaskflow(ProcessInput input,
                                     std::function<void()> done_cb,
                                     std::function<void()> error_cb) override;

private:
  std::string name_;
  CartesianTaskflowParams params_;

  /**
   * @brief Checks that the ProcessInput is in the correct format.
   * @param input ProcessInput to be checked
   * @return True if in the correct format
   */
  bool checkProcessInput(const ProcessInput& input) const;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_PROCESS_MANAGERS_CARTESIAN_TASKFLOW_H
