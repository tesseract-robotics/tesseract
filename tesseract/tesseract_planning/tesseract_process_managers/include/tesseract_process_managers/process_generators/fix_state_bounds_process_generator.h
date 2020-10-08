/**
 * @file fix_state_bounds_process_generator.h
 * @brief Process generator for process that pushes plan instructions back within joint limits
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_FIX_STATE_BOUNDS_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_FIX_STATE_BOUNDS_PROCESS_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generator.h>
#include <tesseract_time_parameterization/iterative_spline_parameterization.h>
#include <tesseract_process_managers/visibility_control.h>

namespace tesseract_planning
{
struct TESSERACT_PROCESS_MANAGERS_PUBLIC FixStateBoundsProfile
{
  using Ptr = std::shared_ptr<FixStateBoundsProfile>;
  using ConstPtr = std::shared_ptr<const FixStateBoundsProfile>;

  enum class Settings
  {
    START_ONLY,
    END_ONLY,
    ALL,
    DISABLED
  };

  FixStateBoundsProfile(Settings mode = Settings::ALL) : mode(mode) {}

  /** @brief Sets which terms will be corrected  */
  Settings mode;

  /** @brief Maximum amount the process is allowed to correct. If deviation is further than this, it will fail */
  double max_deviation_global = std::numeric_limits<double>::max();
};
using FixStateBoundsProfileMap = std::unordered_map<std::string, FixStateBoundsProfile::Ptr>;

/**
 * @brief This generator modifies the const input instructions in order to push waypoints that are outside of their
 * limits back within them.
 */
class TESSERACT_PROCESS_MANAGERS_PUBLIC FixStateBoundsProcessGenerator : public ProcessGenerator
{
public:
  using UPtr = std::unique_ptr<FixStateBoundsProcessGenerator>;

  FixStateBoundsProcessGenerator(std::string name = "Fix State Bounds");

  ~FixStateBoundsProcessGenerator() override = default;
  FixStateBoundsProcessGenerator(const FixStateBoundsProcessGenerator&) = delete;
  FixStateBoundsProcessGenerator& operator=(const FixStateBoundsProcessGenerator&) = delete;
  FixStateBoundsProcessGenerator(FixStateBoundsProcessGenerator&&) = delete;
  FixStateBoundsProcessGenerator& operator=(FixStateBoundsProcessGenerator&&) = delete;

  const std::string& getName() const override;

  std::function<void()> generateTask(ProcessInput input) override;

  std::function<int()> generateConditionalTask(ProcessInput input) override;

  bool getAbort() const override;

  void setAbort(bool abort) override;

  FixStateBoundsProfileMap composite_profiles;

private:
  /** @brief If true, all tasks return immediately. Workaround for https://github.com/taskflow/taskflow/issues/201 */
  std::atomic<bool> abort_{ false };

  std::string name_;

  int conditionalProcess(ProcessInput input) const;

  void process(ProcessInput input) const;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_PROCESS_MANAGERS_FIX_STATE_BOUNDS_PROCESS_GENERATOR_H
