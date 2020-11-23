/**
 * @file iterative_spline_parameterization_process_generator.h
 * @brief Perform iterative spline time parameterization
 *
 * @author Levi Armstrong
 * @date August 11. 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_ITERATIVE_SPLINE_PARAMETERIZATION_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_ITERATIVE_SPLINE_PARAMETERIZATION_PROCESS_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generator.h>
#include <tesseract_time_parameterization/iterative_spline_parameterization.h>

namespace tesseract_planning
{
struct IterativeSplineParameterizationProfile
{
  using Ptr = std::shared_ptr<IterativeSplineParameterizationProfile>;
  using ConstPtr = std::shared_ptr<const IterativeSplineParameterizationProfile>;

  IterativeSplineParameterizationProfile(double max_velocity_scaling_factor = 1.0,
                                         double max_acceleration_scaling_factor = 1.0);

  /** @brief max_velocity_scaling_factor The max velocity scaling factor passed to the solver */
  double max_velocity_scaling_factor = 1.0;

  /** @brief max_velocity_scaling_factor The max acceleration scaling factor passed to the solver */
  double max_acceleration_scaling_factor = 1.0;
};
using IterativeSplineParameterizationProfileMap =
    std::unordered_map<std::string, IterativeSplineParameterizationProfile::Ptr>;

class IterativeSplineParameterizationProcessGenerator : public ProcessGenerator
{
public:
  using UPtr = std::unique_ptr<IterativeSplineParameterizationProcessGenerator>;

  IterativeSplineParameterizationProcessGenerator(bool add_points = true,
                                                  std::string name = "Iterative Spline Parameterization");

  ~IterativeSplineParameterizationProcessGenerator() override = default;
  IterativeSplineParameterizationProcessGenerator(const IterativeSplineParameterizationProcessGenerator&) = delete;
  IterativeSplineParameterizationProcessGenerator&
  operator=(const IterativeSplineParameterizationProcessGenerator&) = delete;
  IterativeSplineParameterizationProcessGenerator(IterativeSplineParameterizationProcessGenerator&&) = delete;
  IterativeSplineParameterizationProcessGenerator&
  operator=(IterativeSplineParameterizationProcessGenerator&&) = delete;

  const std::string& getName() const override;

  std::function<void()> generateTask(ProcessInput input, std::size_t unique_id) override;

  std::function<int()> generateConditionalTask(ProcessInput input, std::size_t unique_id) override;

  bool getAbort() const override;

  void setAbort(bool abort) override;

  IterativeSplineParameterizationProfileMap composite_profiles;
  IterativeSplineParameterizationProfileMap move_profiles;

private:
  /** @brief If true, all tasks return immediately. Workaround for https://github.com/taskflow/taskflow/issues/201 */
  std::atomic<bool> abort_{ false };

  std::string name_;

  IterativeSplineParameterization solver_;

  int conditionalProcess(ProcessInput input, std::size_t unique_id) const;

  void process(ProcessInput input, std::size_t unique_id) const;
};
class IterativeSplineParameterizationProcessInfo : public ProcessInfo
{
public:
  IterativeSplineParameterizationProcessInfo(std::size_t unique_id,
                                             std::string name = "Iterative Spline Parameterization");
};
}  // namespace tesseract_planning
#endif  // TESSERACT_PROCESS_MANAGERS_ITERATIVE_SPLINE_PARAMETERIZATION_PROCESS_GENERATOR_H
