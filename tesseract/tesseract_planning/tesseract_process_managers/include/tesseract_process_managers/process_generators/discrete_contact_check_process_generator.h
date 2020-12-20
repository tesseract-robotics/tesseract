/**
 * @file discrete_contact_check_process_generator.h
 * @brief Discrete Collision check trajectory
 *
 * @author Levi Armstrong
 * @date August 10. 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_DISCRETE_CONTACT_CHECK_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_DISCRETE_CONTACT_CHECK_PROCESS_GENERATOR_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/process_generator.h>
#include <tesseract_process_managers/core/process_input.h>

namespace tesseract_planning
{
class DiscreteContactCheckProcessGenerator : public ProcessGenerator
{
public:
  using UPtr = std::unique_ptr<DiscreteContactCheckProcessGenerator>;

  DiscreteContactCheckProcessGenerator(std::string name = "Discrete Contact Check Trajectory");

  DiscreteContactCheckProcessGenerator(double longest_valid_segment_length,
                                       double contact_distance,
                                       std::string name = "Discrete Contact Check Trajectory");

  ~DiscreteContactCheckProcessGenerator() override = default;
  DiscreteContactCheckProcessGenerator(const DiscreteContactCheckProcessGenerator&) = delete;
  DiscreteContactCheckProcessGenerator& operator=(const DiscreteContactCheckProcessGenerator&) = delete;
  DiscreteContactCheckProcessGenerator(DiscreteContactCheckProcessGenerator&&) = delete;
  DiscreteContactCheckProcessGenerator& operator=(DiscreteContactCheckProcessGenerator&&) = delete;

  tesseract_collision::CollisionCheckConfig config;

  int conditionalProcess(ProcessInput input, std::size_t unique_id) const override;

  void process(ProcessInput input, std::size_t unique_id) const override;
};

class DiscreteContactCheckProcessInfo : public ProcessInfo
{
public:
  DiscreteContactCheckProcessInfo(std::size_t unique_id, std::string name = "Discrete Contact Check Trajectory");

  std::vector<tesseract_collision::ContactResultMap> contact_results;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_DISCRETE_CONTACT_CHECK_PROCESS_GENERATOR_H
