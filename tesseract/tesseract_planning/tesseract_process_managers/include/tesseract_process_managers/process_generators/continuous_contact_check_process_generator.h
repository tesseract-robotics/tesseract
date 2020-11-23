/**
 * @file continuous_contact_check_process_generator.h
 * @brief Continuous Collision check trajectory
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
#ifndef TESSERACT_PROCESS_MANAGERS_CONTINUOUS_CONTACT_CHECK_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_CONTINUOUS_CONTACT_CHECK_PROCESS_GENERATOR_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generator.h>

namespace tesseract_planning
{
class ContinuousContactCheckProcessGenerator : public ProcessGenerator
{
public:
  using UPtr = std::unique_ptr<ContinuousContactCheckProcessGenerator>;

  ContinuousContactCheckProcessGenerator(std::string name = "Continuous Contact Check Trajectory");

  ContinuousContactCheckProcessGenerator(double longest_valid_segment_length,
                                         double contact_distance,
                                         std::string name = "Continuous Contact Check Trajectory");

  ~ContinuousContactCheckProcessGenerator() override = default;
  ContinuousContactCheckProcessGenerator(const ContinuousContactCheckProcessGenerator&) = delete;
  ContinuousContactCheckProcessGenerator& operator=(const ContinuousContactCheckProcessGenerator&) = delete;
  ContinuousContactCheckProcessGenerator(ContinuousContactCheckProcessGenerator&&) = delete;
  ContinuousContactCheckProcessGenerator& operator=(ContinuousContactCheckProcessGenerator&&) = delete;

  const std::string& getName() const override;

  std::function<void()> generateTask(ProcessInput input, std::size_t unique_id) override;

  std::function<int()> generateConditionalTask(ProcessInput input, std::size_t unique_id) override;

  bool getAbort() const override;

  void setAbort(bool abort) override;

  tesseract_collision::CollisionCheckConfig config;

private:
  /** @brief If true, all tasks return immediately. Workaround for https://github.com/taskflow/taskflow/issues/201 */
  std::atomic<bool> abort_{ false };

  std::string name_;

  int conditionalProcess(ProcessInput input, std::size_t unique_id) const;

  void process(ProcessInput input, std::size_t unique_id) const;
};

class ContinuousContactCheckProcessInfo : public ProcessInfo
{
public:
  ContinuousContactCheckProcessInfo(std::size_t unique_id, std::string name = "Continuous Contact Check Trajectory");

  std::vector<tesseract_collision::ContactResultMap> contact_results;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_PROCESS_MANAGERS_CONTINUOUS_CONTACT_CHECK_PROCESS_GENERATOR_H
