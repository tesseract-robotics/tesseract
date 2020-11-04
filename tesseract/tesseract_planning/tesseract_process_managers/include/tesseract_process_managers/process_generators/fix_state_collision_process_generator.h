/**
 * @file fix_state_collision_process_generator.h
 * @brief Process generator for process that pushes plan instructions to be out of collision
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
#ifndef TESSERACT_PROCESS_MANAGERS_FIX_STATE_COLLISION_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_FIX_STATE_COLLISION_PROCESS_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generator.h>

namespace tesseract_planning
{
struct FixStateCollisionProfile
{
  using Ptr = std::shared_ptr<FixStateCollisionProfile>;
  using ConstPtr = std::shared_ptr<const FixStateCollisionProfile>;

  enum class Settings
  {
    START_ONLY,
    END_ONLY,
    ALL,
    DISABLED
  };

  /** @brief Used to specify method used to correct states in collision */
  enum class CorrectionMethod
  {
    NONE,
    TRAJOPT,
    RANDOM_SAMPLER
  };

  FixStateCollisionProfile(Settings mode = Settings::ALL) : mode(mode) {}

  /** @brief Sets which terms will be corrected  */
  Settings mode;

  /** @brief Order that correction methods will be applied. These will be attempted in order until one succeeds or all
   * have been tried */
  std::vector<CorrectionMethod> correction_workflow{ CorrectionMethod::TRAJOPT, CorrectionMethod::RANDOM_SAMPLER };

  /** @brief Percent of the total joint range that a joint will be allowed to be adjusted */
  double jiggle_factor{ 0.02 };

  /** @brief Safety margin applied to collision costs/cnts when using trajopt to correct collisions */
  double safety_margin{ 0.025 };

  /** @brief Number of sampling attempts if TrajOpt correction fails*/
  int sampling_attempts{ 100 };
};
using FixStateCollisionProfileMap = std::unordered_map<std::string, FixStateCollisionProfile::Ptr>;

/**
 * @brief This generator modifies the const input instructions in order to push waypoints that are in collision out of
 * collision.
 *
 * First it uses TrajOpt to correct the waypoint. If that fails, it reverts to random sampling
 */
class FixStateCollisionProcessGenerator : public ProcessGenerator
{
public:
  using UPtr = std::unique_ptr<FixStateCollisionProcessGenerator>;

  FixStateCollisionProcessGenerator(std::string name = "FixStateCollision");

  ~FixStateCollisionProcessGenerator() override = default;
  FixStateCollisionProcessGenerator(const FixStateCollisionProcessGenerator&) = delete;
  FixStateCollisionProcessGenerator& operator=(const FixStateCollisionProcessGenerator&) = delete;
  FixStateCollisionProcessGenerator(FixStateCollisionProcessGenerator&&) = delete;
  FixStateCollisionProcessGenerator& operator=(FixStateCollisionProcessGenerator&&) = delete;

  const std::string& getName() const override;

  std::function<void()> generateTask(ProcessInput input) override;

  std::function<int()> generateConditionalTask(ProcessInput input) override;

  bool getAbort() const override;

  void setAbort(bool abort) override;

  FixStateCollisionProfileMap composite_profiles;

private:
  /** @brief If true, all tasks return immediately. Workaround for https://github.com/taskflow/taskflow/issues/201 */
  std::atomic<bool> abort_{ false };

  std::string name_;

  int conditionalProcess(ProcessInput input) const;

  void process(ProcessInput input) const;
};

/**
 * @brief Checks if a joint state is in collision
 * @param start_pos Vector that represents a joint state
 * @param input Process Input associated with waypoint. Needed for kinematics, etc.
 * @return True if in collision
 */
bool StateInCollision(const Eigen::Ref<const Eigen::VectorXd>& start_pos,
                      const ProcessInput& input,
                      const FixStateCollisionProfile& profile);

/**
 * @brief Checks if a waypoint is in collision
 * @param waypoint Must be a waypoint for which getJointPosition will return a position
 * @param input Process Input associated with waypoint. Needed for kinematics, etc.
 * @return True if in collision
 */
bool WaypointInCollision(const Waypoint& waypoint, const ProcessInput& input, const FixStateCollisionProfile& profile);

/**
 * @brief Takes a waypoint and uses a small trajopt problem to push it out of collision if necessary
 * @param waypoint Must be a waypoint for which getJointPosition will return a position
 * @param input Process Input associated with waypoint. Needed for kinematics, etc.
 * @param profile Profile containing needed params
 * @return True if successful
 */
bool MoveWaypointFromCollisionTrajopt(Waypoint& waypoint,
                                      const ProcessInput& input,
                                      const FixStateCollisionProfile& profile);

/**
 * @brief Takes a waypoint and uses random sampling to find a position that is out of collision
 * @param waypoint Must be a waypoint for which getJointPosition will return a position
 * @param input Process Input associated with waypoint. Needed for kinematics, etc.
 * @param profile Profile containing needed params
 * @return True if successful
 */
bool MoveWaypointFromCollisionRandomSampler(Waypoint& waypoint,
                                            const ProcessInput& input,
                                            const FixStateCollisionProfile& profile);

bool ApplyCorrectionWorkflow(Waypoint& waypoint, const ProcessInput& input, const FixStateCollisionProfile& profile);
}  // namespace tesseract_planning
#endif  // TESSERACT_PROCESS_MANAGERS_FIX_STATE_BOUNDS_PROCESS_GENERATOR_H
