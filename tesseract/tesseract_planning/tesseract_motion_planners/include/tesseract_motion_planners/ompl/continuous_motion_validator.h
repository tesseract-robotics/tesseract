/**
 * @file continuous_motion_validator.h
 * @brief Tesseract OMPL planner continuous collision check between two states
 *
 * @author Jonathan Meyer
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_CONTINUOUS_MOTION_VALIDATOR_H
#define TESSERACT_MOTION_PLANNERS_CONTINUOUS_MOTION_VALIDATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/MotionValidator.h>
#include <mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_motion_planners
{
/** @brief Continuous collision check between two states */
class ContinuousMotionValidator : public ompl::base::MotionValidator
{
public:
  ContinuousMotionValidator(const ompl::base::SpaceInformationPtr& space_info,
                            const tesseract_environment::Environment::ConstPtr& env,
                            tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                            double collision_safety_margin);

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

  bool checkMotion(const ompl::base::State* s1,
                   const ompl::base::State* s2,
                   std::pair<ompl::base::State*, double>& lastValid) const override;

private:
  /**
   * @brief Perform a continuous collision check between two ompl states
   * @param s1 First OMPL State
   * @param s2 Second OMPL State
   * @return True if not in collision, otherwise false.
   */
  bool continuousCollisionCheck(const ompl::base::State* s1, const ompl::base::State* s2) const;

  /**
   * @brief Perform a discrete collision for a given ompl state
   * @param s2 OMPL State
   * @return True if not in collision, otherwise false.
   */
  bool discreteCollisionCheck(const ompl::base::State* s2) const;

  /** @brief The Tesseract Environment */
  tesseract_environment::Environment::ConstPtr env_;

  /**< @brief The tesseract state solver */
  tesseract_environment::StateSolver::ConstPtr state_solver_;

  /** @brief The Tesseract Forward Kinematics */
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;

  /** @brief The continuous contact manager used for creating cached continuous contact managers. */
  tesseract_collision::ContinuousContactManager::Ptr continuous_contact_manager_;

  /** @brief The discrete contact manager used for creating cached discrete contact managers. */
  tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_;

  /** @brief A list of active links */
  std::vector<std::string> links_;

  /** @brief A list of active joints */
  std::vector<std::string> joints_;

  // The items below are to cache the contact manager based on thread ID. Currently ompl is multi
  // threaded but the methods used to implement collision checking are not thread safe. To prevent
  // reconstructing the collision environment for every check this will cache a contact manager
  // based on its thread ID.

  /** @brief Contact manager caching mutex */
  mutable std::mutex mutex_;

  /** @brief The continuous contact manager cache */
  mutable std::map<unsigned long int, tesseract_collision::ContinuousContactManager::Ptr> continuous_contact_managers_;

  /** @brief The discrete contact manager cache */
  mutable std::map<unsigned long int, tesseract_collision::DiscreteContactManager::Ptr> discrete_contact_managers_;

  /** @brief The state solver manager cache */
  mutable std::map<unsigned long int, tesseract_environment::StateSolver::Ptr> state_solver_managers_;
};
}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_CONTINUOUS_MOTION_VALIDATOR_H
