/**
 * @file state_collision_validator.h
 * @brief Tesseract OMPL planner OMPL state collision check
 *
 * @author Levi Armstrong
 * @date February 17, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_STATE_COLLISION_VALIDATOR_H
#define TESSERACT_MOTION_PLANNERS_OMPL_STATE_COLLISION_VALIDATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/StateValidityChecker.h>
#include <mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_planning
{
/** @brief Continuous collision check between two states */
class StateCollisionValidator : public ompl::base::StateValidityChecker
{
public:
  StateCollisionValidator(const ompl::base::SpaceInformationPtr& space_info,
                          tesseract_environment::Environment::ConstPtr env,
                          tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                          double collision_safety_margin,
                          OMPLStateExtractor extractor);

  bool isValid(const ompl::base::State* state) const override;

private:
  /** @brief The Tesseract Environment */
  tesseract_environment::Environment::ConstPtr env_;

  /**< @brief The tesseract state solver */
  tesseract_environment::StateSolver::ConstPtr state_solver_;

  /** @brief The Tesseract Forward Kinematics */
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;

  /** @brief The continuous contact manager used for creating cached continuous contact managers. */
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;

  /** @brief A list of active links */
  std::vector<std::string> links_;

  /** @brief A list of active joints */
  std::vector<std::string> joints_;

  /** @bried This will extract an Eigen::VectorXd from the OMPL State */
  OMPLStateExtractor extractor_;

  // The items below are to cache the contact manager based on thread ID. Currently ompl is multi
  // threaded but the methods used to implement collision checking are not thread safe. To prevent
  // reconstructing the collision environment for every check this will cache a contact manager
  // based on its thread ID.

  /** @brief Contact manager caching mutex */
  mutable std::mutex mutex_;

  /** @brief The continuous contact manager cache */
  mutable std::map<unsigned long int, tesseract_collision::DiscreteContactManager::Ptr> contact_managers_;

  /** @brief The state solver manager cache */
  mutable std::map<unsigned long int, tesseract_environment::StateSolver::Ptr> state_solver_managers_;
};

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_OMPL_STATE_COLLISION_VALIDATOR_H
