/**
 * @file state_collision_validator.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/SpaceInformation.h>
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/ompl/state_collision_validator.h>

namespace tesseract_planning
{
StateCollisionValidator::StateCollisionValidator(const ompl::base::SpaceInformationPtr& space_info,
                                                 tesseract_environment::Environment::ConstPtr env,
                                                 tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                                                 double collision_safety_margin,
                                                 OMPLStateExtractor extractor)
  : StateValidityChecker(space_info)
  , env_(std::move(env))
  , state_solver_(env_->getStateSolver())
  , kin_(std::move(kin))
  , contact_manager_(env_->getDiscreteContactManager())
  , extractor_(extractor)
{
  joints_ = kin_->getJointNames();

  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  tesseract_environment::AdjacencyMap adj_map(
      env_->getSceneGraph(), kin_->getActiveLinkNames(), env_->getCurrentState()->link_transforms);
  links_ = adj_map.getActiveLinkNames();

  contact_manager_->setActiveCollisionObjects(links_);
  contact_manager_->setContactDistanceThreshold(collision_safety_margin);
}

bool StateCollisionValidator::isValid(const ompl::base::State* state) const
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::DiscreteContactManager::Ptr cm;
  mutex_.lock();
  auto it = contact_managers_.find(hash);
  if (it == contact_managers_.end())
  {
    cm = contact_manager_->clone();
    contact_managers_[hash] = cm;
  }
  else
  {
    cm = it->second;
  }
  mutex_.unlock();

  Eigen::Map<Eigen::VectorXd> finish_joints = extractor_(state);
  tesseract_environment::EnvState::Ptr state1 = env_->getState(joints_, finish_joints);

  for (const auto& link_name : links_)
    cm->setCollisionObjectsTransform(link_name, state1->link_transforms[link_name]);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  return contact_map.empty();
}

}  // namespace tesseract_motion_planners
