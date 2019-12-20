/**
 * @file descartes_collision.hpp
 * @brief Tesseract Descartes Collision Implementation
 *
 * @author Levi Armstrong
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
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_HPP

#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_environment/core/utils.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
bool DescartesCollision<FloatType>::isContactAllowed(const std::string& a, const std::string& b) const
{
  return acm_.isCollisionAllowed(a, b);
}

template <typename FloatType>
DescartesCollision<FloatType>::DescartesCollision(const tesseract_environment::Environment::ConstPtr& collision_env,
                                                  std::vector<std::string> active_links,
                                                  std::vector<std::string> joint_names,
                                                  double collision_safety_margin,
                                                  bool debug)
  : state_solver_(collision_env->getStateSolver())
  , acm_(*(collision_env->getAllowedCollisionMatrix()))
  , active_link_names_(std::move(active_links))
  , joint_names_(std::move(joint_names))
  , contact_manager_(collision_env->getDiscreteContactManager())
  , collision_safety_margin_(collision_safety_margin)
  , debug_(debug)
{
  contact_manager_->setActiveCollisionObjects(active_link_names_);
  contact_manager_->setContactDistanceThreshold(collision_safety_margin_);
  contact_manager_->setIsContactAllowedFn(
      std::bind(&tesseract_motion_planners::DescartesCollision<FloatType>::isContactAllowed,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  contact_manager_->setContactDistanceThreshold(collision_safety_margin_);
}

template <typename FloatType>
DescartesCollision<FloatType>::DescartesCollision(const DescartesCollision& collision_interface)
  : state_solver_(collision_interface.state_solver_->clone())
  , acm_(collision_interface.acm_)
  , active_link_names_(collision_interface.active_link_names_)
  , joint_names_(collision_interface.joint_names_)
  , contact_manager_(collision_interface.contact_manager_->clone())
  , collision_safety_margin_(collision_interface.collision_safety_margin_)
  , debug_(collision_interface.debug_)
{
  contact_manager_->setActiveCollisionObjects(active_link_names_);
  contact_manager_->setContactDistanceThreshold(collision_safety_margin_);
  contact_manager_->setIsContactAllowedFn(
      std::bind(&tesseract_motion_planners::DescartesCollision<FloatType>::isContactAllowed,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

template <typename FloatType>
bool DescartesCollision<FloatType>::validate(const FloatType* pos, std::size_t size)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  Eigen::VectorXd joint_angles(size);
  for (int i = 0; i < static_cast<int>(size); ++i)
    joint_angles(i) = pos[i];

  tesseract_environment::EnvState::Ptr env_state = state_solver_->getState(joint_names_, joint_angles);

  std::vector<tesseract_collision::ContactResultMap> results;
  bool in_contact =
      checkTrajectoryState(results, *contact_manager_, env_state, tesseract_collision::ContactTestType::FIRST, debug_);
  return (!in_contact);
}

template <typename FloatType>
FloatType DescartesCollision<FloatType>::distance(const FloatType* pos, std::size_t size)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  Eigen::VectorXd joint_angles(size);
  for (int i = 0; i < static_cast<int>(size); ++i)
    joint_angles(i) = pos[i];
  tesseract_environment::EnvState::Ptr env_state = state_solver_->getState(joint_names_, joint_angles);

  std::vector<tesseract_collision::ContactResultMap> results;
  bool in_contact = checkTrajectoryState(
      results, *contact_manager_, env_state, tesseract_collision::ContactTestType::CLOSEST, debug_);

  if (!in_contact)
    return static_cast<FloatType>(contact_manager_->getContactDistanceThreshold());

  return static_cast<FloatType>(results.begin()->begin()->second[0].distance);
}

template <typename FloatType>
typename descartes_light::CollisionInterface<FloatType>::Ptr DescartesCollision<FloatType>::clone() const
{
  return std::make_shared<DescartesCollision>(*this);
}

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_HPP
