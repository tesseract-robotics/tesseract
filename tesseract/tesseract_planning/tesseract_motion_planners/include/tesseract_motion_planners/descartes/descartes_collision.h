/**
 * @file descartes_collision.h
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_H

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <descartes_light/interface/collision_interface.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_motion_planners
{
template <typename FloatType>
class DescartesCollision : public descartes_light::CollisionInterface<FloatType>
{
public:
  /**
   * @brief TesseractCollision
   * @param collision_env The collision Environment
   * @param active_links The list of active links
   * @param joint_names The list of joint names in the order that the data will be provided to the validate function.
   */
  DescartesCollision(const tesseract_environment::Environment::ConstPtr& collision_env,
                     const std::vector<std::string>& active_links,
                     const std::vector<std::string>& joint_names);

  /**
   * @brief Copy constructor that clones the object
   * @param collision_interface Object to copy/clone
   */
  DescartesCollision(const DescartesCollision& collision_interface);

  bool validate(const FloatType* pos, const std::size_t size) override;

  FloatType distance(const FloatType* pos, const std::size_t size) override;

  typename descartes_light::CollisionInterface<FloatType>::Ptr clone() const override;

private:
  bool isContactAllowed(const std::string& a, const std::string& b) const;

  tesseract_environment::StateSolver::Ptr state_solver_;
  tesseract_scene_graph::AllowedCollisionMatrix acm_;
  std::vector<std::string> active_link_names_;
  std::vector<std::string> joint_names_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
};

using DescartesCollisionF = DescartesCollision<float>;
using DescartesCollisionD = DescartesCollision<double>;

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_H
