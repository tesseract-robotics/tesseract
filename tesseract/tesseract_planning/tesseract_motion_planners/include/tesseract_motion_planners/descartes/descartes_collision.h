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

#include <tesseract_common/macros.h>
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
   * @param collision_safety_margin The minimum distance allowed from a collision object
   * @param longest_valid_segment_length Used to check collisions between two state if norm(state0-state1) >
   * longest_valid_segment_length.
   * @param debug If true, this print debug information to the terminal
   */
  DescartesCollision(const tesseract_environment::Environment::ConstPtr& collision_env,
                     std::vector<std::string> active_links,
                     std::vector<std::string> joint_names,
                     double collision_safety_margin = 0.025,
                     bool debug = false);
  ~DescartesCollision() override = default;

  /**
   * @brief Copy constructor that clones the object
   * @param collision_interface Object to copy/clone
   */
  DescartesCollision(const DescartesCollision& collision_interface);
  DescartesCollision& operator=(const DescartesCollision&) = delete;
  DescartesCollision(DescartesCollision&&) = delete;
  DescartesCollision& operator=(DescartesCollision&&) = delete;

  /**
   * @brief This check is the provided solution passes the collision test defined by this class
   * @param pos The joint values array to validate
   * @param size The length of the array
   * @return True if passes collision test, otherwise false
   */
  bool validate(const FloatType* pos, std::size_t size) override;

  /**
   * @brief This gets the distance to the closest object
   * @param pos The joint values array to calculate the distance to the closest object
   * @param size The length of the array
   * @return The distance to the closest object
   */
  FloatType distance(const FloatType* pos, std::size_t size) override;

  /**
   * @brief This should clone the object and make new instance of objects that are not safe to share across threads
   * @return Descartes collision interface
   */
  typename descartes_light::CollisionInterface<FloatType>::Ptr clone() const override;

private:
  /**
   * @brief Check if two links are allowed to be in collision
   * @param a The name of the first link
   * @param b The name of the second link
   * @return True if allowed to be in collision, otherwise false
   */
  bool isContactAllowed(const std::string& a, const std::string& b) const;

  tesseract_environment::StateSolver::Ptr state_solver_;             /**< @brief The tesseract state solver */
  tesseract_scene_graph::AllowedCollisionMatrix acm_;                /**< @brief The allowed collision matrix */
  std::vector<std::string> active_link_names_;                       /**< @brief A vector of active link names */
  std::vector<std::string> joint_names_;                             /**< @brief A vector of joint names */
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_; /**< @brief The discrete contact manager */
  double collision_safety_margin_; /**< @brief The minimum allowed collision distance */
  bool debug_;                     /**< @brief Enable debug information to be printed to the terminal */
};

using DescartesCollisionF = DescartesCollision<float>;
using DescartesCollisionD = DescartesCollision<double>;

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_H
