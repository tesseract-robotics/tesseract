/**
 * @file continuous_contact_manager.h
 * @brief This is the continuous contact manager base class
 *
 * It should be used to perform continuous contact checking.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_H
#define TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>

#ifdef SWIG
%shared_ptr(tesseract_collision::ContinuousContactManager)
#endif  // SWIG

namespace tesseract_collision
{
class ContinuousContactManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ContinuousContactManager>;
  using ConstPtr = std::shared_ptr<const ContinuousContactManager>;

  ContinuousContactManager() = default;
  virtual ~ContinuousContactManager() = default;
  ContinuousContactManager(const ContinuousContactManager&) = delete;
  ContinuousContactManager& operator=(const ContinuousContactManager&) = delete;
  ContinuousContactManager(ContinuousContactManager&&) = delete;
  ContinuousContactManager& operator=(ContinuousContactManager&&) = delete;

  /**
   * @brief Clone the manager
   *
   * This is to be used for multi threaded application. A user should
   * make a clone for each thread.
   */
  virtual std::shared_ptr<ContinuousContactManager> clone() const = 0;

  /**
   * @brief Add a collision object to the checker
   *
   * All objects are added should initially be added as static objects. Use the
   * setContactRequest method of defining which collision objects are moving.
   *
   * @param name            The name of the object, must be unique.
   * @param mask_id         User defined id which gets stored in the results structure.
   * @param shapes          A vector of shapes that make up the collision object.
   * @param shape_poses     A vector of poses for each shape, must be same length as shapes
   * @param shape_types     A vector of shape types for encode the collision object. If the vector is of length 1 it is
   * used for all shapes.
   * @param collision_object_types A int identifying a conversion mode for the object. (ex. convert meshes to
   * convex_hulls)
   * @param enabled         Indicate if the object is enabled for collision checking.
   * @return true if successfully added, otherwise false.
   */
  virtual bool addCollisionObject(const std::string& name,
                                  const int& mask_id,
                                  const CollisionShapesConst& shapes,
                                  const tesseract_common::VectorIsometry3d& shape_poses,
                                  bool enabled = true) = 0;

  /**
   * @brief Get a collision objects collision geometries
   * @param name The collision objects name
   * @return A vector of collision geometries. The vector will be empty if the collision object is not found.
   */
  virtual const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const = 0;

  /**
   * @brief Get a collision objects collision geometries transforms
   * @param name  The collision objects name
   * @return A vector of collision geometries transforms. The vector will be empty if the collision object is not found.
   */
  virtual const tesseract_common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const std::string& name) const = 0;

  /**
   * @brief Find if a collision object already exists
   * @param name The name of the collision object
   * @return true if it exists, otherwise false.
   */
  virtual bool hasCollisionObject(const std::string& name) const = 0;

  /**
   * @brief Remove an object from the checker
   * @param name The name of the object
   * @return true if successfully removed, otherwise false.
   */
  virtual bool removeCollisionObject(const std::string& name) = 0;

  /**
   * @brief Enable an object
   * @param name The name of the object
   */
  virtual bool enableCollisionObject(const std::string& name) = 0;

  /**
   * @brief Disable an object
   * @param name The name of the object
   */
  virtual bool disableCollisionObject(const std::string& name) = 0;

  /**
   * @brief Set a single static collision object's tansforms
   * @param name The name of the object
   * @param pose The tranformation in world
   */
  virtual void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) = 0;

  /**
   * @brief Set a series of static collision object's tranforms
   * @param names The name of the object
   * @param poses The tranformation in world
   */
  virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                            const tesseract_common::VectorIsometry3d& poses) = 0;

  /**
   * @brief Set a series of static collision object's tranforms
   * @param transforms A transform map <name, pose>
   */
  virtual void setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms) = 0;

  /**
   * @brief Set a single cast(moving) collision object's tansforms
   *
   * This should only be used for moving objects. Use the base
   * class methods for static objects.
   *
   * @param name The name of the object
   * @param pose1 The start tranformation in world
   * @param pose2 The end tranformation in world
   */
  virtual void setCollisionObjectsTransform(const std::string& name,
                                            const Eigen::Isometry3d& pose1,
                                            const Eigen::Isometry3d& pose2) = 0;

  /**
   * @brief Set a series of cast(moving) collision object's tranforms
   *
   * This should only be used for moving objects. Use the base
   * class methods for static objects.
   *
   * @param names The name of the object
   * @param pose1 The start tranformations in world
   * @param pose2 The end tranformations in world
   */
  virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                            const tesseract_common::VectorIsometry3d& pose1,
                                            const tesseract_common::VectorIsometry3d& pose2) = 0;

  /**
   * @brief Set a series of cast(moving) collision object's tranforms
   *
   * This should only be used for moving objects. Use the base
   * class methods for static objects.
   *
   * @param pose1 A start transform map <name, pose>
   * @param pose2 A end transform map <name, pose>
   */
  virtual void setCollisionObjectsTransform(const tesseract_common::TransformMap& pose1,
                                            const tesseract_common::TransformMap& pose2) = 0;

  /**
   * @brief Get all collision objects
   * @return A list of collision object names
   */
  virtual const std::vector<std::string>& getCollisionObjects() const = 0;

  /**
   * @brief Set which collision objects can move
   * @param names A vector of collision object names
   */
  virtual void setActiveCollisionObjects(const std::vector<std::string>& names) = 0;

  /**
   * @brief Get which collision objects can move
   * @return A list of collision object names
   */
  virtual const std::vector<std::string>& getActiveCollisionObjects() const = 0;

  /**
   * @brief Set the contact distance thresholds for which collision should be considered on a per pair basis
   * @param collision_margin_data Contains the data that will replace the current settings
   * @param override_type This determines how the provided CollisionMarginData is applied
   */
  virtual void
  setCollisionMarginData(CollisionMarginData collision_margin_data,
                         CollisionMarginOverrideType override_type = CollisionMarginOverrideType::REPLACE) = 0;

  /**
   * @brief Set the default collision margin
   * @param default_collision_margin New default collision margin
   */
  virtual void setDefaultCollisionMarginData(double default_collision_margin) = 0;

  /**
   * @brief Set the margin for a given contact pair
   *
   * The order of the object names does not matter, that is handled internal to
   * the class.
   *
   * @param obj1 The first object name. Order doesn't matter
   * @param obj2 The Second object name. Order doesn't matter
   * @param collision_margin contacts with distance < collision_margin are considered in collision
   */
  virtual void setPairCollisionMarginData(const std::string& name1,
                                          const std::string& name2,
                                          double collision_margin) = 0;

  /**
   * @brief Get the contact distance threshold
   * @return The contact distance
   */
  virtual const CollisionMarginData& getCollisionMarginData() const = 0;

  /** @brief Set the active function for determining if two links are allowed to be in collision */
  virtual void setIsContactAllowedFn(IsContactAllowedFn fn) = 0;

  /** @brief Get the active function for determining if two links are allowed to be in collision */
  virtual IsContactAllowedFn getIsContactAllowedFn() const = 0;

  /**
   * @brief Perform a contact test for all objects based
   * @param collisions The Contact results data
   * @param type The type of contact test
   */
  virtual void contactTest(ContactResultMap& collisions, const ContactRequest& request) = 0;
};

}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_H
