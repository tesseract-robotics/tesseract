/**
 * @file continuous_contact_manager.h
 * @brief This is the continuous contact manager base class
 *
 * It should be used to perform continuous contact checking.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/types.h>
#include <tesseract/common/fwd.h>
#include <tesseract/common/types.h>

namespace tesseract::collision
{
class ContinuousContactManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ContinuousContactManager>;
  using ConstPtr = std::shared_ptr<const ContinuousContactManager>;
  using UPtr = std::unique_ptr<ContinuousContactManager>;
  using ConstUPtr = std::unique_ptr<const ContinuousContactManager>;

  ContinuousContactManager() = default;
  virtual ~ContinuousContactManager() = default;
  ContinuousContactManager(const ContinuousContactManager&) = delete;
  ContinuousContactManager& operator=(const ContinuousContactManager&) = delete;
  ContinuousContactManager(ContinuousContactManager&&) = delete;
  ContinuousContactManager& operator=(ContinuousContactManager&&) = delete;

  /**
   * @brief Get the name of the contact manager
   * @return The name
   */
  virtual std::string getName() const = 0;

  /**
   * @brief Clone the manager
   *
   * This is to be used for multi threaded application. A user should
   * make a clone for each thread.
   */
  virtual ContinuousContactManager::UPtr clone() const = 0;

  /**
   * @brief Add a object to the checker
   * @param id              The LinkId of the object, must be unique.
   * @param mask_id         User defined id which gets stored in the results structure.
   * @param shapes          A vector of shapes that make up the collision object.
   * @param shape_poses     A vector of poses for each shape, must be same length as shapes
   * @return true if successfully added, otherwise false.
   */
  virtual bool addCollisionObject(const tesseract::common::LinkId& id,
                                  const int& mask_id,
                                  const CollisionShapesConst& shapes,
                                  const tesseract::common::VectorIsometry3d& shape_poses,
                                  bool enabled = true) = 0;

  /** @brief Add a collision object by name (delegates to LinkId overload) */
  virtual bool addCollisionObject(const std::string& name,
                                  const int& mask_id,
                                  const CollisionShapesConst& shapes,
                                  const tesseract::common::VectorIsometry3d& shape_poses,
                                  bool enabled = true);

  /**
   * @brief Get a collision objects collision geometries
   * @param id The collision object's LinkId
   * @return A vector of collision geometries. The vector will be empty if the collision object is not found.
   */
  virtual const CollisionShapesConst& getCollisionObjectGeometries(const tesseract::common::LinkId& id) const = 0;

  /** @brief Get collision geometries by name (delegates to LinkId overload) */
  virtual const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const;

  /**
   * @brief Get a collision objects collision geometries transforms
   * @param id The collision object's LinkId
   * @return A vector of collision geometries transforms. The vector will be empty if the collision object is not found.
   */
  virtual const tesseract::common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const tesseract::common::LinkId& id) const = 0;

  /** @brief Get collision geometry transforms by name (delegates to LinkId overload) */
  virtual const tesseract::common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const std::string& name) const;

  /**
   * @brief Find if a collision object already exists
   * @param id The LinkId of the collision object
   * @return true if it exists, otherwise false.
   */
  virtual bool hasCollisionObject(const tesseract::common::LinkId& id) const = 0;

  /** @brief Check if collision object exists by name (delegates to LinkId overload) */
  virtual bool hasCollisionObject(const std::string& name) const;

  /**
   * @brief Remove an object from the checker
   * @param id The LinkId of the object
   * @return true if successfully removed, otherwise false.
   */
  virtual bool removeCollisionObject(const tesseract::common::LinkId& id) = 0;

  /** @brief Remove collision object by name (delegates to LinkId overload) */
  virtual bool removeCollisionObject(const std::string& name);

  /**
   * @brief Enable an object
   * @param id The LinkId of the object
   */
  virtual bool enableCollisionObject(const tesseract::common::LinkId& id) = 0;

  /** @brief Enable collision object by name (delegates to LinkId overload) */
  virtual bool enableCollisionObject(const std::string& name);

  /**
   * @brief Disable an object
   * @param id The LinkId of the object
   */
  virtual bool disableCollisionObject(const tesseract::common::LinkId& id) = 0;

  /** @brief Disable collision object by name (delegates to LinkId overload) */
  virtual bool disableCollisionObject(const std::string& name);

  /**
   * @brief Check if collision object is enabled
   * @param id The LinkId of the object
   * @return True if enabled, otherwise false
   */
  virtual bool isCollisionObjectEnabled(const tesseract::common::LinkId& id) const = 0;

  /** @brief Check if collision object is enabled by name (delegates to LinkId overload) */
  virtual bool isCollisionObjectEnabled(const std::string& name) const;

  /**
   * @brief Set a single collision object's transforms
   * @param id The LinkId of the object
   * @param pose The transformation in world
   */
  virtual void setCollisionObjectsTransform(const tesseract::common::LinkId& id, const Eigen::Isometry3d& pose) = 0;

  /** @brief Set a single collision object's transform by name (delegates to LinkId overload) */
  virtual void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose);

  /**
   * @brief Set a series of collision object's transforms using integer link IDs
   * @param transforms A transform map <LinkId, pose>
   */
  virtual void setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& transforms) = 0;

  /** @brief Set a series of collision object's transforms by name (delegates to LinkId overload) */
  virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                            const tesseract::common::VectorIsometry3d& poses);

  /**
   * @brief Set a single cast(moving) collision object's transforms
   *
   * This should only be used for moving objects. Use the base
   * class methods for static objects.
   *
   * @param id The LinkId of the object
   * @param pose1 The start transformation in world
   * @param pose2 The end transformation in world
   */
  virtual void setCollisionObjectsTransform(const tesseract::common::LinkId& id,
                                            const Eigen::Isometry3d& pose1,
                                            const Eigen::Isometry3d& pose2) = 0;

  /** @brief Set a single cast collision object's transforms by name (delegates to LinkId overload) */
  virtual void setCollisionObjectsTransform(const std::string& name,
                                            const Eigen::Isometry3d& pose1,
                                            const Eigen::Isometry3d& pose2);

  /**
   * @brief Set a series of cast(moving) collision object's transforms using integer link IDs
   * @param pose1 A start transform map <LinkId, pose>
   * @param pose2 An end transform map <LinkId, pose>
   */
  virtual void setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& pose1,
                                            const tesseract::common::LinkIdTransformMap& pose2) = 0;

  /** @brief Set a series of cast collision object's transforms by name (delegates to LinkId overload) */
  virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                            const tesseract::common::VectorIsometry3d& pose1,
                                            const tesseract::common::VectorIsometry3d& pose2);

  /**
   * @brief Get all collision objects
   * @return A list of collision object IDs
   */
  virtual const std::vector<tesseract::common::LinkId>& getCollisionObjects() const = 0;

  /**
   * @brief Set which collision objects can move using integer link IDs
   * @param ids A vector of LinkIds identifying the active collision objects
   */
  virtual void setActiveCollisionObjects(const std::vector<tesseract::common::LinkId>& ids) = 0;

  /** @brief Set which collision objects can move by name (delegates to LinkId overload) */
  virtual void setActiveCollisionObjects(const std::vector<std::string>& names);

  /**
   * @brief Get which collision objects can move as LinkIds
   * @return A set of active collision object LinkIds
   */
  virtual const std::unordered_set<tesseract::common::LinkId, tesseract::common::LinkId::Hash>&
  getActiveCollisionObjectIds() const = 0;

  /**
   * @brief Get which collision objects can move as names
   * @return A list of collision object names (derived from getActiveCollisionObjectIds)
   */
  virtual std::vector<std::string> getActiveCollisionObjectNames() const;

  /**
   * @brief Set the contact distance threshold
   * @param collision_margin_data The contact distance
   */
  virtual void setCollisionMarginData(CollisionMarginData collision_margin_data) = 0;

  /**
   * @brief Get the contact distance threshold
   * @return The contact distance
   */
  virtual const CollisionMarginData& getCollisionMarginData() const = 0;

  /**
   * @brief Set the pair contact distance thresholds for which collision should be considered on a per pair basis
   * @param pair_margin_data Contains the pair collision margins that will replace the current settings
   * @param override_type This determines how the provided CollisionMarginData is applied
   */
  virtual void setCollisionMarginPairData(
      const CollisionMarginPairData& pair_margin_data,
      CollisionMarginPairOverrideType override_type = CollisionMarginPairOverrideType::REPLACE) = 0;

  /**
   * @brief Set the default collision margin
   * @param default_collision_margin New default collision margin
   */
  virtual void setDefaultCollisionMargin(double default_collision_margin) = 0;

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
  virtual void setCollisionMarginPair(const tesseract::common::LinkId& id1,
                                      const tesseract::common::LinkId& id2,
                                      double collision_margin) = 0;

  /**
   * @brief Increment the collision margin data by some value
   * @param increment The value to increment the collision margin value
   */
  virtual void incrementCollisionMargin(double increment) = 0;

  /** @brief Set the active function for determining if two links are allowed to be in collision */
  virtual void
  setContactAllowedValidator(std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator) = 0;

  /** @brief Get the active function for determining if two links are allowed to be in collision */
  virtual std::shared_ptr<const tesseract::common::ContactAllowedValidator> getContactAllowedValidator() const = 0;

  /**
   * @brief Perform a contact test for all objects based
   * @param collisions The Contact results data
   * @param type The type of contact test
   */
  virtual void contactTest(ContactResultMap& collisions, const ContactRequest& request) = 0;

  /**
   * @brief Applies settings in the config
   * @param config Settings to be applies
   */
  virtual void applyContactManagerConfig(const ContactManagerConfig& config);
};

}  // namespace tesseract::collision

#endif  // TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_H
