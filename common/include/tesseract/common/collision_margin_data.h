/**
 * @file collision_margin_data.h
 * @brief This is used to store collision margin information
 *
 * It should be used to perform continuous contact checking.
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 *
 * @copyright Copyright (c) 2018, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_COLLISION_MARGIN_DATA_H
#define TESSERACT_COMMON_COLLISION_MARGIN_DATA_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <optional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>
#include <tesseract/common/utils.h>

namespace tesseract::common
{
class CollisionMarginPairData;
class CollisionMarginData;

template <class Archive>
void serialize(Archive& ar, CollisionMarginPairData& obj);

template <class Archive>
void serialize(Archive& ar, CollisionMarginData& obj);

/** @brief Identifies how the provided contact margin data should be applied */
enum class CollisionMarginPairOverrideType : std::uint8_t
{
  /** @brief Do not apply contact margin data */
  NONE,
  /** @brief Replace the contact manager's CollisionMarginPairData */
  REPLACE,
  /**
   * @brief Modify the contact managers pair margins
   * @details This will preserve existing pairs not being modified by the provided margin data.
   * If a pair already exist it will be updated with the provided margin data.
   */
  MODIFY
};

using PairsCollisionMarginData = std::unordered_map<tesseract::common::LinkNamesPair, double>;

class CollisionMarginPairData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionMarginPairData() = default;
  CollisionMarginPairData(const PairsCollisionMarginData& pair_margins);

  /**
   * @brief Set the margin for a given contact pair
   *        * The order of the object names does not matter, that is handled internal to
   * the class.
   *        * @param obj1 The first object name. Order doesn't matter
   * @param obj2 The Second object name. Order doesn't matter
   * @param margin contacts with distance < collision_margin are considered in collision
   */
  void setCollisionMargin(const std::string& obj1, const std::string& obj2, double margin);

  /**
   * @brief Get the pairs collision margin data
   *
   * If a collision margin for the request pair does not exist it returns the default collision margin data.
   *
   * @param obj1 The first object name
   * @param obj2 The second object name
   * @return A link pair contact margin if exists
   */
  std::optional<double> getCollisionMargin(const std::string& obj1, const std::string& obj2) const;

  /**
   * @brief Get the largest pair collision margin
   * @return Max pair contact distance threshold if objects exist
   */
  std::optional<double> getMaxCollisionMargin() const;

  /**
   * @brief Get the largest collision margin for the given object
   * @return Max contact distance threshold if object exists
   */
  std::optional<double> getMaxCollisionMargin(const std::string& obj) const;

  /**
   * @brief Get Collision Margin Data for stored pairs
   * @return A map of link pairs collision margin data
   */
  const PairsCollisionMarginData& getCollisionMargins() const;

  /**
   * @brief Increment all margins by input amount. Useful for inflating or reducing margins
   * @param increment Amount to increment margins
   */
  void incrementMargins(double increment);

  /**
   * @brief Scale all margins by input value
   * @param scale Value by which all margins are multiplied
   */
  void scaleMargins(double scale);

  /**
   * @brief Check if empty
   * @return True if empty otherwise false
   */
  bool empty() const;

  /**
   * @brief Clear/Reset the data structure
   */
  void clear();

  /**
   * @brief Apply the contents of the provide CollisionMarginPairData based on the override type
   * @param pair_margin_data The collision margin pair data to apply
   * @param override_type The type indicating how the provided data should be applied.
   */
  void apply(const CollisionMarginPairData& pair_margin_data, CollisionMarginPairOverrideType override_type);

  bool operator==(const CollisionMarginPairData& rhs) const;
  bool operator!=(const CollisionMarginPairData& rhs) const;

private:
  /** @brief A map of link pair names to contact distance */
  PairsCollisionMarginData lookup_table_;

  /** @brief Stores the largest collision margin */
  std::optional<double> max_collision_margin_;

  /** @brief Stores the maximum collision margin for each object */
  std::unordered_map<std::string, double> object_max_margins_;

  /** @brief Set the margin for a given contact pair without updating the max margins */
  void setCollisionMarginHelper(const std::string& obj1, const std::string& obj2, double margin);

  /** @brief Recalculate the overall and the per-object max margins */
  void updateMaxMargins();

  template <class Archive>
  friend void ::tesseract::common::serialize(Archive& ar, CollisionMarginPairData& obj);
};

/** @brief Stores information about how the margins allowed between collision objects */
class CollisionMarginData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CollisionMarginData>;
  using ConstPtr = std::shared_ptr<const CollisionMarginData>;

  CollisionMarginData(double default_collision_margin = 0);
  CollisionMarginData(double default_collision_margin, CollisionMarginPairData pair_collision_margins);
  CollisionMarginData(CollisionMarginPairData pair_collision_margins);

  /**
   * @brief Set the default collision margin
   * @param default_collision_margin New default collision margin
   */
  void setDefaultCollisionMargin(double default_collision_margin);

  /**
   * @brief Get the default collision margin
   * @return default collision margin
   */
  double getDefaultCollisionMargin() const;

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
  void setCollisionMargin(const std::string& obj1, const std::string& obj2, double collision_margin);

  /**
   * @brief Get the pairs collision margin data
   *
   * If a collision margin for the request pair does not exist it returns the default collision margin data.
   *
   * @param obj1 The first object name
   * @param obj2 The second object name
   * @return A Vector2d[Contact Distance Threshold, Coefficient]
   */
  double getCollisionMargin(const std::string& obj1, const std::string& obj2) const;

  /**
   * @brief Get Collision Margin Data for stored pairs
   * @return A map of link pairs collision margin data
   */
  const CollisionMarginPairData& getCollisionMarginPairData() const;

  /**
   * @brief Get the largest collision margin
   *
   * This used when setting the contact distance in the contact manager.
   *
   * @return Max contact distance threshold
   */
  double getMaxCollisionMargin() const;

  /**
   * @brief Get the largest collision margin for the given object
   *
   * This used when setting the contact distance in the contact manager.
   *
   * @return Max contact distance threshold
   */
  double getMaxCollisionMargin(const std::string& obj) const;

  /**
   * @brief Increment all margins by input amount. Useful for inflating or reducing margins
   * @param increment Amount to increment margins
   */
  void incrementMargins(double increment);

  /**
   * @brief Scale all margins by input value
   * @param scale Value by which all margins are multiplied
   */
  void scaleMargins(double scale);

  /**
   * @brief Apply the contents of the provide CollisionMarginPairData based on the override type
   * @param pair_margin_data The collision margin pair data to apply
   * @param override_type The type indicating how the provided data should be applied.
   */
  void apply(const CollisionMarginPairData& pair_margin_data, CollisionMarginPairOverrideType override_type);

  bool operator==(const CollisionMarginData& rhs) const;
  bool operator!=(const CollisionMarginData& rhs) const;

private:
  /** @brief Stores the collision margin used if no pair-specific one is set */
  double default_collision_margin_{ 0 };

  /** @brief A map of link pair names to contact distance */
  CollisionMarginPairData pair_margins_;

  template <class Archive>
  friend void ::tesseract::common::serialize(Archive& ar, CollisionMarginData& obj);
};
}  // namespace tesseract::common

#endif  // TESSERACT_COMMON_COLLISION_MARGIN_DATA_H
