/**
 * @file collision_margin_data.h
 * @brief This is used to store collision margin information
 *
 * It should be used to perform continuous contact checking.
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <string>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>

namespace tesseract_common
{
/** @brief Identifies how the provided contact margin data should be applied */
enum class CollisionMarginOverrideType
{
  /** @brief Do not apply contact margin data */
  NONE,
  /** @brief Replace the contact manager's CollisionMarginData */
  REPLACE,
  /** @brief Override the contact managers default margin only */
  OVERRIDE_DEFAULT_MARGIN,
  /** @brief Override the contact managers pair margin only. This does not preserve any existing pair margin data */
  OVERRIDE_PAIR_MARGIN,
  /**
   * @brief Modify the contact managers pair margin only.
   * @details This will preserve existing pairs not being modified by the provided margin data.
   * If a pair already exist it will be updated with the provided margin data.
   */
  MODIFY_PAIR_MARGIN
};

using PairsCollisionMarginData =
    std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash>;

/** @brief Stores information about how the margins allowed between collision objects */
class CollisionMarginData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CollisionMarginData>;
  using ConstPtr = std::shared_ptr<const CollisionMarginData>;

  CollisionMarginData(double default_collision_margin = 0)
    : default_collision_margin_(default_collision_margin), max_collision_margin_(default_collision_margin)
  {
  }

  CollisionMarginData(double default_collision_margin, PairsCollisionMarginData pair_collision_margins)
    : default_collision_margin_(default_collision_margin), lookup_table_(std::move(pair_collision_margins))
  {
    updateMaxCollisionMargin();
  }

  CollisionMarginData(PairsCollisionMarginData pair_collision_margins)
    : lookup_table_(std::move(pair_collision_margins))
  {
    updateMaxCollisionMargin();
  }

  /**
   * @brief Set the default collision margin
   * @param default_collision_margin New default collision margin
   */
  void setDefaultCollisionMargin(double default_collision_margin)
  {
    default_collision_margin_ = default_collision_margin;
    updateMaxCollisionMargin();
  }

  /**
   * @brief Get the default collision margin
   * @return default collision margin
   */
  double getDefaultCollisionMargin() const { return default_collision_margin_; };

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
  void setPairCollisionMargin(const std::string& obj1, const std::string& obj2, double collision_margin)
  {
    auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
    lookup_table_[key] = collision_margin;
    updateMaxCollisionMargin();
  }

  /**
   * @brief Get the pairs collision margin data
   *
   * If a collision margin for the request pair does not exist it returns the default collision margin data.
   *
   * @param obj1 The first object name
   * @param obj2 The second object name
   * @return A Vector2d[Contact Distance Threshold, Coefficient]
   */
  double getPairCollisionMargin(const std::string& obj1, const std::string& obj2) const
  {
    auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
    const auto it = lookup_table_.find(key);

    if (it != lookup_table_.end())
      return it->second;

    return default_collision_margin_;
  }

  /**
   * @brief Get Collision Margin Data for stored pairs
   * @return A map of link pairs collision margin data
   */
  const PairsCollisionMarginData& getPairCollisionMargins() const { return lookup_table_; }

  /**
   * @brief Get the largest collision margin
   *
   * This used when setting the contact distance in the contact manager.
   *
   * @return Max contact distance threshold
   */
  double getMaxCollisionMargin() const { return max_collision_margin_; }

  /**
   * @brief Increment all margins by input amount. Useful for inflating or reducing margins
   * @param increment Amount to increment margins
   */
  void incrementMargins(const double& increment)
  {
    default_collision_margin_ += increment;
    max_collision_margin_ += increment;
    for (auto& pair : lookup_table_)
      pair.second += increment;
  }

  /**
   * @brief Scale all margins by input value
   * @param scale Value by which all margins are multipled
   */
  void scaleMargins(const double& scale)
  {
    default_collision_margin_ *= scale;
    max_collision_margin_ *= scale;
    for (auto& pair : lookup_table_)
      pair.second *= scale;
  }

  /**
   * @brief Apply the contents of the provide CollisionMarginData based on the override type
   * @param collision_margin_data The collision margin data to apply
   * @param override_type The type indicating how the provided data should be applied.
   */
  void apply(const CollisionMarginData& collision_margin_data, CollisionMarginOverrideType override_type)
  {
    switch (override_type)
    {
      case CollisionMarginOverrideType::REPLACE:
      {
        *this = collision_margin_data;
        break;
      }
      case CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN:
      {
        default_collision_margin_ = collision_margin_data.default_collision_margin_;
        updateMaxCollisionMargin();
        break;
      }
      case CollisionMarginOverrideType::OVERRIDE_PAIR_MARGIN:
      {
        lookup_table_ = collision_margin_data.lookup_table_;
        updateMaxCollisionMargin();
        break;
      }
      case CollisionMarginOverrideType::MODIFY_PAIR_MARGIN:
      {
        for (const auto& p : collision_margin_data.lookup_table_)
          lookup_table_[p.first] = p.second;

        updateMaxCollisionMargin();
        break;
      }
      case CollisionMarginOverrideType::NONE:
      {
        break;
      }
    }
  }

  bool operator==(const CollisionMarginData& other) const
  {
    bool ret_val = true;
    ret_val &=
        (tesseract_common::almostEqualRelativeAndAbs(default_collision_margin_, other.default_collision_margin_, 1e-5));
    ret_val &= (tesseract_common::almostEqualRelativeAndAbs(max_collision_margin_, other.max_collision_margin_, 1e-5));
    ret_val &= (lookup_table_.size() == other.lookup_table_.size());
    if (ret_val)
    {
      for (const auto& pair : lookup_table_)
      {
        auto cp = other.lookup_table_.find(pair.first);
        ret_val = (cp != other.lookup_table_.end());
        if (!ret_val)
          break;

        ret_val = tesseract_common::almostEqualRelativeAndAbs(pair.second, cp->second, 1e-5);
        if (!ret_val)
          break;
      }
    }
    return ret_val;
  }

  bool operator!=(const CollisionMarginData& rhs) const { return !operator==(rhs); }

private:
  /** @brief Stores the collision margin used if no pair-specific one is set */
  double default_collision_margin_{ 0 };

  /** @brief Stores the largest collision margin */
  double max_collision_margin_{ 0 };

  /** @brief A map of link pair names to contact distance */
  PairsCollisionMarginData lookup_table_;

  /** @brief Update the max collision margin */
  void updateMaxCollisionMargin()
  {
    max_collision_margin_ = default_collision_margin_;
    for (const auto& p : lookup_table_)
    {
      if (p.second > max_collision_margin_)
        max_collision_margin_ = p.second;
    }
  }
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_COLLISION_MARGIN_DATA_H
