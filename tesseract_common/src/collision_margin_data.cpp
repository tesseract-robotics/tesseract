/**
 * @file collision_margin_data.cpp
 * @brief CollisionMarginData
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/utility.hpp>
#if (BOOST_VERSION >= 107400) && (BOOST_VERSION < 107500)
#include <boost/serialization/library_version_type.hpp>
#endif
#include <boost/serialization/unordered_map.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/collision_margin_data.h>

namespace tesseract_common
{
CollisionMarginPairData::CollisionMarginPairData(const PairsCollisionMarginData& pair_margins)
{
  for (const auto& pair : pair_margins)
    setCollisionMarginHelper(pair.first.first, pair.first.second, pair.second);
  updateMaxMargins();
}

void CollisionMarginPairData::setCollisionMargin(const std::string& obj1, const std::string& obj2, double margin)
{
  setCollisionMarginHelper(obj1, obj2, margin);
  updateMaxMargins();
}

inline void CollisionMarginPairData::setCollisionMarginHelper(const std::string& obj1,
                                                              const std::string& obj2,
                                                              double margin)
{
  auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
  lookup_table_[key] = margin;
}

std::optional<double> CollisionMarginPairData::getCollisionMargin(const std::string& obj1,
                                                                  const std::string& obj2) const
{
  thread_local LinkNamesPair key;
  tesseract_common::makeOrderedLinkPair(key, obj1, obj2);
  const auto it = lookup_table_.find(key);

  if (it != lookup_table_.end())
    return it->second;

  return {};
}

double CollisionMarginPairData::getMaxCollisionMargin() const { return max_collision_margin_; }

double CollisionMarginPairData::getMaxCollisionMargin(const std::string& obj) const
{
  auto it = object_max_margins_.find(obj);
  if (it != object_max_margins_.end())
    return it->second;

  return std::numeric_limits<double>::lowest();
}

const PairsCollisionMarginData& CollisionMarginPairData::getCollisionMargins() const { return lookup_table_; }

void CollisionMarginPairData::incrementMargins(double increment)
{
  if (lookup_table_.empty())
    return;

  max_collision_margin_ += increment;
  for (auto& pair : lookup_table_)
    pair.second += increment;

  // Increment all object max margins by the same amount
  for (auto& obj_margin : object_max_margins_)
    obj_margin.second += increment;
}

void CollisionMarginPairData::scaleMargins(double scale)
{
  if (lookup_table_.empty())
    return;

  max_collision_margin_ *= scale;
  for (auto& pair : lookup_table_)
    pair.second *= scale;

  // Scale all object max margins by the same factor
  for (auto& obj_margin : object_max_margins_)
    obj_margin.second *= scale;
}

bool CollisionMarginPairData::empty() const { return lookup_table_.empty(); }

void CollisionMarginPairData::clear()
{
  lookup_table_.clear();
  object_max_margins_.clear();
  max_collision_margin_ = std::numeric_limits<double>::lowest();
}

void CollisionMarginPairData::updateMaxMargins()
{
  max_collision_margin_ = std::numeric_limits<double>::lowest();
  object_max_margins_.clear();
  for (const auto& pair : lookup_table_)
  {
    const std::string& obj1 = pair.first.first;
    const std::string& obj2 = pair.first.second;
    double margin = pair.second;

    // Update the overall max margin
    max_collision_margin_ = std::max(max_collision_margin_, margin);

    // Update max margin for obj1
    auto it1 = object_max_margins_.find(obj1);
    if (it1 == object_max_margins_.end())
      object_max_margins_[obj1] = margin;
    else
      it1->second = std::max(it1->second, margin);

    // Update max margin for obj2
    auto it2 = object_max_margins_.find(obj2);
    if (it2 == object_max_margins_.end())
      object_max_margins_[obj2] = margin;
    else
      it2->second = std::max(it2->second, margin);
  }
}

void CollisionMarginPairData::apply(const CollisionMarginPairData& pair_margin_data,
                                    CollisionMarginPairOverrideType override_type)
{
  switch (override_type)
  {
    case CollisionMarginPairOverrideType::REPLACE:
    {
      *this = pair_margin_data;
      break;
    }
    case CollisionMarginPairOverrideType::MODIFY:
    {
      for (const auto& p : pair_margin_data.lookup_table_)
        lookup_table_[p.first] = p.second;

      updateMaxMargins();
      break;
    }
    case CollisionMarginPairOverrideType::NONE:
    {
      break;
    }
  }
}

bool CollisionMarginPairData::operator==(const CollisionMarginPairData& rhs) const
{
  bool ret_val = true;
  ret_val &= (tesseract_common::almostEqualRelativeAndAbs(max_collision_margin_, rhs.max_collision_margin_, 1e-5));
  ret_val &= (lookup_table_.size() == rhs.lookup_table_.size());
  ret_val &= (object_max_margins_.size() == rhs.object_max_margins_.size());
  // The contents of object_max_margins_ are not compared, as they are derived from lookup_table_

  if (ret_val)
  {
    for (const auto& pair : lookup_table_)
    {
      auto cp = rhs.lookup_table_.find(pair.first);
      ret_val = (cp != rhs.lookup_table_.end());
      if (!ret_val)
        break;

      ret_val = tesseract_common::almostEqualRelativeAndAbs(pair.second, cp->second, 1e-5);
      if (!ret_val)
        break;
    }
  }

  return ret_val;
}

bool CollisionMarginPairData::operator!=(const CollisionMarginPairData& rhs) const { return !operator==(rhs); }

template <class Archive>
void CollisionMarginPairData::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(lookup_table_);

  // Recreate max_collision_margin_ and object_max_margins_ after deserialization
  if (Archive::is_loading::value)
  {
    updateMaxMargins();
  }
}

CollisionMarginData::CollisionMarginData(double default_collision_margin)
  : default_collision_margin_(default_collision_margin)
{
}

CollisionMarginData::CollisionMarginData(double default_collision_margin,
                                         CollisionMarginPairData pair_collision_margins)
  : default_collision_margin_(default_collision_margin), pair_margins_(std::move(pair_collision_margins))
{
}

CollisionMarginData::CollisionMarginData(CollisionMarginPairData pair_collision_margins)
  : pair_margins_(std::move(pair_collision_margins))
{
}

void CollisionMarginData::setDefaultCollisionMargin(double default_collision_margin)
{
  default_collision_margin_ = default_collision_margin;
}

double CollisionMarginData::getDefaultCollisionMargin() const { return default_collision_margin_; }

void CollisionMarginData::setCollisionMargin(const std::string& obj1, const std::string& obj2, double margin)
{
  pair_margins_.setCollisionMargin(obj1, obj2, margin);
}

double CollisionMarginData::getCollisionMargin(const std::string& obj1, const std::string& obj2) const
{
  std::optional<double> margin = pair_margins_.getCollisionMargin(obj1, obj2);
  if (margin.has_value())
    return margin.value();

  return default_collision_margin_;
}

const CollisionMarginPairData& CollisionMarginData::getCollisionMarginPairData() const { return pair_margins_; }

double CollisionMarginData::getMaxCollisionMargin() const
{
  if (pair_margins_.empty())
    return default_collision_margin_;

  return std::max(default_collision_margin_, pair_margins_.getMaxCollisionMargin());
}

double CollisionMarginData::getMaxCollisionMargin(const std::string& obj) const
{
  if (pair_margins_.empty())
    return default_collision_margin_;

  return std::max(default_collision_margin_, pair_margins_.getMaxCollisionMargin(obj));
}

void CollisionMarginData::incrementMargins(double increment)
{
  default_collision_margin_ += increment;
  pair_margins_.incrementMargins(increment);
}

void CollisionMarginData::scaleMargins(double scale)
{
  default_collision_margin_ *= scale;
  pair_margins_.scaleMargins(scale);
}

void CollisionMarginData::apply(const CollisionMarginPairData& pair_margin_data,
                                CollisionMarginPairOverrideType override_type)
{
  pair_margins_.apply(pair_margin_data, override_type);
}

bool CollisionMarginData::operator==(const CollisionMarginData& rhs) const
{
  bool ret_val = true;
  ret_val &=
      (tesseract_common::almostEqualRelativeAndAbs(default_collision_margin_, rhs.default_collision_margin_, 1e-5));
  ret_val &= (pair_margins_ == rhs.pair_margins_);
  return ret_val;
}

bool CollisionMarginData::operator!=(const CollisionMarginData& rhs) const { return !operator==(rhs); }

template <class Archive>
void CollisionMarginData::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(default_collision_margin_);
  ar& BOOST_SERIALIZATION_NVP(pair_margins_);
}
}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::CollisionMarginPairData)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::CollisionMarginPairData)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::CollisionMarginData)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::CollisionMarginData)
