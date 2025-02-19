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
CollisionMarginData::CollisionMarginData(double default_collision_margin)
  : default_collision_margin_(default_collision_margin), max_collision_margin_(default_collision_margin)
{
}

CollisionMarginData::CollisionMarginData(double default_collision_margin,
                                         PairsCollisionMarginData pair_collision_margins)
  : default_collision_margin_(default_collision_margin), lookup_table_(std::move(pair_collision_margins))
{
  updateMaxCollisionMargin();
}

CollisionMarginData::CollisionMarginData(PairsCollisionMarginData pair_collision_margins)
  : lookup_table_(std::move(pair_collision_margins))
{
  updateMaxCollisionMargin();
}

void CollisionMarginData::setDefaultCollisionMargin(double default_collision_margin)
{
  default_collision_margin_ = default_collision_margin;
  updateMaxCollisionMargin();
}

double CollisionMarginData::getDefaultCollisionMargin() const { return default_collision_margin_; }

void CollisionMarginData::setPairCollisionMargin(const std::string& obj1,
                                                 const std::string& obj2,
                                                 double collision_margin)
{
  auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
  lookup_table_[key] = collision_margin;
  updateMaxCollisionMargin();
}

double CollisionMarginData::getPairCollisionMargin(const std::string& obj1, const std::string& obj2) const
{
  thread_local LinkNamesPair key;
  tesseract_common::makeOrderedLinkPair(key, obj1, obj2);
  const auto it = lookup_table_.find(key);

  if (it != lookup_table_.end())
    return it->second;

  return default_collision_margin_;
}

const PairsCollisionMarginData& CollisionMarginData::getPairCollisionMargins() const { return lookup_table_; }

double CollisionMarginData::getMaxCollisionMargin() const { return max_collision_margin_; }

void CollisionMarginData::incrementMargins(double increment)
{
  default_collision_margin_ += increment;
  max_collision_margin_ += increment;
  for (auto& pair : lookup_table_)
    pair.second += increment;
}

void CollisionMarginData::scaleMargins(double scale)
{
  default_collision_margin_ *= scale;
  max_collision_margin_ *= scale;
  for (auto& pair : lookup_table_)
    pair.second *= scale;
}

void CollisionMarginData::apply(const CollisionMarginData& collision_margin_data,
                                CollisionMarginOverrideType override_type)
{
  switch (override_type)
  {
    case CollisionMarginOverrideType::REPLACE:
    {
      *this = collision_margin_data;
      break;
    }
    case CollisionMarginOverrideType::MODIFY:
    {
      default_collision_margin_ = collision_margin_data.default_collision_margin_;

      for (const auto& p : collision_margin_data.lookup_table_)
        lookup_table_[p.first] = p.second;

      updateMaxCollisionMargin();
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

void CollisionMarginData::updateMaxCollisionMargin()
{
  max_collision_margin_ = default_collision_margin_;
  for (const auto& p : lookup_table_)
    max_collision_margin_ = std::max(p.second, max_collision_margin_);
}

bool CollisionMarginData::operator==(const CollisionMarginData& rhs) const
{
  bool ret_val = true;
  ret_val &=
      (tesseract_common::almostEqualRelativeAndAbs(default_collision_margin_, rhs.default_collision_margin_, 1e-5));
  ret_val &= (tesseract_common::almostEqualRelativeAndAbs(max_collision_margin_, rhs.max_collision_margin_, 1e-5));
  ret_val &= (lookup_table_.size() == rhs.lookup_table_.size());
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

bool CollisionMarginData::operator!=(const CollisionMarginData& rhs) const { return !operator==(rhs); }

template <class Archive>
void CollisionMarginData::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(default_collision_margin_);
  ar& BOOST_SERIALIZATION_NVP(max_collision_margin_);
  ar& BOOST_SERIALIZATION_NVP(lookup_table_);
}
}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::CollisionMarginData)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::CollisionMarginData)
