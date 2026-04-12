/**
 * @file collision_margin_data.cpp
 * @brief CollisionMarginData
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
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

#include <tesseract/common/utils.h>
#include <tesseract/common/collision_margin_data.h>
#include <cassert>

namespace tesseract::common
{
bool MarginEntry::operator==(const MarginEntry& other) const
{
  return name1 == other.name1 && name2 == other.name2 &&
         tesseract::common::almostEqualRelativeAndAbs(margin, other.margin, 1e-5);
}
bool MarginEntry::operator!=(const MarginEntry& other) const { return !(*this == other); }

CollisionMarginPairData::CollisionMarginPairData(const PairsCollisionMarginData& pair_margins)
{
  for (const auto& [key, entry] : pair_margins)
    setCollisionMarginHelper(entry.name1, entry.name2, entry.margin);
  updateMaxMargins();
}

void CollisionMarginPairData::setCollisionMargin(const std::string& obj1, const std::string& obj2, double margin)
{
  setCollisionMarginHelper(obj1, obj2, margin);
  updateMaxMargins();
}

void CollisionMarginPairData::setCollisionMarginHelper(const std::string& obj1, const std::string& obj2, double margin)
{
  setCollisionMarginHelper(LinkId::fromName(obj1), LinkId::fromName(obj2), margin);
}

void CollisionMarginPairData::setCollisionMargin(const LinkId& id1, const LinkId& id2, double margin)
{
  setCollisionMarginHelper(id1, id2, margin);
  updateMaxMargins();
}

void CollisionMarginPairData::setCollisionMarginHelper(const LinkId& id1, const LinkId& id2, double margin)
{
  auto key = LinkIdPair::make(id1, id2);

  // Hash collision check
  auto it = lookup_table_.find(key);
  if (it != lookup_table_.end())
  {
    bool names_match = (it->second.name1 == id1.name() && it->second.name2 == id2.name()) ||
                       (it->second.name1 == id2.name() && it->second.name2 == id1.name());
    if (!names_match)
      throw std::runtime_error("MarginData LinkIdPair hash collision: ('" + id1.name() + "', '" + id2.name() +
                               "') collides with ('" + it->second.name1 + "', '" + it->second.name2 + "')");
  }

  if (id1.value <= id2.value)
    lookup_table_[key] = MarginEntry{ id1.name(), id2.name(), margin };
  else
    lookup_table_[key] = MarginEntry{ id2.name(), id1.name(), margin };
}

std::optional<double> CollisionMarginPairData::getCollisionMargin(const LinkId& id1, const LinkId& id2) const
{
  const auto it = lookup_table_.find(LinkIdPair::make(id1, id2));
  if (it != lookup_table_.end())
    return it->second.margin;
  return {};
}

std::optional<double> CollisionMarginPairData::getCollisionMargin(const std::string& obj1,
                                                                  const std::string& obj2) const
{
  return getCollisionMargin(LinkId::fromName(obj1), LinkId::fromName(obj2));
}

std::optional<double> CollisionMarginPairData::getMaxCollisionMargin() const { return max_collision_margin_; }

std::optional<double> CollisionMarginPairData::getMaxCollisionMargin(const LinkId& obj_id) const
{
  auto it = object_max_margins_.find(obj_id);
  if (it != object_max_margins_.end())
    return it->second;
  return {};
}

std::optional<double> CollisionMarginPairData::getMaxCollisionMargin(const std::string& obj) const
{
  return getMaxCollisionMargin(LinkId::fromName(obj));
}

const PairsCollisionMarginData& CollisionMarginPairData::getCollisionMargins() const { return lookup_table_; }

void CollisionMarginPairData::incrementMargins(double increment)
{
  if (lookup_table_.empty())
    return;

  assert(max_collision_margin_.has_value());
  max_collision_margin_.value() += increment;  // NOLINT
  for (auto& [key, entry] : lookup_table_)
    entry.margin += increment;

  for (auto& [id, max_margin] : object_max_margins_)
    max_margin += increment;
}

void CollisionMarginPairData::scaleMargins(double scale)
{
  if (lookup_table_.empty())
    return;

  assert(max_collision_margin_.has_value());
  max_collision_margin_.value() *= scale;  // NOLINT
  for (auto& [key, entry] : lookup_table_)
    entry.margin *= scale;

  for (auto& [id, max_margin] : object_max_margins_)
    max_margin *= scale;
}

bool CollisionMarginPairData::empty() const { return lookup_table_.empty(); }

void CollisionMarginPairData::clear()
{
  lookup_table_.clear();
  object_max_margins_.clear();
  max_collision_margin_.reset();
}

void CollisionMarginPairData::updateMaxMargins()
{
  if (lookup_table_.empty())
    return;

  max_collision_margin_ = std::numeric_limits<double>::lowest();
  object_max_margins_.clear();
  for (const auto& [key, entry] : lookup_table_)
  {
    auto id1 = LinkId::fromName(entry.name1);
    auto id2 = LinkId::fromName(entry.name2);

    assert(max_collision_margin_.has_value());
    max_collision_margin_ = std::max(max_collision_margin_.value(), entry.margin);  // NOLINT

    auto it1 = object_max_margins_.find(id1);
    if (it1 == object_max_margins_.end())
      object_max_margins_[id1] = entry.margin;
    else
      it1->second = std::max(it1->second, entry.margin);

    auto it2 = object_max_margins_.find(id2);
    if (it2 == object_max_margins_.end())
      object_max_margins_[id2] = entry.margin;
    else
      it2->second = std::max(it2->second, entry.margin);
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
      for (const auto& [key, entry] : pair_margin_data.lookup_table_)
        lookup_table_[key] = entry;

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
  ret_val &= (max_collision_margin_.has_value() == rhs.max_collision_margin_.has_value());

  if (max_collision_margin_.has_value() && rhs.max_collision_margin_.has_value())
    ret_val &= (tesseract::common::almostEqualRelativeAndAbs(
        max_collision_margin_.value(), rhs.max_collision_margin_.value(), 1e-5));

  ret_val &= (lookup_table_.size() == rhs.lookup_table_.size());
  ret_val &= (object_max_margins_.size() == rhs.object_max_margins_.size());

  if (ret_val)
  {
    for (const auto& [key, entry] : lookup_table_)
    {
      auto cp = rhs.lookup_table_.find(key);
      ret_val = (cp != rhs.lookup_table_.end());
      if (!ret_val)
        break;

      ret_val = tesseract::common::almostEqualRelativeAndAbs(entry.margin, cp->second.margin, 1e-5);
      if (!ret_val)
        break;
    }
  }

  return ret_val;
}

bool CollisionMarginPairData::operator!=(const CollisionMarginPairData& rhs) const { return !operator==(rhs); }

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
  pair_margins_.setCollisionMargin(LinkId::fromName(obj1), LinkId::fromName(obj2), margin);
}

void CollisionMarginData::setCollisionMargin(const LinkId& id1, const LinkId& id2, double margin)
{
  pair_margins_.setCollisionMargin(id1, id2, margin);
}

double CollisionMarginData::getCollisionMargin(const LinkId& id1, const LinkId& id2) const
{
  std::optional<double> margin = pair_margins_.getCollisionMargin(id1, id2);
  if (margin.has_value())
    return margin.value();
  return default_collision_margin_;
}

double CollisionMarginData::getCollisionMargin(const std::string& obj1, const std::string& obj2) const
{
  return getCollisionMargin(LinkId::fromName(obj1), LinkId::fromName(obj2));
}

const CollisionMarginPairData& CollisionMarginData::getCollisionMarginPairData() const { return pair_margins_; }

double CollisionMarginData::getMaxCollisionMargin() const
{
  if (pair_margins_.empty())
    return default_collision_margin_;

  std::optional<double> mv = pair_margins_.getMaxCollisionMargin();
  if (!mv.has_value())
    return default_collision_margin_;

  return std::max(default_collision_margin_, mv.value());
}

double CollisionMarginData::getMaxCollisionMargin(const LinkId& obj_id) const
{
  if (pair_margins_.empty())
    return default_collision_margin_;

  std::optional<double> object_max = pair_margins_.getMaxCollisionMargin(obj_id);
  if (!object_max.has_value())
    return default_collision_margin_;

  return std::max(default_collision_margin_, object_max.value());
}

double CollisionMarginData::getMaxCollisionMargin(const std::string& obj) const
{
  return getMaxCollisionMargin(LinkId::fromName(obj));
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
      (tesseract::common::almostEqualRelativeAndAbs(default_collision_margin_, rhs.default_collision_margin_, 1e-5));
  ret_val &= (pair_margins_ == rhs.pair_margins_);
  return ret_val;
}

bool CollisionMarginData::operator!=(const CollisionMarginData& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::common
