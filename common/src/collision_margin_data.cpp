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
CollisionMarginPairData::CollisionMarginPairData(const PairsCollisionMarginData& pair_margins)
{
  lookup_table_.reserve(pair_margins.size());
  for (const auto& [key, entry] : pair_margins)
    insertEntryChecked(key, entry);
  updateMaxMargins();
}

void CollisionMarginPairData::setCollisionMargin(const LinkId& id1, const LinkId& id2, double margin)
{
  setCollisionMarginHelper(id1, id2, margin);
  updateMaxMargins();
}

void CollisionMarginPairData::setCollisionMarginHelper(const LinkId& id1, const LinkId& id2, double margin)
{
  const LinkIdPair key(id1, id2);
  auto [name1, name2] = orderedPairNames(id1, id2);
  insertEntryChecked(key, PairMarginEntry{ std::move(name1), std::move(name2), margin });
}

void CollisionMarginPairData::insertEntryChecked(const LinkIdPair& key, PairMarginEntry entry)
{
  auto [it, inserted] = lookup_table_.try_emplace(key, std::move(entry));
  if (!inserted)
  {
    checkPairHashCollision("MarginData", entry.name1, entry.name2, it->second.name1, it->second.name2);
    it->second.margin = entry.margin;
  }
}

std::optional<double> CollisionMarginPairData::getCollisionMargin(const LinkIdPair& pair) const
{
  const auto it = lookup_table_.find(pair);
  if (it != lookup_table_.end())
    return it->second.margin;
  return {};
}

std::optional<double> CollisionMarginPairData::getCollisionMargin(const LinkId& id1, const LinkId& id2) const
{
  return getCollisionMargin(LinkIdPair(id1, id2));
}

std::optional<double> CollisionMarginPairData::getMaxCollisionMargin() const { return max_collision_margin_; }

std::optional<double> CollisionMarginPairData::getMaxCollisionMargin(const LinkId& obj_id) const
{
  auto it = object_max_margins_.find(obj_id.value());
  if (it != object_max_margins_.end())
    return it->second;

  return {};
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

  // Increment all object max margins by the same amount
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

  // Scale all object max margins by the same factor
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
    // Update the overall max margin
    assert(max_collision_margin_.has_value());
    max_collision_margin_ = std::max(max_collision_margin_.value(), entry.margin);  // NOLINT

    // Update max margin for obj1
    auto it1 = object_max_margins_.find(key.first_id());
    if (it1 == object_max_margins_.end())
      object_max_margins_[key.first_id()] = entry.margin;
    else
      it1->second = std::max(it1->second, entry.margin);

    // Update max margin for obj2
    auto it2 = object_max_margins_.find(key.second_id());
    if (it2 == object_max_margins_.end())
      object_max_margins_[key.second_id()] = entry.margin;
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
        insertEntryChecked(key, entry);

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
  // The contents of object_max_margins_ are not compared, as they are derived from lookup_table_

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

void CollisionMarginData::setCollisionMargin(const LinkId& id1, const LinkId& id2, double margin)
{
  pair_margins_.setCollisionMargin(id1, id2, margin);
}

double CollisionMarginData::getCollisionMargin(const LinkIdPair& pair) const
{
  std::optional<double> margin = pair_margins_.getCollisionMargin(pair);
  if (margin.has_value())
    return margin.value();

  return default_collision_margin_;
}

double CollisionMarginData::getCollisionMargin(const LinkId& id1, const LinkId& id2) const
{
  return getCollisionMargin(LinkIdPair(id1, id2));
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
