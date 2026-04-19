/**
 * @file allowed_collision_matrix.cpp
 * @brief AllowedCollisionMatrix
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
#include <tesseract/common/allowed_collision_matrix.h>

#include <utility>

namespace tesseract::common
{
bool operator==(const AllowedCollisionEntries& entries_1, const AllowedCollisionEntries& entries_2)
{
  if (entries_1.size() != entries_2.size())
    return false;

  for (const auto& entry : entries_1)
  {
    auto cp = entries_2.find(entry.first);
    if (cp == entries_2.end())
      return false;
    if (cp->second != entry.second)
      return false;
  }
  return true;
}

AllowedCollisionMatrix::AllowedCollisionMatrix(AllowedCollisionEntries entries) : lookup_table_(std::move(entries)) {}

void AllowedCollisionMatrix::addAllowedCollision(const LinkId& link_id1,
                                                 const LinkId& link_id2,
                                                 const std::string& reason)
{
  const LinkIdPair key(link_id1, link_id2);
  auto [name1, name2] = orderedPairNames(link_id1, link_id2);

  auto [it, inserted] = lookup_table_.try_emplace(key);
  if (!inserted)
  {
    checkPairHashCollision("ACM", name1, name2, it->second.name1, it->second.name2);
    it->second.reason = reason;
  }
  else
  {
    it->second = ACMEntry{ std::move(name1), std::move(name2), reason };
  }
}

const AllowedCollisionEntries& AllowedCollisionMatrix::getAllAllowedCollisions() const { return lookup_table_; }

void AllowedCollisionMatrix::removeAllowedCollision(const LinkId& link_id1, const LinkId& link_id2)
{
  lookup_table_.erase(LinkIdPair(link_id1, link_id2));
}

void AllowedCollisionMatrix::removeAllowedCollision(const LinkId& link_id)
{
  const uint64_t id = link_id.value();
  for (auto it = lookup_table_.begin(); it != lookup_table_.end();)
  {
    if (it->first.first_id() == id || it->first.second_id() == id)
      it = lookup_table_.erase(it);
    else
      ++it;
  }
}

bool AllowedCollisionMatrix::isCollisionAllowed(const LinkIdPair& pair) const { return lookup_table_.contains(pair); }

void AllowedCollisionMatrix::clearAllowedCollisions() { lookup_table_.clear(); }

void AllowedCollisionMatrix::insertAllowedCollisionMatrix(const AllowedCollisionMatrix& acm)
{
  lookup_table_.insert(acm.getAllAllowedCollisions().begin(), acm.getAllAllowedCollisions().end());
}

void AllowedCollisionMatrix::reserveAllowedCollisionMatrix(std::size_t size) { lookup_table_.reserve(size); }

bool AllowedCollisionMatrix::operator==(const AllowedCollisionMatrix& rhs) const
{
  bool equal = true;
  equal &= lookup_table_ == rhs.lookup_table_;

  return equal;
}
bool AllowedCollisionMatrix::operator!=(const AllowedCollisionMatrix& rhs) const { return !operator==(rhs); }

std::ostream& operator<<(std::ostream& os, const AllowedCollisionMatrix& acm)
{
  for (const auto& [key, entry] : acm.getAllAllowedCollisions())
    os << "link=" << entry.name1 << " link=" << entry.name2 << " reason=" << entry.reason << "\n";
  return os;
}
}  // namespace tesseract::common
