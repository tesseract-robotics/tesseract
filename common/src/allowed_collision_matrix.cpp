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

namespace tesseract::common
{
bool operator==(const AllowedCollisionEntries& entries_1, const AllowedCollisionEntries& entries_2)
{
  if (entries_1.size() != entries_2.size())
    return false;

  for (const auto& entry : entries_1)
  {
    // Check if the key exists
    auto cp = entries_2.find(entry.first);
    if (cp == entries_2.end())
      return false;
    // Check if the value is the same
    if (cp->second != entry.second)
      return false;
  }
  return true;
}

AllowedCollisionMatrix::AllowedCollisionMatrix(const AllowedCollisionEntries& entries)
{
  for (const auto& [key, reason] : entries)
    lookup_table_[key] = reason;
}

void AllowedCollisionMatrix::addAllowedCollision(const LinkId& link_id1,
                                                 const LinkId& link_id2,
                                                 const std::string& reason)
{
  // Temporary, not a reused scratch pair: operator[] stores the key, so only an rvalue can move the names in.
  lookup_table_[LinkIdPair(link_id1, link_id2)] = reason;
}

void AllowedCollisionMatrix::addAllowedCollision(const LinkIdPair& pair, const std::string& reason)
{
  lookup_table_[pair] = reason;
}

const AllowedCollisionEntries& AllowedCollisionMatrix::getAllAllowedCollisions() const { return lookup_table_; }

void AllowedCollisionMatrix::removeAllowedCollision(const LinkId& link_id1, const LinkId& link_id2)
{
  TESSERACT_THREAD_LOCAL LinkIdPair link_pair;
  link_pair.assign(link_id1, link_id2);
  lookup_table_.erase(link_pair);
}

void AllowedCollisionMatrix::removeAllowedCollision(const LinkIdPair& pair) { lookup_table_.erase(pair); }

void AllowedCollisionMatrix::removeAllowedCollision(const LinkId& link_id)
{
  for (auto it = lookup_table_.begin(); it != lookup_table_.end(); /* no increment */)
  {
    if (it->first.first() == link_id || it->first.second() == link_id)
    {
      it = lookup_table_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

bool AllowedCollisionMatrix::isCollisionAllowed(const LinkIdPair& pair) const
{
  return lookup_table_.find(pair) != lookup_table_.end();
}

void AllowedCollisionMatrix::clearAllowedCollisions() { lookup_table_.clear(); }

void AllowedCollisionMatrix::insertAllowedCollisionMatrix(const AllowedCollisionMatrix& acm)
{
  for (const auto& [key, reason] : acm.getAllAllowedCollisions())
    lookup_table_[key] = reason;
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
  for (const auto& [key, reason] : acm.getAllAllowedCollisions())
  {
    const auto [link1, link2] = key.orderedNameView();
    os << "link=" << link1 << " link=" << link2 << " reason=" << reason << "\n";
  }
  return os;
}
}  // namespace tesseract::common
