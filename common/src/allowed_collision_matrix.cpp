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
    auto cp = entries_2.find(entry.first);
    if (cp == entries_2.end())
      return false;
    if (cp->second != entry.second)
      return false;
  }
  return true;
}

AllowedCollisionMatrix::AllowedCollisionMatrix(const AllowedCollisionEntries& entries) : lookup_table_(entries) {}

void AllowedCollisionMatrix::addAllowedCollision(const std::string& link_name1,
                                                 const std::string& link_name2,
                                                 const std::string& reason)
{
  auto key = LinkIdPair::make(LinkId::fromName(link_name1), LinkId::fromName(link_name2));

  // Hash collision check
  auto it = lookup_table_.find(key);
  if (it != lookup_table_.end())
  {
    // Key exists — verify names match (canonically ordered)
    bool names_match = (it->second.name1 == link_name1 && it->second.name2 == link_name2) ||
                       (it->second.name1 == link_name2 && it->second.name2 == link_name1);
    if (!names_match)
      throw std::runtime_error("ACM LinkIdPair hash collision: ('" + link_name1 + "', '" + link_name2 +
                               "') collides with ('" + it->second.name1 + "', '" + it->second.name2 + "')");
  }

  // Canonical name ordering: match LinkIdPair's canonical order
  auto id1 = LinkId::fromName(link_name1);
  auto id2 = LinkId::fromName(link_name2);
  if (id1.value <= id2.value)
    lookup_table_[key] = ACMEntry{ link_name1, link_name2, reason };
  else
    lookup_table_[key] = ACMEntry{ link_name2, link_name1, reason };
}

const AllowedCollisionEntries& AllowedCollisionMatrix::getAllAllowedCollisions() const { return lookup_table_; }

void AllowedCollisionMatrix::removeAllowedCollision(const std::string& link_name1, const std::string& link_name2)
{
  auto key = LinkIdPair::make(LinkId::fromName(link_name1), LinkId::fromName(link_name2));
  lookup_table_.erase(key);
}

void AllowedCollisionMatrix::removeAllowedCollision(const std::string& link_name)
{
  auto link_id = LinkId::fromName(link_name);
  for (auto it = lookup_table_.begin(); it != lookup_table_.end();)
  {
    if (it->first.first == link_id || it->first.second == link_id)
      it = lookup_table_.erase(it);
    else
      ++it;
  }
}

bool AllowedCollisionMatrix::isCollisionAllowed(LinkId link_id1, LinkId link_id2) const
{
  return lookup_table_.find(LinkIdPair::make(link_id1, link_id2)) != lookup_table_.end();
}

bool AllowedCollisionMatrix::isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const
{
  return isCollisionAllowed(LinkId::fromName(link_name1), LinkId::fromName(link_name2));
}

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
