/**
 * @file allowed_collision_matrix.cpp
 * @brief AllowedCollisionMatrix
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
#include <tesseract_common/allowed_collision_matrix.h>

namespace tesseract_common
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
  for (const auto& entry : entries)
  {
    auto link_pair = tesseract_common::makeOrderedLinkPair(entry.first.first, entry.first.second);
    lookup_table_[link_pair] = entry.second;
  }
}

void AllowedCollisionMatrix::addAllowedCollision(const std::string& link_name1,
                                                 const std::string& link_name2,
                                                 const std::string& reason)
{
  auto link_pair = tesseract_common::makeOrderedLinkPair(link_name1, link_name2);
  lookup_table_[link_pair] = reason;
}

const AllowedCollisionEntries& AllowedCollisionMatrix::getAllAllowedCollisions() const { return lookup_table_; }

void AllowedCollisionMatrix::removeAllowedCollision(const std::string& link_name1, const std::string& link_name2)
{
  auto link_pair = tesseract_common::makeOrderedLinkPair(link_name1, link_name2);
  lookup_table_.erase(link_pair);
}

void AllowedCollisionMatrix::removeAllowedCollision(const std::string& link_name)
{
  for (auto it = lookup_table_.begin(); it != lookup_table_.end() /* not hoisted */; /* no increment */)
  {
    if (it->first.first == link_name || it->first.second == link_name)
    {
      it = lookup_table_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

bool AllowedCollisionMatrix::isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const
{
  thread_local LinkNamesPair link_pair;
  tesseract_common::makeOrderedLinkPair(link_pair, link_name1, link_name2);
  return (lookup_table_.find(link_pair) != lookup_table_.end());
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

template <class Archive>
void AllowedCollisionMatrix::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(lookup_table_);
}

std::ostream& operator<<(std::ostream& os, const AllowedCollisionMatrix& acm)
{
  for (const auto& pair : acm.getAllAllowedCollisions())
    os << "link=" << pair.first.first << " link=" << pair.first.second << " reason=" << pair.second << "\n";
  return os;
}
}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::AllowedCollisionMatrix)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::AllowedCollisionMatrix)
