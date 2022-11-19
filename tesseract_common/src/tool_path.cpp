/**
 * @file tool_path.cpp
 * @brief Common Tesseract Tool Path
 *
 * @author Levi Armstrong
 * @date Nov 16, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#include <boost/serialization/vector.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_serialize.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/tool_path.h>

namespace tesseract_common
{
ToolPath::ToolPath(std::string description)
  : uuid_(boost::uuids::random_generator()()), description_(std::move(description))
{
}

ToolPath::ToolPath(boost::uuids::uuid uuid, std::string description)
  : uuid_(std::move(uuid)), description_(std::move(description))
{
}

boost::uuids::uuid ToolPath::getUUID() const { return uuid_; }

void ToolPath::regenerateUUID() { uuid_ = boost::uuids::random_generator()(); }

const boost::uuids::uuid& ToolPath::getParentUUID() const { return parent_uuid_; }

void ToolPath::setParentUUID(const boost::uuids::uuid& uuid) { parent_uuid_ = uuid; }

void ToolPath::setDescription(const std::string& desc) { description_ = desc; }
const std::string& ToolPath::getDescription() const { return description_; }

bool ToolPath::operator==(const ToolPath& rhs) const
{
  bool equal = true;
  equal &= (uuid_ == rhs.uuid_);                // NOLINT
  equal &= (parent_uuid_ == rhs.parent_uuid_);  // NOLINT
  equal &= (description_ == rhs.description_);  // NOLINT
  equal &= (container_ == rhs.container_);
  return equal;
}

void ToolPath::setNamespace(std::string ns) { ns_ = std::move(ns); }

const std::string& ToolPath::getNamespace() const { return ns_; }

bool ToolPath::operator!=(const ToolPath& rhs) const { return !operator==(rhs); }

template <class Archive>
void ToolPath::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(uuid_);
  ar& BOOST_SERIALIZATION_NVP(parent_uuid_);
  ar& BOOST_SERIALIZATION_NVP(description_);
  ar& BOOST_SERIALIZATION_NVP(container_);
}

// LCOV_EXCL_START

///////////////////////////
// C++ container support //
///////////////////////////

///////////////
// Iterators //
///////////////
ToolPath::iterator ToolPath::begin() { return container_.begin(); }
ToolPath::const_iterator ToolPath::begin() const { return container_.begin(); }
ToolPath::iterator ToolPath::end() { return container_.end(); }
ToolPath::const_iterator ToolPath::end() const { return container_.end(); }
ToolPath::reverse_iterator ToolPath::rbegin() { return container_.rbegin(); }
ToolPath::const_reverse_iterator ToolPath::rbegin() const { return container_.rbegin(); }
ToolPath::reverse_iterator ToolPath::rend() { return container_.rend(); }
ToolPath::const_reverse_iterator ToolPath::rend() const { return container_.rend(); }
ToolPath::const_iterator ToolPath::cbegin() const { return container_.cbegin(); }
ToolPath::const_iterator ToolPath::cend() const { return container_.cend(); }
ToolPath::const_reverse_iterator ToolPath::crbegin() const { return container_.crbegin(); }
ToolPath::const_reverse_iterator ToolPath::crend() const { return container_.crend(); }

//////////////
// Capacity //
//////////////
bool ToolPath::empty() const { return container_.empty(); }
ToolPath::size_type ToolPath::size() const { return container_.size(); }
ToolPath::size_type ToolPath::max_size() const { return container_.max_size(); }
void ToolPath::reserve(size_type n) { container_.reserve(n); }
ToolPath::size_type ToolPath::capacity() const { return container_.capacity(); }
void ToolPath::shrink_to_fit() { container_.shrink_to_fit(); }

////////////////////
// Element Access //
////////////////////
ToolPath::reference ToolPath::front() { return container_.front(); }
ToolPath::const_reference ToolPath::front() const { return container_.front(); }
ToolPath::reference ToolPath::back() { return container_.back(); }
ToolPath::const_reference ToolPath::back() const { return container_.back(); }
ToolPath::reference ToolPath::at(size_type n) { return container_.at(n); }
ToolPath::const_reference ToolPath::at(size_type n) const { return container_.at(n); }
ToolPath::pointer ToolPath::data() { return container_.data(); }
ToolPath::const_pointer ToolPath::data() const { return container_.data(); }
ToolPath::reference ToolPath::operator[](size_type pos) { return container_[pos]; }
ToolPath::const_reference ToolPath::operator[](size_type pos) const { return container_[pos]; }

///////////////
// Modifiers //
///////////////
void ToolPath::clear() { container_.clear(); }

ToolPath::iterator ToolPath::insert(const_iterator p, const ToolPathSegment& x) { return container_.insert(p, x); }
ToolPath::iterator ToolPath::insert(const_iterator p, ToolPathSegment&& x) { return container_.insert(p, x); }
ToolPath::iterator ToolPath::insert(const_iterator p, std::initializer_list<ToolPathSegment> l)
{
  return container_.insert(p, l);
}

template <class... Args>
ToolPath::iterator ToolPath::emplace(const_iterator pos, Args&&... args)
{
  return container_.emplace(pos, std::forward<Args>(args)...);
}

ToolPath::iterator ToolPath::erase(const_iterator p) { return container_.erase(p); }
ToolPath::iterator ToolPath::erase(const_iterator first, const_iterator last) { return container_.erase(first, last); }
void ToolPath::push_back(const ToolPathSegment& x) { container_.push_back(x); }
void ToolPath::push_back(const ToolPathSegment&& x) { container_.push_back(x); }

template <typename... Args>
#if __cplusplus > 201402L
ToolPath::reference ToolPath::emplace_back(Args&&... args)
{
  return container_.emplace_back(std::forward<Args>(args)...);
}
#else
void ToolPath::emplace_back(Args&&... args)
{
  container_.emplace_back(std::forward<Args>(args)...);
}
#endif

void ToolPath::pop_back() { container_.pop_back(); }
void ToolPath::swap(AlignedVector<ToolPathSegment>& other) { container_.swap(other); }
// LCOV_EXCL_STOP
}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::ToolPath)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::ToolPath)
