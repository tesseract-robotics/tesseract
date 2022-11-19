/**
 * @file tool_path_segment.cpp
 * @brief Common Tesseract Tool Path Segment
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

#include <tesseract_common/tool_path_segment.h>
#include <tesseract_common/eigen_serialization.h>

namespace tesseract_common
{
ToolPathSegment::ToolPathSegment(std::string description)
  : uuid_(boost::uuids::random_generator()()), description_(std::move(description))
{
}

ToolPathSegment::ToolPathSegment(boost::uuids::uuid uuid, std::string description)
  : uuid_(std::move(uuid)), description_(std::move(description))
{
}

boost::uuids::uuid ToolPathSegment::getUUID() const { return uuid_; }

void ToolPathSegment::regenerateUUID() { uuid_ = boost::uuids::random_generator()(); }

const boost::uuids::uuid& ToolPathSegment::getParentUUID() const { return parent_uuid_; }

void ToolPathSegment::setParentUUID(const boost::uuids::uuid& uuid) { parent_uuid_ = uuid; }

void ToolPathSegment::setDescription(const std::string& desc) { description_ = desc; }
const std::string& ToolPathSegment::getDescription() const { return description_; }

bool ToolPathSegment::operator==(const ToolPathSegment& rhs) const
{
  bool equal = true;
  equal &= (uuid_ == rhs.uuid_);                // NOLINT
  equal &= (parent_uuid_ == rhs.parent_uuid_);  // NOLINT
  equal &= (description_ == rhs.description_);  // NOLINT
  equal &= (container_.size() == rhs.container_.size());
  if (equal)
  {
    for (std::size_t i = 0; i < container_.size(); ++i)
    {
      equal &= (container_[i].isApprox(rhs.container_[i], std::numeric_limits<float>::epsilon()));

      if (!equal)
        break;
    }
  }
  return equal;
}

bool ToolPathSegment::operator!=(const ToolPathSegment& rhs) const { return !operator==(rhs); }

template <class Archive>
void ToolPathSegment::serialize(Archive& ar, const unsigned int /*version*/)
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
ToolPathSegment::iterator ToolPathSegment::begin() { return container_.begin(); }
ToolPathSegment::const_iterator ToolPathSegment::begin() const { return container_.begin(); }
ToolPathSegment::iterator ToolPathSegment::end() { return container_.end(); }
ToolPathSegment::const_iterator ToolPathSegment::end() const { return container_.end(); }
ToolPathSegment::reverse_iterator ToolPathSegment::rbegin() { return container_.rbegin(); }
ToolPathSegment::const_reverse_iterator ToolPathSegment::rbegin() const { return container_.rbegin(); }
ToolPathSegment::reverse_iterator ToolPathSegment::rend() { return container_.rend(); }
ToolPathSegment::const_reverse_iterator ToolPathSegment::rend() const { return container_.rend(); }
ToolPathSegment::const_iterator ToolPathSegment::cbegin() const { return container_.cbegin(); }
ToolPathSegment::const_iterator ToolPathSegment::cend() const { return container_.cend(); }
ToolPathSegment::const_reverse_iterator ToolPathSegment::crbegin() const { return container_.crbegin(); }
ToolPathSegment::const_reverse_iterator ToolPathSegment::crend() const { return container_.crend(); }

//////////////
// Capacity //
//////////////
bool ToolPathSegment::empty() const { return container_.empty(); }
ToolPathSegment::size_type ToolPathSegment::size() const { return container_.size(); }
ToolPathSegment::size_type ToolPathSegment::max_size() const { return container_.max_size(); }
void ToolPathSegment::reserve(size_type n) { container_.reserve(n); }
ToolPathSegment::size_type ToolPathSegment::capacity() const { return container_.capacity(); }
void ToolPathSegment::shrink_to_fit() { container_.shrink_to_fit(); }

////////////////////
// Element Access //
////////////////////
ToolPathSegment::reference ToolPathSegment::front() { return container_.front(); }
ToolPathSegment::const_reference ToolPathSegment::front() const { return container_.front(); }
ToolPathSegment::reference ToolPathSegment::back() { return container_.back(); }
ToolPathSegment::const_reference ToolPathSegment::back() const { return container_.back(); }
ToolPathSegment::reference ToolPathSegment::at(size_type n) { return container_.at(n); }
ToolPathSegment::const_reference ToolPathSegment::at(size_type n) const { return container_.at(n); }
ToolPathSegment::pointer ToolPathSegment::data() { return container_.data(); }
ToolPathSegment::const_pointer ToolPathSegment::data() const { return container_.data(); }
ToolPathSegment::reference ToolPathSegment::operator[](size_type pos) { return container_[pos]; }
ToolPathSegment::const_reference ToolPathSegment::operator[](size_type pos) const { return container_[pos]; }

///////////////
// Modifiers //
///////////////
void ToolPathSegment::clear() { container_.clear(); }

ToolPathSegment::iterator ToolPathSegment::insert(const_iterator p, const Eigen::Isometry3d& x)
{
  return container_.insert(p, x);
}
ToolPathSegment::iterator ToolPathSegment::insert(const_iterator p, Eigen::Isometry3d&& x)
{
  return container_.insert(p, x);
}
ToolPathSegment::iterator ToolPathSegment::insert(const_iterator p, std::initializer_list<Eigen::Isometry3d> l)
{
  return container_.insert(p, l);
}

template <class... Args>
ToolPathSegment::iterator ToolPathSegment::emplace(const_iterator pos, Args&&... args)
{
  return container_.emplace(pos, std::forward<Args>(args)...);
}

ToolPathSegment::iterator ToolPathSegment::erase(const_iterator p) { return container_.erase(p); }
ToolPathSegment::iterator ToolPathSegment::erase(const_iterator first, const_iterator last)
{
  return container_.erase(first, last);
}
void ToolPathSegment::push_back(const Eigen::Isometry3d& x) { container_.push_back(x); }
void ToolPathSegment::push_back(const Eigen::Isometry3d&& x) { container_.push_back(x); }

template <typename... Args>
#if __cplusplus > 201402L
ToolPathSegment::reference ToolPathSegment::emplace_back(Args&&... args)
{
  return container_.emplace_back(std::forward<Args>(args)...);
}
#else
void ToolPathSegment::emplace_back(Args&&... args)
{
  container_.emplace_back(std::forward<Args>(args)...);
}
#endif

void ToolPathSegment::pop_back() { container_.pop_back(); }
void ToolPathSegment::swap(VectorIsometry3d& other) { container_.swap(other); }

// LCOV_EXCL_STOP
}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::ToolPathSegment)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::ToolPathSegment)
