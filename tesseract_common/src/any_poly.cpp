/**
 * @file any.cpp
 * @brief This a boost serializable any
 *
 * @author Levi Armstrong
 * @date February 27, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
#if (BOOST_VERSION >= 107400) && (BOOST_VERSION < 107500)
#include <boost/serialization/library_version_type.hpp>
#endif
#include <boost/serialization/unordered_map.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/any_poly.h>

namespace tesseract_common
{
std::type_index AnyInterface::getType() const { return typeid(*this); }

// Operators
bool AnyInterface::operator==(const AnyInterface& rhs) const { return equals(rhs); }

// LCOV_EXCL_START
bool AnyInterface::operator!=(const AnyInterface& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void AnyInterface::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

AnyPoly::AnyPoly(const AnyPoly& other)
{
  if (other.impl_)
    impl_ = other.impl_->clone();  // Deep copy
}

AnyPoly& AnyPoly::operator=(const AnyPoly& other)
{
  if (this != &other)
    impl_ = other.impl_ ? other.impl_->clone() : nullptr;

  return *this;
}

AnyPoly::AnyPoly(const AnyInterface& impl) : impl_(impl.clone()) {}

std::type_index AnyPoly::getType() const
{
  if (impl_ == nullptr)
    return typeid(nullptr);

  return impl_->getType();
}

bool AnyPoly::isNull() const { return (impl_ == nullptr); }
AnyInterface& AnyPoly::get() { return *impl_; }
const AnyInterface& AnyPoly::get() const { return *impl_; }

bool AnyPoly::operator==(const AnyPoly& rhs) const
{
  if (impl_ == nullptr && rhs.impl_ == nullptr)
    return true;

  if (impl_ == nullptr || rhs.impl_ == nullptr)
    return false;

  if (getType() != rhs.getType())
    return false;

  return (*impl_ == *rhs.impl_);
}
// LCOV_EXCL_START
bool AnyPoly::operator!=(const AnyPoly& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void AnyPoly::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("impl", impl_);
}
}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::AnyInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::AnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::AnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::BoolAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::BoolAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::IntAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::IntAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::UnsignedAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::UnsignedAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::DoubleAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::DoubleAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::FloatAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::FloatAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::StringAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::StringAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::SizeTAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::SizeTAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::UMapStringStringAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::UMapStringStringAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::UMapStringBoolAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::UMapStringBoolAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::UMapStringIntAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::UMapStringIntAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::UMapStringUnsignedAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::UMapStringUnsignedAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::UMapStringDoubleAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::UMapStringDoubleAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::UMapStringFloatAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::UMapStringFloatAnyPoly)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::UMapStringSizeTAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::UMapStringSizeTAnyPoly)
