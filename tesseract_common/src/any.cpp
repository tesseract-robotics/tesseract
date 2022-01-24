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

#include <tesseract_common/any.h>

namespace tesseract_common
{
template <class Archive>
void detail_any::AnyInnerBase::serialize(Archive& /*ar*/, const unsigned int /*version*/)  // NOLINT
{
}

Any::Any()  // NOLINT
  : any_type_(nullptr)
{
}

Any::Any(const Any& other) : any_type_(other.any_type_->clone()) {}

Any::Any(Any&& other) noexcept { any_type_.swap(other.any_type_); }

Any& Any::operator=(Any&& other) noexcept
{
  any_type_.swap(other.any_type_);
  return (*this);
}

Any& Any::operator=(const Any& other)
{
  (*this) = Any(other);
  return (*this);
}

bool Any::operator==(const Any& rhs) const { return any_type_->operator==(*rhs.any_type_); }

bool Any::operator!=(const Any& rhs) const { return !operator==(rhs); }

template <class Archive>
void Any::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("any_type", any_type_);
}

}  // namespace tesseract_common

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_common::detail_any::AnyInnerBase::serialize(boost::archive::xml_oarchive& ar,
                                                                    const unsigned int version);
template void tesseract_common::detail_any::AnyInnerBase::serialize(boost::archive::xml_iarchive& ar,
                                                                    const unsigned int version);

template void tesseract_common::Any::serialize(boost::archive::xml_oarchive& ar, const unsigned int version);
template void tesseract_common::Any::serialize(boost::archive::xml_iarchive& ar, const unsigned int version);
