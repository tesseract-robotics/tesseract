/**
 * @file any.cpp
 * @brief This a boost serializable any
 *
 * @author Levi Armstrong
 * @date February 27, 2021
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

#include <tesseract_common/any_poly.h>

namespace tesseract_common
{
AnyInterface::~AnyInterface() = default;

std::type_index AnyInterface::getType() const { return typeid(*this); }

// Operators
bool AnyInterface::operator==(const AnyInterface& rhs) const { return equals(rhs); }

// LCOV_EXCL_START
bool AnyInterface::operator!=(const AnyInterface& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

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

}  // namespace tesseract_common
