/**
 * @file box.cpp
 * @brief Tesseract Box Geometry
 *
 * @author Levi Armstrong
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/uuid/random_generator.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/geometry.h>

namespace tesseract::geometry
{
Geometry::Geometry(GeometryType type) : type_(type), uuid_(boost::uuids::random_generator()()) {}

GeometryType Geometry::getType() const { return type_; }

void Geometry::setUUID(const boost::uuids::uuid& uuid) { uuid_ = uuid; }

const boost::uuids::uuid& Geometry::getUUID() const { return uuid_; }

bool Geometry::operator==(const Geometry& rhs) const { return (type_ == rhs.type_ && uuid_ == rhs.uuid_); }
bool Geometry::operator!=(const Geometry& rhs) const { return !operator==(rhs); }  // LCOV_EXCL_LINE

}  // namespace tesseract::geometry
