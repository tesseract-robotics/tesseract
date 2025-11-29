/**
 * @file capsule.cpp
 * @brief Tesseract Capsule Geometry
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

#include <tesseract_common/utils.h>
#include <tesseract_geometry/impl/capsule.h>

namespace tesseract_geometry
{
Capsule::Capsule(double radius, double length) : Geometry(GeometryType::CAPSULE), r_(radius), l_(length) {}

double Capsule::getRadius() const { return r_; }
double Capsule::getLength() const { return l_; }

Geometry::Ptr Capsule::clone() const { return std::make_shared<Capsule>(r_, l_); }

bool Capsule::operator==(const Capsule& rhs) const
{
  bool equal = true;
  equal &= Geometry::operator==(rhs);
  equal &= tesseract_common::almostEqualRelativeAndAbs(r_, rhs.r_);
  equal &= tesseract_common::almostEqualRelativeAndAbs(l_, rhs.l_);
  return equal;
}
bool Capsule::operator!=(const Capsule& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_geometry
