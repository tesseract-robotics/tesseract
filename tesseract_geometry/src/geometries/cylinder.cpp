/**
 * @file Cylinder.cpp
 * @brief Tesseract Cylinder Geometry
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
#include <tesseract_geometry/impl/cylinder.h>

namespace tesseract::geometry
{
Cylinder::Cylinder(double radius, double length) : Geometry(GeometryType::CYLINDER), r_(radius), l_(length) {}

double Cylinder::getRadius() const { return r_; }
double Cylinder::getLength() const { return l_; }

Geometry::Ptr Cylinder::clone() const { return std::make_shared<Cylinder>(r_, l_); }

bool Cylinder::operator==(const Cylinder& rhs) const
{
  bool equal = true;
  equal &= Geometry::operator==(rhs);
  equal &= tesseract::common::almostEqualRelativeAndAbs(r_, rhs.r_);
  equal &= tesseract::common::almostEqualRelativeAndAbs(l_, rhs.l_);
  return equal;
}
bool Cylinder::operator!=(const Cylinder& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::geometry
