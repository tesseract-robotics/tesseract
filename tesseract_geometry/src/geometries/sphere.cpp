/**
 * @file Sphere.cpp
 * @brief Tesseract Sphere Geometry
 *
 * @author Levi Armstrong
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

#include <tesseract_common/utils.h>
#include <tesseract_geometry/impl/sphere.h>

namespace tesseract_geometry
{
Sphere::Sphere(double r) : Geometry(GeometryType::SPHERE), r_(r) {}

double Sphere::getRadius() const { return r_; }

Geometry::Ptr Sphere::clone() const { return std::make_shared<Sphere>(r_); }

bool Sphere::operator==(const Sphere& rhs) const
{
  bool equal = true;
  equal &= Geometry::operator==(rhs);
  equal &= tesseract_common::almostEqualRelativeAndAbs(r_, rhs.r_);
  return equal;
}
bool Sphere::operator!=(const Sphere& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_geometry
