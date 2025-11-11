/**
 * @file Plane.cpp
 * @brief Tesseract Plane Geometry
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
#include <tesseract_geometry/impl/plane.h>

namespace tesseract_geometry
{
Plane::Plane(double a, double b, double c, double d) : Geometry(GeometryType::PLANE), a_(a), b_(b), c_(c), d_(d) {}

double Plane::getA() const { return a_; }
double Plane::getB() const { return b_; }
double Plane::getC() const { return c_; }
double Plane::getD() const { return d_; }

Geometry::Ptr Plane::clone() const { return std::make_shared<Plane>(a_, b_, c_, d_); }

bool Plane::operator==(const Plane& rhs) const
{
  bool equal = true;
  equal &= Geometry::operator==(rhs);
  equal &= tesseract_common::almostEqualRelativeAndAbs(a_, rhs.a_);
  equal &= tesseract_common::almostEqualRelativeAndAbs(b_, rhs.b_);
  equal &= tesseract_common::almostEqualRelativeAndAbs(c_, rhs.c_);
  equal &= tesseract_common::almostEqualRelativeAndAbs(d_, rhs.d_);
  return equal;
}
bool Plane::operator!=(const Plane& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_geometry
