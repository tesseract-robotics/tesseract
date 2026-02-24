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

#include <tesseract/common/utils.h>
#include <tesseract/geometry/impl/box.h>

namespace tesseract::geometry
{
Box::Box(double x, double y, double z) : Geometry(GeometryType::BOX), x_(x), y_(y), z_(z) {}

double Box::getX() const { return x_; }
double Box::getY() const { return y_; }
double Box::getZ() const { return z_; }

Geometry::Ptr Box::clone() const { return std::make_shared<Box>(x_, y_, z_); }

bool Box::operator==(const Box& rhs) const
{
  bool equal = true;
  equal &= Geometry::operator==(rhs);
  equal &= tesseract::common::almostEqualRelativeAndAbs(x_, rhs.x_);
  equal &= tesseract::common::almostEqualRelativeAndAbs(y_, rhs.y_);
  equal &= tesseract::common::almostEqualRelativeAndAbs(z_, rhs.z_);
  return equal;
}
bool Box::operator!=(const Box& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::geometry
