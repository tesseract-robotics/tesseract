/**
 * @file cylinder.h
 * @brief Tesseract Cylinder Geometry
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_GEOMETRY_CYLINDER_H
#define TESSERACT_GEOMETRY_CYLINDER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace tesseract_geometry
{
class Cylinder : public Geometry
{
public:
  using Ptr = std::shared_ptr<Cylinder>;
  using ConstPtr = std::shared_ptr<const Cylinder>;

  Cylinder(double r, double l) : Geometry(GeometryType::CYLINDER), r_(r), l_(l) {}
  ~Cylinder() override = default;
  Cylinder(const Cylinder&) = delete;
  Cylinder& operator=(const Cylinder&) = delete;
  Cylinder(Cylinder&&) = delete;
  Cylinder& operator=(Cylinder&&) = delete;

  double getRadius() const { return r_; }
  double getLength() const { return l_; }

  Geometry::Ptr clone() const override { return std::make_shared<Cylinder>(r_, l_); }

private:
  double r_;
  double l_;
};
}  // namespace tesseract_geometry
#endif
