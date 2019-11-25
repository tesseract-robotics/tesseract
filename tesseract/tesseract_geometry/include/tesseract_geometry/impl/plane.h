/**
 * @file plane.h
 * @brief Tesseract Plane Geometry
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
#ifndef TESSERACT_GEOMETRY_PLANE_H
#define TESSERACT_GEOMETRY_PLANE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace tesseract_geometry
{
class Plane : public Geometry
{
public:
  using Ptr = std::shared_ptr<Plane>;
  using ConstPtr = std::shared_ptr<const Plane>;

  Plane(double a, double b, double c, double d) : Geometry(GeometryType::PLANE), a_(a), b_(b), c_(c), d_(d) {}
  ~Plane() override = default;
  Plane(const Plane&) = delete;
  Plane& operator=(const Plane&) = delete;
  Plane(Plane&&) = delete;
  Plane& operator=(Plane&&) = delete;

  double getA() const { return a_; }
  double getB() const { return b_; }
  double getC() const { return c_; }
  double getD() const { return d_; }

  Geometry::Ptr clone() const override { return std::make_shared<Plane>(a_, b_, c_, d_); }

private:
  double a_;
  double b_;
  double c_;
  double d_;
};
}  // namespace tesseract_geometry
#endif
