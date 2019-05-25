/**
 * @file sphere.h
 * @brief Tesseract Sphere Geometry
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
#ifndef TESSERACT_GEOMETRY_SPHERE_H
#define TESSERACT_GEOMETRY_SPHERE_H

#include <tesseract_geometry/macros.h>
TESSERACT_GEOMETRY_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_GEOMETRY_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace tesseract_geometry
{
  class Sphere;
  typedef std::shared_ptr<Sphere> SpherePtr;
  typedef std::shared_ptr<const Sphere> SphereConstPtr;

  class Sphere : public Geometry
  {
  public:
    explicit Sphere(double r) : Geometry(GeometryType::SPHERE), r_(r) {}
    ~Sphere() override = default;

    double getRadius() const { return r_; }

    GeometryPtr clone() const override { return SpherePtr(new Sphere(r_)); }

  private:
    double r_;
  };
}
#endif
