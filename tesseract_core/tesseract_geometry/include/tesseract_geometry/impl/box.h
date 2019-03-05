/**
 * @file box.h
 * @brief Tesseract Box Geometry
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
#ifndef TESSERACT_GEOMETRY_BOX_H
#define TESSERACT_GEOMETRY_BOX_H

#include <tesseract_geometry/macros.h>
TESSERACT_GEOMETRY_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_GEOMETRY_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace tesseract_geometry
{
  class Box;
  typedef std::shared_ptr<Box> BoxPtr;
  typedef std::shared_ptr<const Box> BoxConstPtr;

  class Box : public Geometry
  {
  public:
    Box(double x, double y, double z) : Geometry(GeometryType::BOX), x_(x), y_(y), z_(z) {}
    ~Box() override = default;

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getZ() const { return z_; }

    GeometryPtr clone() const override { return BoxPtr(new Box(x_, y_, z_)); }

  private:
    double x_;
    double y_;
    double z_;
  };
}
#endif
