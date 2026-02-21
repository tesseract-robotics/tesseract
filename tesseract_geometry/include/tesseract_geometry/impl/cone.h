/**
 * @file cone.h
 * @brief Tesseract Cone Geometry
 *
 * @author Levi Armstrong
 * @date January 18, 2018
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
#ifndef TESSERACT_GEOMETRY_CONE_H
#define TESSERACT_GEOMETRY_CONE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace tesseract::geometry
{
class Cone;
template <class Archive>
void serialize(Archive& ar, Cone& obj);

class Cone : public Geometry
{
public:
  using Ptr = std::shared_ptr<Cone>;
  using ConstPtr = std::shared_ptr<const Cone>;

  Cone(double radius, double length);
  Cone() = default;
  ~Cone() override = default;

  double getRadius() const;
  double getLength() const;

  Geometry::Ptr clone() const override final;
  bool operator==(const Cone& rhs) const;
  bool operator!=(const Cone& rhs) const;

private:
  double r_{ 0 };
  double l_{ 0 };

  template <class Archive>
  friend void ::tesseract::geometry::serialize(Archive& ar, Cone& obj);
};
}  // namespace tesseract::geometry

#endif
