/**
 * @file capsule.h
 * @brief Tesseract Capsule Geometry
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
#ifndef TESSERACT_GEOMETRY_CAPSULE_H
#define TESSERACT_GEOMETRY_CAPSULE_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/geometry.h>

namespace tesseract::geometry
{
class Capsule;
template <class Archive>
void serialize(Archive& ar, Capsule& obj);

class Capsule : public Geometry
{
public:
  using Ptr = std::shared_ptr<Capsule>;
  using ConstPtr = std::shared_ptr<const Capsule>;

  Capsule(double radius, double length);
  Capsule() = default;
  ~Capsule() override = default;

  double getRadius() const;
  double getLength() const;

  Geometry::Ptr clone() const override final;
  bool operator==(const Capsule& rhs) const;
  bool operator!=(const Capsule& rhs) const;

private:
  double r_{ 0 };
  double l_{ 0 };

  template <class Archive>
  friend void ::tesseract::geometry::serialize(Archive& ar, Capsule& obj);
};
}  // namespace tesseract::geometry

#endif  // TESSERACT_GEOMETRY_CAPSULE_H
