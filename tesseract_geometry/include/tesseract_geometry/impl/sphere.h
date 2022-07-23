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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace tesseract_geometry
{
class Sphere : public Geometry
{
public:
  using Ptr = std::shared_ptr<Sphere>;
  using ConstPtr = std::shared_ptr<const Sphere>;

  explicit Sphere(double r) : Geometry(GeometryType::SPHERE), r_(r) {}
  Sphere() = default;
  ~Sphere() override = default;

  double getRadius() const { return r_; }

  Geometry::Ptr clone() const override final { return std::make_shared<Sphere>(r_); }
  bool operator==(const Sphere& rhs) const;
  bool operator!=(const Sphere& rhs) const;

private:
  double r_{ 0 };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_geometry

#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_geometry::Sphere, "Sphere")
BOOST_CLASS_TRACKING(tesseract_geometry::Sphere, boost::serialization::track_never)
#endif
