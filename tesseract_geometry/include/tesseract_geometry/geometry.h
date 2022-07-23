/**
 * @file geometry.h
 * @brief Tesseract Geometries
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
#ifndef TESSERACT_GEOMETRY_GEOMETRY_H
#define TESSERACT_GEOMETRY_GEOMETRY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <memory>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_geometry
{
enum GeometryType
{
  UNINITIALIZED,
  SPHERE,
  CYLINDER,
  CAPSULE,
  CONE,
  BOX,
  PLANE,
  MESH,
  CONVEX_MESH,
  SDF_MESH,
  OCTREE,
  POLYGON_MESH
};
static const std::vector<std::string> GeometryTypeStrings = { "UNINITIALIZED", "SPHERE",   "CYLINDER", "CAPSULE",
                                                              "CONE",          "BOX",      "PLANE",    "MESH",
                                                              "CONVEX_MESH",   "SDF_MESH", "OCTREE",   "POLYGON_MESH" };

class Geometry
{
public:
  using Ptr = std::shared_ptr<Geometry>;
  using ConstPtr = std::shared_ptr<const Geometry>;

  explicit Geometry(GeometryType type = GeometryType::UNINITIALIZED) : type_(type) {}
  virtual ~Geometry() = default;
  Geometry(const Geometry&) = default;
  Geometry& operator=(const Geometry&) = default;
  Geometry(Geometry&&) = default;
  Geometry& operator=(Geometry&&) = default;

  /** \brief Create a copy of this shape */
  virtual Geometry::Ptr clone() const = 0;

  GeometryType getType() const { return type_; }

  bool operator==(const Geometry& rhs) const;
  bool operator!=(const Geometry& rhs) const;

private:
  /** \brief The type of the shape */
  GeometryType type_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

using Geometrys = std::vector<Geometry::Ptr>;
using GeometrysConst = std::vector<Geometry::ConstPtr>;
}  // namespace tesseract_geometry

#endif  // TESSERACT_GEOMETRY_GEOMETRY_H
