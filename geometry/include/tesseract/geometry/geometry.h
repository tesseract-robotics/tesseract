/**
 * @file geometry.h
 * @brief Tesseract Geometries
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
#ifndef TESSERACT_GEOMETRY_GEOMETRY_H
#define TESSERACT_GEOMETRY_GEOMETRY_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/uuid/uuid.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/fwd.h>

namespace tesseract::geometry
{
enum class GeometryType : std::uint8_t
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
  POLYGON_MESH,
  COMPOUND_MESH
};
static const std::vector<std::string> GeometryTypeStrings = { "UNINITIALIZED", "SPHERE",   "CYLINDER", "CAPSULE",
                                                              "CONE",          "BOX",      "PLANE",    "MESH",
                                                              "CONVEX_MESH",   "SDF_MESH", "OCTREE",   "POLYGON_MESH",
                                                              "COMPOUND_MESH" };
class Geometry;
template <class Archive>
void serialize(Archive& ar, Geometry& obj);

class Geometry
{
public:
  using Ptr = std::shared_ptr<Geometry>;
  using ConstPtr = std::shared_ptr<const Geometry>;

  explicit Geometry(GeometryType type = GeometryType::UNINITIALIZED);
  virtual ~Geometry() = default;
  Geometry(const Geometry&) = default;
  Geometry& operator=(const Geometry&) = default;
  Geometry(Geometry&&) = default;
  Geometry& operator=(Geometry&&) = default;

  /** @brief Create a copy of this shape */
  virtual Geometry::Ptr clone() const = 0;

  /** @brief Get the geometry type */
  GeometryType getType() const;

  /** @brief Set the geometry UUID */
  void setUUID(const boost::uuids::uuid& uuid);

  /** @brief Get the geometry UUID */
  const boost::uuids::uuid& getUUID() const;

  bool operator==(const Geometry& rhs) const;
  bool operator!=(const Geometry& rhs) const;

private:
  /** @brief The type of the shape */
  GeometryType type_;

  /** @brief The uuid of the shape */
  boost::uuids::uuid uuid_{};

  template <class Archive>
  friend void ::tesseract::geometry::serialize(Archive& ar, Geometry& obj);
};

using Geometrys = std::vector<Geometry::Ptr>;
using GeometrysConst = std::vector<Geometry::ConstPtr>;
}  // namespace tesseract::geometry

#endif  // TESSERACT_GEOMETRY_GEOMETRY_H
