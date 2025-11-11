/**
 * @file compound_mesh.h
 * @brief Tesseract Compound Mesh Geometry
 *
 * @author Levi Armstrong
 * @date September 14, 2024
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2024, Levi Armstrong
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
#ifndef TESSERACT_GEOMETRY_COMPOUND_MESH_H
#define TESSERACT_GEOMETRY_COMPOUND_MESH_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/polygon_mesh.h>

namespace tesseract_geometry
{
class ConvexMesh;
class Mesh;
class SDFMesh;

class CompoundMesh;
template <class Archive>
void serialize(Archive& ar, CompoundMesh& obj);

/**  @brief This is store meshes that are associated with as single resource */
class CompoundMesh : public Geometry
{
public:
  using Ptr = std::shared_ptr<CompoundMesh>;
  using ConstPtr = std::shared_ptr<const CompoundMesh>;

  CompoundMesh(std::vector<std::shared_ptr<PolygonMesh>> meshes);
  CompoundMesh(std::vector<std::shared_ptr<ConvexMesh>> meshes);
  CompoundMesh(std::vector<std::shared_ptr<Mesh>> meshes);
  CompoundMesh(std::vector<std::shared_ptr<SDFMesh>> meshes);

  /**
   * @brief Get the meshes
   * @return The meshes
   */
  const std::vector<std::shared_ptr<PolygonMesh>>& getMeshes() const;

  /**
   * @brief Get the path to file used to generate the meshs
   *
   * Note: If empty, assume it was manually generated.
   *
   * @return Absolute path to the mesh file
   */
  std::shared_ptr<const tesseract_common::Resource> getResource() const;

  /**
   * @brief Get the scale applied to file used to generate the meshs
   * @return The scale x, y, z
   */
  const Eigen::Vector3d& getScale() const;

  Geometry::Ptr clone() const override final;

  bool operator==(const CompoundMesh& rhs) const;
  bool operator!=(const CompoundMesh& rhs) const;

private:
  std::vector<std::shared_ptr<PolygonMesh>> meshes_;

  template <class Archive>
  friend void ::tesseract_geometry::serialize(Archive& ar, CompoundMesh& obj);
};

}  // namespace tesseract_geometry

#endif  // TESSERACT_GEOMETRY_COMPOUND_MESH_H
