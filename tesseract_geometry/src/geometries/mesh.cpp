/**
 * @file Mesh.cpp
 * @brief Tesseract Mesh Geometry
 *
 * @author Levi Armstrong
 * @date March 16, 2022
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

#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/impl/mesh_material.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract::geometry
{
Mesh::Mesh(std::shared_ptr<const tesseract::common::VectorVector3d> vertices,
           std::shared_ptr<const Eigen::VectorXi> triangles,
           std::shared_ptr<const tesseract::common::Resource> resource,
           const Eigen::Vector3d& scale,
           std::shared_ptr<const tesseract::common::VectorVector3d> normals,
           std::shared_ptr<const tesseract::common::VectorVector4d> vertex_colors,
           std::shared_ptr<MeshMaterial> mesh_material,
           std::shared_ptr<const std::vector<std::shared_ptr<MeshTexture>>> mesh_textures)
  : PolygonMesh(std::move(vertices),
                std::move(triangles),
                std::move(resource),
                scale,
                std::move(normals),
                std::move(vertex_colors),
                std::move(mesh_material),
                std::move(mesh_textures),
                GeometryType::MESH)
{
  if ((static_cast<long>(getFaceCount()) * 4) != getFaces()->size())
    std::throw_with_nested(std::runtime_error("Mesh is not triangular"));  // LCOV_EXCL_LINE
}

Mesh::Mesh(std::shared_ptr<const tesseract::common::VectorVector3d> vertices,
           std::shared_ptr<const Eigen::VectorXi> triangles,
           int triangle_count,
           std::shared_ptr<const tesseract::common::Resource> resource,
           const Eigen::Vector3d& scale,
           std::shared_ptr<const tesseract::common::VectorVector3d> normals,
           std::shared_ptr<const tesseract::common::VectorVector4d> vertex_colors,
           std::shared_ptr<MeshMaterial> mesh_material,
           std::shared_ptr<const std::vector<std::shared_ptr<MeshTexture>>> mesh_textures)
  : PolygonMesh(std::move(vertices),
                std::move(triangles),
                triangle_count,
                std::move(resource),
                scale,
                std::move(normals),
                std::move(vertex_colors),
                std::move(mesh_material),
                std::move(mesh_textures),
                GeometryType::MESH)
{
  if ((static_cast<long>(getFaceCount()) * 4) != getFaces()->size())
    std::throw_with_nested(std::runtime_error("Mesh is not triangular"));  // LCOV_EXCL_LINE
}

Geometry::Ptr Mesh::clone() const
{
  // getMaterial returns a pointer-to-const, so deference and make_shared, but also guard against nullptr
  std::shared_ptr<Mesh> ptr;
  if (getMaterial() != nullptr)
  {
    ptr = std::make_shared<Mesh>(getVertices(),
                                 getFaces(),
                                 getFaceCount(),
                                 getResource(),
                                 getScale(),
                                 getNormals(),
                                 getVertexColors(),
                                 std::make_shared<MeshMaterial>(*getMaterial()),
                                 getTextures());
  }
  else
  {
    ptr = std::make_shared<Mesh>(getVertices(),
                                 getFaces(),
                                 getFaceCount(),
                                 getResource(),
                                 getScale(),
                                 getNormals(),
                                 getVertexColors(),
                                 nullptr,
                                 getTextures());
  }
  return ptr;
}

bool Mesh::operator==(const Mesh& rhs) const
{
  bool equal = true;
  equal &= PolygonMesh::operator==(rhs);
  return equal;
}
bool Mesh::operator!=(const Mesh& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::geometry
