/**
 * @file SDFMesh.cpp
 * @brief Tesseract SDFMesh Geometry
 *
 * @author Levi Armstrong
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/sdf_mesh.h>
#include <tesseract_geometry/impl/mesh_material.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract_geometry
{
SDFMesh::SDFMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
                 std::shared_ptr<const Eigen::VectorXi> triangles,
                 std::shared_ptr<const tesseract_common::Resource> resource,
                 const Eigen::Vector3d& scale,
                 std::shared_ptr<const tesseract_common::VectorVector3d> normals,
                 std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors,
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
                GeometryType::SDF_MESH)
{
  if ((static_cast<long>(getFaceCount()) * 4) != getFaces()->size())
    std::throw_with_nested(std::runtime_error("Mesh is not triangular"));  // LCOV_EXCL_LINE
}

SDFMesh::SDFMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
                 std::shared_ptr<const Eigen::VectorXi> triangles,
                 int triangle_count,
                 std::shared_ptr<const tesseract_common::Resource> resource,
                 const Eigen::Vector3d& scale,
                 std::shared_ptr<const tesseract_common::VectorVector3d> normals,
                 std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors,
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
                GeometryType::SDF_MESH)
{
  if ((static_cast<long>(getFaceCount()) * 4) != getFaces()->size())
    std::throw_with_nested(std::runtime_error("Mesh is not triangular"));  // LCOV_EXCL_LINE
}

Geometry::Ptr SDFMesh::clone() const
{
  return std::make_shared<SDFMesh>(getVertices(), getFaces(), getFaceCount(), getResource(), getScale());
}

bool SDFMesh::operator==(const SDFMesh& rhs) const
{
  bool equal = true;
  equal &= Geometry::operator==(rhs);
  return equal;
}
bool SDFMesh::operator!=(const SDFMesh& rhs) const { return !operator==(rhs); }

template <class Archive>
void SDFMesh::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PolygonMesh);
}
}  // namespace tesseract_geometry

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_geometry::SDFMesh)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_geometry::SDFMesh)
