/**
 * @file ConvexMesh.cpp
 * @brief Tesseract ConvexMesh Geometry
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

#include <tesseract_geometry/impl/convex_mesh.h>
#include <tesseract_geometry/impl/mesh_material.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract::geometry
{
ConvexMesh::ConvexMesh(std::shared_ptr<const tesseract::common::VectorVector3d> vertices,
                       std::shared_ptr<const Eigen::VectorXi> faces,
                       tesseract::common::Resource::ConstPtr resource,
                       const Eigen::Vector3d& scale,
                       std::shared_ptr<const tesseract::common::VectorVector3d> normals,
                       std::shared_ptr<const tesseract::common::VectorVector4d> vertex_colors,
                       MeshMaterial::Ptr mesh_material,
                       std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures)
  : PolygonMesh(std::move(vertices),
                std::move(faces),
                std::move(resource),
                scale,
                std::move(normals),
                std::move(vertex_colors),
                std::move(mesh_material),
                std::move(mesh_textures),
                GeometryType::CONVEX_MESH)
{
}

ConvexMesh::ConvexMesh(std::shared_ptr<const tesseract::common::VectorVector3d> vertices,
                       std::shared_ptr<const Eigen::VectorXi> faces,
                       int face_count,
                       tesseract::common::Resource::ConstPtr resource,
                       const Eigen::Vector3d& scale,
                       std::shared_ptr<const tesseract::common::VectorVector3d> normals,
                       std::shared_ptr<const tesseract::common::VectorVector4d> vertex_colors,
                       MeshMaterial::Ptr mesh_material,
                       std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures)
  : PolygonMesh(std::move(vertices),
                std::move(faces),
                face_count,
                std::move(resource),
                scale,
                std::move(normals),
                std::move(vertex_colors),
                std::move(mesh_material),
                std::move(mesh_textures),
                GeometryType::CONVEX_MESH)
{
}

ConvexMesh::CreationMethod ConvexMesh::getCreationMethod() const { return creation_method_; }

void ConvexMesh::setCreationMethod(CreationMethod method) { creation_method_ = method; }

Geometry::Ptr ConvexMesh::clone() const
{
  return std::make_shared<ConvexMesh>(getVertices(), getFaces(), getFaceCount(), getResource(), getScale());
}

bool ConvexMesh::operator==(const ConvexMesh& rhs) const
{
  bool equal = true;
  equal &= PolygonMesh::operator==(rhs);
  equal &= creation_method_ == rhs.creation_method_;
  return equal;
}
bool ConvexMesh::operator!=(const ConvexMesh& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::geometry
