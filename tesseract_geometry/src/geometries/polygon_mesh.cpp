/**
 * @file polygon_mesh.cpp
 * @brief Tesseract PolygonMesh Geometry
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
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

#include <tesseract_common/utils.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_geometry/impl/polygon_mesh.h>
#include <tesseract_geometry/impl/mesh_material.h>

namespace tesseract_geometry
{
PolygonMesh::PolygonMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
                         std::shared_ptr<const Eigen::VectorXi> faces,
                         std::shared_ptr<const tesseract_common::Resource> resource,
                         const Eigen::Vector3d& scale,  // NOLINT
                         std::shared_ptr<const tesseract_common::VectorVector3d> normals,
                         std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors,
                         std::shared_ptr<MeshMaterial> mesh_material,
                         std::shared_ptr<const std::vector<std::shared_ptr<MeshTexture>>> mesh_textures,
                         GeometryType type)
  : Geometry(type)
  , vertices_(std::move(vertices))
  , faces_(std::move(faces))
  , vertex_count_(static_cast<int>(vertices_->size()))
  , resource_(std::move(resource))
  , scale_(scale)
  , normals_(std::move(normals))
  , vertex_colors_(std::move(vertex_colors))
  , mesh_material_(std::move(mesh_material))
  , mesh_textures_(std::move(mesh_textures))
{
  for (int i = 0; i < faces_->size(); ++i)
  {
    ++face_count_;
    int num_verts = (*faces_)(i);  // NOLINT
    i += num_verts;
  }
}

PolygonMesh::PolygonMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
                         std::shared_ptr<const Eigen::VectorXi> faces,
                         int face_count,
                         tesseract_common::Resource::ConstPtr resource,
                         const Eigen::Vector3d& scale,  // NOLINT
                         std::shared_ptr<const tesseract_common::VectorVector3d> normals,
                         std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors,
                         MeshMaterial::Ptr mesh_material,
                         std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures,
                         GeometryType type)
  : Geometry(type)
  , vertices_(std::move(vertices))
  , faces_(std::move(faces))
  , vertex_count_(static_cast<int>(vertices_->size()))
  , face_count_(face_count)
  , resource_(std::move(resource))
  , scale_(scale)
  , normals_(std::move(normals))
  , vertex_colors_(std::move(vertex_colors))
  , mesh_material_(std::move(mesh_material))
  , mesh_textures_(std::move(mesh_textures))
{
}

const std::shared_ptr<const tesseract_common::VectorVector3d>& PolygonMesh::getVertices() const { return vertices_; }

const std::shared_ptr<const Eigen::VectorXi>& PolygonMesh::getFaces() const { return faces_; }

int PolygonMesh::getVertexCount() const { return vertex_count_; }

int PolygonMesh::getFaceCount() const { return face_count_; }

tesseract_common::Resource::ConstPtr PolygonMesh::getResource() const { return resource_; }

const Eigen::Vector3d& PolygonMesh::getScale() const { return scale_; }

const std::shared_ptr<const tesseract_common::VectorVector3d>& PolygonMesh::getNormals() const { return normals_; }

const std::shared_ptr<const tesseract_common::VectorVector4d>& PolygonMesh::getVertexColors() const
{
  return vertex_colors_;
}

MeshMaterial::ConstPtr PolygonMesh::getMaterial() const { return mesh_material_; }

const std::shared_ptr<const std::vector<MeshTexture::Ptr>>& PolygonMesh::getTextures() const { return mesh_textures_; }

Geometry::Ptr PolygonMesh::clone() const
{
  return std::make_shared<PolygonMesh>(vertices_, faces_, face_count_, resource_, scale_);
}

bool PolygonMesh::operator==(const PolygonMesh& rhs) const
{
  bool equal = true;
  equal &= Geometry::operator==(rhs);
  equal &= vertex_count_ == rhs.vertex_count_;
  equal &= face_count_ == rhs.face_count_;
  equal &= tesseract_common::almostEqualRelativeAndAbs(scale_, rhs.scale_);
  /// @todo Finish PolygonMesh == operator
  return equal;
}
bool PolygonMesh::operator!=(const PolygonMesh& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_geometry
