/**
 * @file sdf_mesh.h
 * @brief Tesseract SDF Mesh Geometry
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
#ifndef TESSERACT_GEOMETRY_SDF_MESH_H
#define TESSERACT_GEOMETRY_SDF_MESH_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>
#include <tesseract_geometry/impl/mesh_material.h>
#include <tesseract_geometry/impl/polygon_mesh.h>

#ifdef SWIG
%shared_ptr(tesseract_geometry::SDFMesh)
%template(SDFMeshVector) std::vector<std::shared_ptr<tesseract_geometry::SDFMesh> >;
#endif  // SWIG

namespace tesseract_geometry
{
class SDFMesh : public PolygonMesh
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<SDFMesh>;
  using ConstPtr = std::shared_ptr<const SDFMesh>;

  /**
   * @brief SDF Mesh geometry
   * @param vertices A vector of vertices associated with the mesh
   * @param triangles A vector of face indices where the first index indicates the number of vertices associated
   *                  with the face followed by the vertices index in parameter vertices. For example a triangle
   *                  has three vertices so there should be four inputs where the first should be 3 indicating there are
   *                  three vertices that define this face followed by three indices.
   * @param resource A resource locator for locating resource
   * @param scale Scale the mesh
   * @param normals A vector of normals for the vertices (optional)
   * @param vertex_colors A vector of colors (RGBA) for the vertices (optional)
   * @param mesh_material A MeshMaterial describing the color and material properties of the mesh (optional)
   * @param mesh_textures A vector of MeshTexture to apply to the mesh (optional)
   */
  SDFMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
          std::shared_ptr<const Eigen::VectorXi> triangles,
          tesseract_common::Resource::Ptr resource = nullptr,
          const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
          std::shared_ptr<const tesseract_common::VectorVector3d> normals = nullptr,
          std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors = nullptr,
          MeshMaterial::Ptr mesh_material = nullptr,
          std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures = nullptr)
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
    if ((getFaceCount() * 4) != getFaces()->size())
      std::throw_with_nested(std::runtime_error("Mesh is not triangular"));  // LCOV_EXCL_LINE
  }

  /**
   * @brief SDF Mesh geometry
   * @param vertices A vector of vertices associated with the mesh
   * @param triangles A vector of face indices where the first index indicates the number of vertices associated
   *                  with the face followed by the vertices index in parameter vertices. For example a triangle
   *                  has three vertices so there should be four inputs where the first should be 3 indicating there are
   *                  three vertices that define this face followed by three indices.
   * @param triangle_count Provide the number of faces. This is faster because it does not need to loop over triangles.
   * @param resource A resource locator for locating resource
   * @param scale Scale the mesh
   * @param normals A vector of normals for the vertices (optional)
   * @param vertex_colors A vector of colors (RGBA) for the vertices (optional)
   * @param mesh_material A MeshMaterial describing the color and material properties of the mesh (optional)
   * @param mesh_textures A vector of MeshTexture to apply to the mesh (optional)
   */
  SDFMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
          std::shared_ptr<const Eigen::VectorXi> triangles,
          int triangle_count,
          tesseract_common::Resource::ConstPtr resource = nullptr,
          const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
          std::shared_ptr<const tesseract_common::VectorVector3d> normals = nullptr,
          std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors = nullptr,
          MeshMaterial::Ptr mesh_material = nullptr,
          std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures = nullptr)
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
    if ((getFaceCount() * 4) != getFaces()->size())
      std::throw_with_nested(std::runtime_error("Mesh is not triangular"));  // LCOV_EXCL_LINE
  }

  ~SDFMesh() override = default;
  SDFMesh(const SDFMesh&) = delete;
  SDFMesh& operator=(const SDFMesh&) = delete;
  SDFMesh(SDFMesh&&) = delete;
  SDFMesh& operator=(SDFMesh&&) = delete;

  /**
   * @brief Get SDF mesh Triangles
   * @return A vector of triangle indices
   */
  [[deprecated("Please use getFaces() instead")]] const std::shared_ptr<const Eigen::VectorXi>& getTriangles() const
  {
    return getFaces();
  }

  /**
   * @brief Get triangle count
   * @return Number of triangles
   */
  [[deprecated("Please use getFaceCount() instead")]] int getTriangleCount() const { return getFaceCount(); }

  Geometry::Ptr clone() const override
  {
    return std::make_shared<SDFMesh>(getVertices(), getFaces(), getFaceCount(), getResource(), getScale());
  }

private:
};
}  // namespace tesseract_geometry
#endif
