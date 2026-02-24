/**
 * @file mesh.h
 * @brief Tesseract Mesh Geometry
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
#ifndef TESSERACT_GEOMETRY_MESH_H
#define TESSERACT_GEOMETRY_MESH_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/impl/polygon_mesh.h>

namespace tesseract::geometry
{
class Mesh : public PolygonMesh
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<Mesh>;
  using ConstPtr = std::shared_ptr<const Mesh>;

  /**
   * @brief Mesh geometry
   * @param vertices A vector of vertices associated with the mesh
   * @param triangles A vector of face indices where the first index indicates the number of vertices associated
   *                  with the face followed by the vertex index in parameter vertices. For example a triangle
   *                  has three vertices so there should be four inputs where the first should be 3 indicating there are
   *                  three vertices that define this face followed by three indices.
   * @param resource A resource locator for locating resource
   * @param scale Scale the mesh
   * @param normals A vector of normals for the vertices (optional)
   * @param vertex_colors A vector of colors (RGBA) for the vertices (optional)
   * @param mesh_material A MeshMaterial describing the color and material properties of the mesh (optional)
   * @param mesh_textures A vector of MeshTexture to apply to the mesh (optional)
   */
  Mesh(std::shared_ptr<const tesseract::common::VectorVector3d> vertices,
       std::shared_ptr<const Eigen::VectorXi> triangles,
       std::shared_ptr<const tesseract::common::Resource> resource = nullptr,
       const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
       std::shared_ptr<const tesseract::common::VectorVector3d> normals = nullptr,
       std::shared_ptr<const tesseract::common::VectorVector4d> vertex_colors = nullptr,
       std::shared_ptr<MeshMaterial> mesh_material = nullptr,
       std::shared_ptr<const std::vector<std::shared_ptr<MeshTexture>>> mesh_textures = nullptr);
  /**
   * @brief Mesh geometry
   * @param vertices A vector of vertices associated with the mesh
   * @param triangles A vector of face indices where the first index indicates the number of vertices associated
   *                  with the face followed by the vertex index in parameter vertices. For example a triangle
   *                  has three vertices so there should be four inputs where the first should be 3 indicating there are
   *                  three vertices that define this face followed by three indices.
   * @param triangle_count Provide the number of faces. This is faster because it does not need to loop over triangles.
   * @param resource A resource locator for locating resource
   * @param scale Scale the resource mesh. The stored mesh data will already be scaled but if using the resource use
   * this.
   * @param normals A vector of normals for the vertices (optional)
   * @param vertex_colors A vector of colors (RGBA) for the vertices (optional)
   * @param mesh_material A MeshMaterial describing the color and material properties of the mesh (optional)
   * @param mesh_textures A vector of MeshTexture to apply to the mesh (optional)
   */
  Mesh(std::shared_ptr<const tesseract::common::VectorVector3d> vertices,
       std::shared_ptr<const Eigen::VectorXi> triangles,
       int triangle_count,
       std::shared_ptr<const tesseract::common::Resource> resource = nullptr,
       const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
       std::shared_ptr<const tesseract::common::VectorVector3d> normals = nullptr,
       std::shared_ptr<const tesseract::common::VectorVector4d> vertex_colors = nullptr,
       std::shared_ptr<MeshMaterial> mesh_material = nullptr,
       std::shared_ptr<const std::vector<std::shared_ptr<MeshTexture>>> mesh_textures = nullptr);
  Mesh() = default;
  ~Mesh() override = default;

  Geometry::Ptr clone() const override final;

  bool operator==(const Mesh& rhs) const;
  bool operator!=(const Mesh& rhs) const;
};
}  // namespace tesseract::geometry

#endif
