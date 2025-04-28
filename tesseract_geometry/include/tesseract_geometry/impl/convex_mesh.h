/**
 * @file convex_mesh.h
 * @brief Tesseract Convex Mesh Geometry
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
#ifndef TESSERACT_GEOMETRY_CONVEX_MESH_H
#define TESSERACT_GEOMETRY_CONVEX_MESH_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/export.hpp>
#include <Eigen/Geometry>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/polygon_mesh.h>

namespace boost::serialization
{
class access;
}

namespace tesseract_geometry
{
class ConvexMesh : public PolygonMesh
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<ConvexMesh>;
  using ConstPtr = std::shared_ptr<const ConvexMesh>;

  enum CreationMethod : std::uint8_t
  {
    DEFAULT,
    MESH,
    CONVERTED
  };

  /**
   * @brief Convex Mesh geometry
   * @param vertices A vector of vertices associated with the mesh
   * @param faces A vector of face indices, where the first number indicates the number of vertices
   *              associated with the face, followed by the vertex index in parameter vertices. For
   *              example, a triangle has three vertices, so there should be four inputs, where the
   *              first should be 3, indicating there are three vertices that define this face,
   *              followed by three indices.
   * @param resource A resource locator for locating resource
   * @param scale Scale the mesh
   * @param normals A vector of normals for the vertices (optional)
   * @param vertex_colors A vector of colors (RGBA) for the vertices (optional)
   * @param mesh_material A MeshMaterial describing the color and material properties of the mesh (optional)
   * @param mesh_textures A vector of MeshTexture to apply to the mesh (optional)
   */
  ConvexMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
             std::shared_ptr<const Eigen::VectorXi> faces,
             std::shared_ptr<const tesseract_common::Resource> resource = nullptr,
             const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
             std::shared_ptr<const tesseract_common::VectorVector3d> normals = nullptr,
             std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors = nullptr,
             std::shared_ptr<MeshMaterial> mesh_material = nullptr,
             std::shared_ptr<const std::vector<std::shared_ptr<MeshTexture>>> mesh_textures = nullptr);

  /**
   * @brief Convex Mesh geometry
   * @param vertices A vector of vertices associated with the mesh
   * @param faces A vector of face indices, where the first number indicates the number of vertices
   *              associated with the face, followed by the vertex index in parameter vertices. For
   *              example, a triangle has three vertices, so there should be four inputs, where the
   *              first should be 3, indicating there are three vertices that define this face,
   *              followed by three indices.
   * @param face_count Provide the number of faces. This is faster because it does not need to loop
   *                   over the faces.
   * @param resource A resource locator for locating resource
   * @param scale Scale the mesh
   * @param normals A vector of normals for the vertices (optional)
   * @param vertex_colors A vector of colors (RGBA) for the vertices (optional)
   * @param mesh_material Describes the color and material properties of the mesh (optional)
   * @param mesh_textures A vector of MeshTexture to apply to the mesh (optional)
   */
  ConvexMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
             std::shared_ptr<const Eigen::VectorXi> faces,
             int face_count,
             std::shared_ptr<const tesseract_common::Resource> resource = nullptr,
             const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
             std::shared_ptr<const tesseract_common::VectorVector3d> normals = nullptr,
             std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors = nullptr,
             std::shared_ptr<MeshMaterial> mesh_material = nullptr,
             std::shared_ptr<const std::vector<std::shared_ptr<MeshTexture>>> mesh_textures = nullptr);

  ConvexMesh() = default;
  ~ConvexMesh() override = default;

  /**
   * @brief Get how the convex hull was created
   * @note This used when writing back out to urdf
   * @return The CreationMethod
   */
  CreationMethod getCreationMethod() const;

  /**
   * @brief Set the method used to create the convex mesh
   * @note This used when writing back out to urdf
   * @param value The CreationMethod
   */
  void setCreationMethod(CreationMethod method);

  Geometry::Ptr clone() const override;

  bool operator==(const ConvexMesh& rhs) const;
  bool operator!=(const ConvexMesh& rhs) const;

private:
  CreationMethod creation_method_{ DEFAULT };

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_geometry

BOOST_CLASS_EXPORT_KEY(tesseract_geometry::ConvexMesh)
#endif
