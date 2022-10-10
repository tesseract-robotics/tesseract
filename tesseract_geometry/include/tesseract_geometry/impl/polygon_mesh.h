/**
 * @file polygon_mesh.h
 * @brief Tesseract Polygon Mesh Geometry
 *
 * @author David Merz, Jr.
 * @date September 9, 2021
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
#ifndef TESSERACT_GEOMETRY_POLYGON_MESH_H
#define TESSERACT_GEOMETRY_POLYGON_MESH_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <Eigen/Geometry>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/types.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_geometry/geometry.h>
#include <tesseract_geometry/impl/mesh_material.h>

namespace tesseract_geometry
{
class PolygonMesh : public Geometry
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<PolygonMesh>;
  using ConstPtr = std::shared_ptr<const PolygonMesh>;

  /**
   * @brief Polygon Mesh geometry
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
  PolygonMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
              std::shared_ptr<const Eigen::VectorXi> faces,
              tesseract_common::Resource::ConstPtr resource = nullptr,
              const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),  // NOLINT
              std::shared_ptr<const tesseract_common::VectorVector3d> normals = nullptr,
              std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors = nullptr,
              MeshMaterial::Ptr mesh_material = nullptr,
              std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures = nullptr,
              GeometryType type = GeometryType::CONVEX_MESH)
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

  /**
   * @brief Polygon Mesh geometry
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
  PolygonMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
              std::shared_ptr<const Eigen::VectorXi> faces,
              int face_count,
              tesseract_common::Resource::ConstPtr resource = nullptr,
              const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),  // NOLINT
              std::shared_ptr<const tesseract_common::VectorVector3d> normals = nullptr,
              std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors = nullptr,
              MeshMaterial::Ptr mesh_material = nullptr,
              std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures = nullptr,
              GeometryType type = GeometryType::CONVEX_MESH)
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

  PolygonMesh() = default;
  ~PolygonMesh() override = default;

  /**
   * @brief Get Polygon mesh vertices
   * @return A vector of vertices
   */
  const std::shared_ptr<const tesseract_common::VectorVector3d>& getVertices() const { return vertices_; }

  /**
   * @brief Get Polygon mesh faces
   * @return A vector of face indices
   */
  const std::shared_ptr<const Eigen::VectorXi>& getFaces() const { return faces_; }

  /**
   * @brief Get vertex count
   * @return Number of vertices
   */
  int getVertexCount() const { return vertex_count_; }

  /**
   * @brief Get face count
   * @return Number of faces
   */
  int getFaceCount() const { return face_count_; }

  /**
   * @brief Get the path to file used to generate the mesh
   *
   * Note: If empty, assume it was manually generated.
   *
   * @return Absolute path to the mesh file
   */
  tesseract_common::Resource::ConstPtr getResource() const { return resource_; }

  /**
   * @brief Get the scale applied to file used to generate the mesh
   * @return The scale x, y, z
   */
  const Eigen::Vector3d& getScale() const { return scale_; }

  /**
   * @brief Get the vertex normal vectors
   *
   * Optional, may be nullptr
   *
   * @return The vertex normal vector
   */
  const std::shared_ptr<const tesseract_common::VectorVector3d>& getNormals() const { return normals_; }

  /**
   * @brief Get the vertex colors
   *
   * Optional, may be nullptr
   *
   * @return Vertex colors
   */
  const std::shared_ptr<const tesseract_common::VectorVector4d>& getVertexColors() const { return vertex_colors_; }

  /**
   * @brief Get material data extracted from the mesh file
   *
   * Mesh files contain material information. The mesh parser will
   * extract the material information and store it in a MeshMaterial structure.
   *
   * @return The MeshMaterial data extracted from mesh file
   */
  MeshMaterial::ConstPtr getMaterial() const { return mesh_material_; }

  /**
   * @brief Get textures extracted from the mesh file
   *
   * Mesh files contain (or reference) image files that form textures on the surface
   * of the mesh. UV coordinates specify how the image is applied to the mesh. The
   * MeshTexture structure contains a resource to the image, and the UV coordinates.
   * Currently only jpg and png image formats are supported.
   *
   * @return Vector of mesh textures
   */
  const std::shared_ptr<const std::vector<MeshTexture::Ptr>>& getTextures() const { return mesh_textures_; }

  Geometry::Ptr clone() const override
  {
    return std::make_shared<PolygonMesh>(vertices_, faces_, face_count_, resource_, scale_);
  }
  bool operator==(const PolygonMesh& rhs) const;
  bool operator!=(const PolygonMesh& rhs) const;

private:
  std::shared_ptr<const tesseract_common::VectorVector3d> vertices_;
  std::shared_ptr<const Eigen::VectorXi> faces_;

  int vertex_count_{ 0 };
  int face_count_{ 0 };
  tesseract_common::Resource::ConstPtr resource_;
  Eigen::Vector3d scale_;
  std::shared_ptr<const tesseract_common::VectorVector3d> normals_;
  std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors_;
  MeshMaterial::Ptr mesh_material_;
  std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_geometry
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_geometry::PolygonMesh, "PolygonMesh")
#endif  // POLYGON_MESH_H
