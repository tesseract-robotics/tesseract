/**
 * @file mesh.h
 * @brief Tesseract Mesh Geometry
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
#ifndef TESSERACT_GEOMETRY_MESH_H
#define TESSERACT_GEOMETRY_MESH_H

#include <tesseract_common/macros.h>
#include <tesseract_common/resource.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>
#include <tesseract_common/types.h>
#include <tesseract_geometry/impl/mesh_material.h>

#ifdef SWIG
%shared_ptr(tesseract_geometry::Mesh)
%template(MeshVector) std::vector<std::shared_ptr<tesseract_geometry::Mesh> >;
#endif  // SWIG

namespace tesseract_geometry
{
class Mesh : public Geometry
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Mesh>;
  using ConstPtr = std::shared_ptr<const Mesh>;

  /**
   * @brief Mesh geometry
   * @param vertices A vector of vertices associated with the mesh
   * @param triangles A vector of face indices where the first indice indicates the number of vertices associated
   *                  with the face followed by the vertice index in parameter vertices. For example a triangle
   *                  has three vertices so there should be four inputs where the first should be 3 indicating there are
   *                  three vertices that define this face followed by three indices.
   * @param resource A resource locator for locating resource
   * @param scale Scale the mesh
   * @param normals A vector of normals for the vertices (optional)
   * @param vertex_colors A vector of colors (RGBA) for the vertices (optional)
   * @param mesh_material A MeshMaterial describing the color and material properties of the mesh (optional)
   * @param mesh_textures A vector of MeshTexture to apply to the mesh (optional)
   */
  Mesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
       std::shared_ptr<const Eigen::VectorXi> triangles,
       tesseract_common::Resource::Ptr resource = nullptr,
       Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
       std::shared_ptr<const tesseract_common::VectorVector3d> normals = nullptr,
       std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors = nullptr,
       MeshMaterial::Ptr mesh_material = nullptr,
       std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures = nullptr)
    : Geometry(GeometryType::MESH)
    , vertices_(std::move(vertices))
    , triangles_(std::move(triangles))
    , resource_(std::move(resource))
    , scale_(std::move(scale))
    , normals_(std::move(normals))
    , vertex_colors_(std::move(vertex_colors))
    , mesh_material_(std::move(mesh_material))
    , mesh_textures_(std::move(mesh_textures))
  {
    vertice_count_ = static_cast<int>(vertices_->size());

    triangle_count_ = 0;
    for (int i = 0; i < triangles_->size(); ++i)
    {
      ++triangle_count_;
      int num_verts = (*triangles_)(i);
      i += num_verts;
      assert(num_verts == 3);
    }
  }

  /**
   * @brief Mesh geometry
   * @param vertices A vector of vertices associated with the mesh
   * @param triangles A vector of face indices where the first indice indicates the number of vertices associated
   *                  with the face followed by the vertice index in parameter vertices. For example a triangle
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
  Mesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
       std::shared_ptr<const Eigen::VectorXi> triangles,
       int triangle_count,
       tesseract_common::Resource::Ptr resource = nullptr,
       Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
       std::shared_ptr<const tesseract_common::VectorVector3d> normals = nullptr,
       std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors = nullptr,
       MeshMaterial::Ptr mesh_material = nullptr,
       std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures = nullptr)
    : Geometry(GeometryType::MESH)
    , vertices_(std::move(vertices))
    , triangles_(std::move(triangles))
    , triangle_count_(triangle_count)
    , resource_(std::move(resource))
    , scale_(std::move(scale))
    , normals_(std::move(normals))
    , vertex_colors_(std::move(vertex_colors))
    , mesh_material_(std::move(mesh_material))
    , mesh_textures_(std::move(mesh_textures))
  {
    vertice_count_ = static_cast<int>(vertices_->size());
    assert((triangle_count_ * 4) == triangles_->size());
  }

  ~Mesh() override = default;
  Mesh(const Mesh&) = delete;
  Mesh& operator=(const Mesh&) = delete;
  Mesh(Mesh&&) = delete;
  Mesh& operator=(Mesh&&) = delete;

  /**
   * @brief Get mesh vertices
   * @return A vector of vertices
   */
  const std::shared_ptr<const tesseract_common::VectorVector3d>& getVertices() const { return vertices_; }

  /**
   * @brief Get mesh Triangles
   * @return A vector of triangle indices
   */
  const std::shared_ptr<const Eigen::VectorXi>& getTriangles() const { return triangles_; }

  /**
   * @brief Get vertice count
   * @return Number of vertices
   */
  int getVerticeCount() const { return vertice_count_; }

  /**
   * @brief Get triangle count
   * @return Number of triangles
   */
  int getTriangleCount() const { return triangle_count_; }

  /**
   * @brief Get the path to file used to generate the mesh
   *
   * Note: If empty, assume it was manually generated.
   *
   * @return Absolute path to the mesh file
   */
  const tesseract_common::Resource::Ptr getResource() const { return resource_; }

  /**
   * @brief Get the scale applied to file used to generate the mesh
   * @return The scale x, y, z
   */
  const Eigen::Vector3d& getScale() const { return scale_; }

  /**
   * @brief Get the vertice normal vectors
   *
   * Optional, may be nullptr
   *
   * @return The vertice normal vector
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
    return std::make_shared<Mesh>(vertices_,
                                  triangles_,
                                  triangle_count_,
                                  resource_,
                                  scale_,
                                  normals_,
                                  vertex_colors_,
                                  mesh_material_,
                                  mesh_textures_);
  }

private:
  std::shared_ptr<const tesseract_common::VectorVector3d> vertices_;
  std::shared_ptr<const Eigen::VectorXi> triangles_;
  int vertice_count_;
  int triangle_count_;
  tesseract_common::Resource::Ptr resource_;
  Eigen::Vector3d scale_;
  std::shared_ptr<const tesseract_common::VectorVector3d> normals_;
  std::shared_ptr<const tesseract_common::VectorVector4d> vertex_colors_;
  MeshMaterial::Ptr mesh_material_;
  std::shared_ptr<const std::vector<MeshTexture::Ptr>> mesh_textures_;
};
}  // namespace tesseract_geometry
#endif
