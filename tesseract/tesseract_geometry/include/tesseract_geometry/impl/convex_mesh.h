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
#include <Eigen/Geometry>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>
#include <tesseract_common/types.h>
#include <tesseract_common/resource.h>

namespace tesseract_geometry
{
class ConvexMesh : public Geometry
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ConvexMesh>;
  using ConstPtr = std::shared_ptr<const ConvexMesh>;

  ConvexMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
             std::shared_ptr<const Eigen::VectorXi> faces,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
    : Geometry(GeometryType::CONVEX_MESH)
    , vertices_(std::move(vertices))
    , faces_(std::move(faces))
    , resource_(std::move(resource))
    , scale_(std::move(scale))
  {
    vertice_count_ = static_cast<int>(vertices_->size());

    face_count_ = 0;
    for (int i = 0; i < faces_->size(); ++i)
    {
      ++face_count_;
      int num_verts = (*faces_)(i);
      i += num_verts;
    }
  }

  ConvexMesh(std::shared_ptr<const tesseract_common::VectorVector3d> vertices,
             std::shared_ptr<const Eigen::VectorXi> faces,
             int face_count,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
    : Geometry(GeometryType::CONVEX_MESH)
    , vertices_(std::move(vertices))
    , faces_(std::move(faces))
    , face_count_(face_count)
    , resource_(std::move(resource))
    , scale_(std::move(scale))
  {
    vertice_count_ = static_cast<int>(vertices_->size());
  }

  ~ConvexMesh() override = default;
  ConvexMesh(const ConvexMesh&) = delete;
  ConvexMesh& operator=(const ConvexMesh&) = delete;
  ConvexMesh(ConvexMesh&&) = delete;
  ConvexMesh& operator=(ConvexMesh&&) = delete;

  const std::shared_ptr<const tesseract_common::VectorVector3d>& getVertices() const { return vertices_; }
  const std::shared_ptr<const Eigen::VectorXi>& getFaces() const { return faces_; }

  int getVerticeCount() const { return vertice_count_; }
  int getFaceCount() const { return face_count_; }

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

  Geometry::Ptr clone() const override
  {
    return std::make_shared<ConvexMesh>(vertices_, faces_, face_count_, resource_, scale_);
  }

private:
  std::shared_ptr<const tesseract_common::VectorVector3d> vertices_;
  std::shared_ptr<const Eigen::VectorXi> faces_;

  int vertice_count_;
  int face_count_;
  tesseract_common::Resource::Ptr resource_;
  Eigen::Vector3d scale_;
};
}  // namespace tesseract_geometry
#endif
