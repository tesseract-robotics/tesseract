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

#include <tesseract_geometry/macros.h>
TESSERACT_GEOMETRY_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
TESSERACT_GEOMETRY_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>
#include <tesseract_geometry/types.h>

namespace tesseract_geometry
{
  class ConvexMesh;
  typedef std::shared_ptr<ConvexMesh> ConvexMeshPtr;
  typedef std::shared_ptr<const ConvexMesh> ConvexMeshConstPtr;

  class ConvexMesh : public Geometry
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConvexMesh(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& faces) : Geometry(GeometryType::CONVEX_MESH), vertices_(vertices), faces_(faces)
    {
      vertice_count_ = static_cast<int>(vertices->size());

      face_count_ = 0;
      for (auto it = faces_->begin(); it != faces_->end(); ++it)
      {
        ++face_count_;
        int num_verts = *it;
        it = it + num_verts;
      }
    }

    ConvexMesh(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& faces, int face_count) : Geometry(GeometryType::CONVEX_MESH), vertices_(vertices), faces_(faces), face_count_(face_count)
    {
      vertice_count_ = static_cast<int>(vertices->size());
    }

    ~ConvexMesh() override = default;

    const std::shared_ptr<const VectorVector3d>& getVertices() const { return vertices_; }
    const std::shared_ptr<const std::vector<int>>& getFaces() const { return faces_; }

    int getVerticeCount() const { return vertice_count_; }
    int getFaceCount() const { return face_count_; }

    GeometryPtr clone() const override { return ConvexMeshPtr(new ConvexMesh(vertices_, faces_, face_count_)); }

  private:
    std::shared_ptr<const VectorVector3d> vertices_;
    std::shared_ptr<const std::vector<int>> faces_;

    int vertice_count_;
    int face_count_;
  };
}
#endif
