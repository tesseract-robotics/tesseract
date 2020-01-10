/**
 * @file sdf_mesh.i
 * @brief SWIG interface file for tesseract_environment/impl/sdf_mesh.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_geometry/impl/sdf_mesh.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::SDFMesh)

%template(tesseract_geometry_SDFMeshVector) std::vector<tesseract_geometry::SDFMesh::Ptr>;

namespace tesseract_geometry
{
  %nodefaultctor SDFMesh;
class SDFMesh : public Geometry
{
public:
  
  using Ptr = std::shared_ptr<SDFMesh>;
  using ConstPtr = std::shared_ptr<const SDFMesh>;
/*
%extend {

  SDFMesh(const tesseract_common::VectorVector3d& vertices,
             const Eigen::VectorXi& faces,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
  {
      std::shared_ptr<const tesseract_common::VectorVector3d> vertices1 = std::make_shared<const tesseract_common::VectorVector3d>(vertices);
      std::shared_ptr<const Eigen::VectorXi> faces1 = std::make_shared<const Eigen::VectorXi>(faces);
      return new tesseract_geometry::SDFMesh(vertices1, faces1, resource, scale);
  }

  SDFMesh(const tesseract_common::VectorVector3d& vertices,
             const Eigen::VectorXi& faces,
             int face_count,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
  {
      std::shared_ptr<const tesseract_common::VectorVector3d> vertices1 = std::make_shared<const tesseract_common::VectorVector3d>(vertices);
      std::shared_ptr<const Eigen::VectorXi> faces1 = std::make_shared<const Eigen::VectorXi>(faces);
      return new tesseract_geometry::SDFMesh(vertices1, faces1, face_count, resource, scale);
  }
}*/
  ~SDFMesh() override;

%extend {

  tesseract_common::VectorVector3d getVertices()
  {
    return *$self->getVertices();
  }

  Eigen::VectorXi getTriangles()
  {
    return *$self->getTriangles();
  }
}

  int getVerticeCount() const;
  int getTriangleCount() const;

  const tesseract_common::Resource::Ptr getResource();

  const Eigen::Vector3d& getScale() const { return scale_; }
};
}  // namespace tesseract_geometry