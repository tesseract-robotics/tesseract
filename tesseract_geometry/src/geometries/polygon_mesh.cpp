/**
 * @file polygon_mesh.cpp
 * @brief Tesseract PolygonMesh Geometry
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/eigen_serialization.h>
#include <tesseract_geometry/impl/polygon_mesh.h>

namespace tesseract_geometry
{
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

template <class Archive>
void PolygonMesh::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Geometry);
  ar& BOOST_SERIALIZATION_NVP(vertices_);
  ar& BOOST_SERIALIZATION_NVP(faces_);
  ar& BOOST_SERIALIZATION_NVP(vertex_count_);
  ar& BOOST_SERIALIZATION_NVP(face_count_);
  ar& BOOST_SERIALIZATION_NVP(scale_);
  ar& BOOST_SERIALIZATION_NVP(normals_);
  ar& BOOST_SERIALIZATION_NVP(vertex_colors_);
  /// @todo Serialize mesh materials and textures
  //    ar& BOOST_SERIALIZATION_NVP(mesh_material_);
  //    ar& BOOST_SERIALIZATION_NVP(mesh_textures_);
}
}  // namespace tesseract_geometry

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_geometry::PolygonMesh)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_geometry::PolygonMesh)
