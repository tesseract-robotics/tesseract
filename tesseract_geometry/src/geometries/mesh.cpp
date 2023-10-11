/**
 * @file Mesh.cpp
 * @brief Tesseract Mesh Geometry
 *
 * @author Levi Armstrong
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_geometry/impl/mesh.h>

namespace tesseract_geometry
{
bool Mesh::operator==(const Mesh& rhs) const
{
  bool equal = true;
  equal &= PolygonMesh::operator==(rhs);
  return equal;
}
bool Mesh::operator!=(const Mesh& rhs) const { return !operator==(rhs); }

template <class Archive>
void Mesh::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PolygonMesh);
}
}  // namespace tesseract_geometry

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_geometry::Mesh)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_geometry::Mesh)
