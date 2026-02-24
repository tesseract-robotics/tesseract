/**
 * @file compund_mesh.cpp
 * @brief Tesseract Compound Mesh Geometry
 *
 * @author Levi Armstrong
 * @date March 16, 2022
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

#include <tesseract/common/utils.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/geometry/impl/compound_mesh.h>
#include <tesseract/geometry/impl/convex_mesh.h>
#include <tesseract/geometry/impl/mesh.h>
#include <tesseract/geometry/impl/sdf_mesh.h>

template <typename T>
std::vector<std::shared_ptr<tesseract::geometry::PolygonMesh>> convert(std::vector<std::shared_ptr<T>>& meshes)
{
  std::vector<std::shared_ptr<tesseract::geometry::PolygonMesh>> polygon_meshes;
  polygon_meshes.reserve(meshes.size());
  for (const auto& mesh : meshes)
    polygon_meshes.push_back(mesh);

  return polygon_meshes;
}

namespace tesseract::geometry
{
CompoundMesh::CompoundMesh(std::vector<std::shared_ptr<PolygonMesh>> meshes)
  : Geometry(GeometryType::COMPOUND_MESH), meshes_(std::move(meshes))
{
  if (meshes_.size() <= 1)
    throw std::runtime_error("Meshes must contain more than one mesh");

#ifndef NDEBUG
  for (const auto& mesh : meshes_)
  {
    assert(tesseract::common::pointersEqual<const tesseract::common::Resource>(meshes_[0]->getResource(),
                                                                               mesh->getResource()));
    assert(tesseract::common::almostEqualRelativeAndAbs(meshes_[0]->getScale(), mesh->getScale()));
  }
#endif
}

CompoundMesh::CompoundMesh(std::vector<std::shared_ptr<ConvexMesh>> meshes) : CompoundMesh(convert(meshes)) {}

CompoundMesh::CompoundMesh(std::vector<std::shared_ptr<Mesh>> meshes) : CompoundMesh(convert(meshes)) {}

CompoundMesh::CompoundMesh(std::vector<std::shared_ptr<SDFMesh>> meshes) : CompoundMesh(convert(meshes)) {}

const std::vector<std::shared_ptr<PolygonMesh>>& CompoundMesh::getMeshes() const { return meshes_; }

std::shared_ptr<const tesseract::common::Resource> CompoundMesh::getResource() const
{
  return meshes_.front()->getResource();
}

const Eigen::Vector3d& CompoundMesh::getScale() const { return meshes_.front()->getScale(); }

Geometry::Ptr CompoundMesh::clone() const
{
  std::vector<std::shared_ptr<PolygonMesh>> meshes;
  meshes.reserve(meshes_.size());
  for (const auto& mesh : meshes_)
    meshes.emplace_back(std::dynamic_pointer_cast<PolygonMesh>(mesh->clone()));

  return std::make_shared<CompoundMesh>(meshes);
}

bool CompoundMesh::operator==(const CompoundMesh& rhs) const
{
  if (meshes_.size() != rhs.meshes_.size())
    return false;

  bool equal = true;
  equal &= Geometry::operator==(rhs);
  for (std::size_t i = 0; i < meshes_.size(); ++i)
    equal &= (*meshes_.at(i) == *rhs.meshes_.at(i));

  return equal;
}
bool CompoundMesh::operator!=(const CompoundMesh& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::geometry
