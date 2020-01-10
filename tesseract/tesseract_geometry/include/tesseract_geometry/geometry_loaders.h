/**
 * @file geometry_loaders.i
 * @brief Tesseract geometry loaders for Python
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

#include <tesseract_geometry/mesh_parser.h>

#pragma once

namespace tesseract_geometry
{
static std::vector<tesseract_geometry::Mesh::Ptr> createMeshFromBytes(const std::string& url,
                                                                      const uint8_t* bytes,
                                                                      size_t bytes_len,
                                                                      Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                                      bool triangulate = false,
                                                                      bool flatten = false)
{
  std::shared_ptr<tesseract_common::Resource> resource =
      std::make_shared<tesseract_common::BytesResource>(url, bytes, bytes_len);
  return tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(resource, scale, triangulate, flatten);
}

static std::vector<tesseract_geometry::SDFMesh::Ptr>
createSDFMeshFromBytes(const std::string& url,
                       const uint8_t* bytes,
                       size_t bytes_len,
                       Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                       bool triangulate = false,
                       bool flatten = false)
{
  std::shared_ptr<tesseract_common::Resource> resource =
      std::make_shared<tesseract_common::BytesResource>(url, bytes, bytes_len);
  return tesseract_geometry::createMeshFromResource<tesseract_geometry::SDFMesh>(resource, scale, triangulate, flatten);
}

static std::vector<tesseract_geometry::ConvexMesh::Ptr>
createConvexMeshFromBytes(const std::string& url,
                          const uint8_t* bytes,
                          size_t bytes_len,
                          Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                          bool triangulate = false,
                          bool flatten = false)
{
  std::shared_ptr<tesseract_common::Resource> resource =
      std::make_shared<tesseract_common::BytesResource>(url, bytes, bytes_len);
  return tesseract_geometry::createMeshFromResource<tesseract_geometry::ConvexMesh>(
      resource, scale, triangulate, flatten);
}

}  // namespace tesseract_geometry
