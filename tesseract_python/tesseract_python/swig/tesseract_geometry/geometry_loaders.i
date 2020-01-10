/**
 * @file geometry_loaders.i
 * @brief SWIG interface file for tesseract_geometry/geometry_loaders.h
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
#include <tesseract_geometry/geometry_loaders.h>
%}
%include <pybuffer.i>

%pybuffer_binary(const uint8_t* bytes, size_t bytes_len);

namespace tesseract_geometry
{

std::vector<tesseract_geometry::Mesh::Ptr> createMeshFromBytes(const std::string& url, const uint8_t* bytes, size_t bytes_len, 
                                                              Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                              bool triangulate = false,
                                                              bool flatten = false);

std::vector<tesseract_geometry::SDFMesh::Ptr> createSDFMeshFromBytes(const std::string& url, const uint8_t* bytes, size_t bytes_len, 
                                                              Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                              bool triangulate = false,
                                                              bool flatten = false);

std::vector<tesseract_geometry::ConvexMesh::Ptr> createConvexMeshFromBytes(const std::string& url, const uint8_t* bytes, size_t bytes_len, 
                                                              Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                              bool triangulate = false,
                                                              bool flatten = false);

}
