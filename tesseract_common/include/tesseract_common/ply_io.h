/**
 * @file ply_io.h
 * @brief Writing ply files
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_COMMON_PLY_IO_H
#define TESSERACT_COMMON_PLY_IO_H

#include <vector>

#include <tesseract_common/eigen_types.h>

namespace tesseract::common
{
/**
 * @brief Write a simple ply file given vertices and faces
 * @param path The file path
 * @param vertices A vector of vertices
 * @param vertices_color The vertices color (0-255,0-255,0-255), if empty uses a default color
 * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
 * @param num_faces The number of faces
 * @return False if failed to write file, otherwise true
 */
bool writeSimplePlyFile(const std::string& path,
                        const tesseract::common::VectorVector3d& vertices,
                        const std::vector<Eigen::Vector3i>& vectices_color,
                        const Eigen::VectorXi& faces,
                        int num_faces);

/**
 * @brief Write a simple ply file given vertices and faces
 * @param path The file path
 * @param vertices A vector of vertices
 * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
 * @param num_faces The number of faces
 * @return False if failed to write file, otherwise true
 */
bool writeSimplePlyFile(const std::string& path,
                        const tesseract::common::VectorVector3d& vertices,
                        const Eigen::VectorXi& faces,
                        int num_faces);

/**
 * @brief Loads a simple ply file given a path
 * @param path The file path
 * @param vertices A vector of vertices
 * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
 * @param triangles_only Convert to only include triangles
 * @return Number of faces, If returned 0 it failed to load.
 */
int loadSimplePlyFile(const std::string& path,
                      tesseract::common::VectorVector3d& vertices,
                      Eigen::VectorXi& faces,
                      bool triangles_only = false);
}  // namespace tesseract::common

#endif  // TESSERACT_COMMON_PLY_IO_H
