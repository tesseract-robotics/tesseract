/**
 * @file common.h
 * @brief This is a collection of common methods
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_COLLISION_COMMON_H
#define TESSERACT_COLLISION_COMMON_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <map>
#include <vector>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>

namespace tesseract_collision
{
using ObjectPairKey = std::pair<std::string, std::string>;

/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @return The collision pair key
 */
ObjectPairKey getObjectPairKey(const std::string& obj1, const std::string& obj2);

/**
 * @brief Get a vector of possible collision object pairs
 * @todo Should this also filter out links without geometry?
 * @param active_links The active link names
 * @param static_links The static link names
 * @param acm The is contact allowed function
 * @return A vector of collision object pairs
 */
std::vector<ObjectPairKey> getCollisionObjectPairs(const std::vector<std::string>& active_links,
                                                   const std::vector<std::string>& static_links,
                                                   const IsContactAllowedFn& acm = nullptr);

/**
 * @brief This will check if a link is active provided a list. If the list is empty the link is considered active.
 * @param active List of active link names
 * @param name The name of link to check if it is active.
 */
bool isLinkActive(const std::vector<std::string>& active, const std::string& name);

/**
 * @brief Determine if contact is allowed between two objects.
 * @param name1 The name of the first object
 * @param name2 The name of the second object
 * @param acm The contact allowed function
 * @param verbose If true print debug information
 * @return True if contact is allowed between the two object, otherwise false.
 */
bool isContactAllowed(const std::string& name1,
                      const std::string& name2,
                      const IsContactAllowedFn& acm,
                      bool verbose = false);

/**
 * @brief processResult Processes the ContactResult based on the information in the ContactTestData
 * @param cdata Information used to process the results
 * @param contact Contacts from the collision checkers that will be processed
 * @param key Link pair used as a key to look up pair specific settings
 * @param found Specifies whether or not a collision has already been found
 * @return Pointer to the ContactResult.
 */
ContactResult* processResult(ContactTestData& cdata,
                             ContactResult& contact,
                             const std::pair<std::string, std::string>& key,
                             bool found);

/**
 * @brief Apply scaling to the geometry coordinates.
 * @details Given a scaling factor s, and center c, a given vertice v is transformed according to s (v - c) + c.
 * @param vertices The vertices to scale
 * @param center The point at which to scale the data about
 * @param scale The scale factor to apply to the vertices.
 */
void scaleVertices(tesseract_common::VectorVector3d& vertices,
                   const Eigen::Vector3d& center,
                   const Eigen::Vector3d& scale);

/**
 * @brief Apply scaling to the geometry coordinates.
 * @details Given a scaling factor s, and center c, a given vertice v is transformed according to s (v - c) + c.
 * @param vertices The vertices to scale
 * @param scale The scale factor to apply to the vertices.
 */
void scaleVertices(tesseract_common::VectorVector3d& vertices, const Eigen::Vector3d& scale);

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
                        const tesseract_common::VectorVector3d& vertices,
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
                        const tesseract_common::VectorVector3d& vertices,
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
                      tesseract_common::VectorVector3d& vertices,
                      Eigen::VectorXi& faces,
                      bool triangles_only = false);

}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_COMMON_H
