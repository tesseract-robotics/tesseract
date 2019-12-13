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
#include <LinearMath/btConvexHullComputer.h>
#include <cstdio>
#include <cctype>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <boost/algorithm/string.hpp>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/common.h>

namespace tesseract_collision
{
using ObjectPairKey = std::pair<std::string, std::string>;

/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @return The collision pair key
 */
inline ObjectPairKey getObjectPairKey(const std::string& obj1, const std::string& obj2)
{
  return obj1 < obj2 ? std::make_pair(obj1, obj2) : std::make_pair(obj2, obj1);
}

/**
 * @brief This will check if a link is active provided a list. If the list is empty the link is considered active.
 * @param active List of active link names
 * @param name The name of link to check if it is active.
 */
inline bool isLinkActive(const std::vector<std::string>& active, const std::string& name)
{
  return active.empty() || (std::find(active.begin(), active.end(), name) != active.end());
}

/**
 * @brief Determine if contact is allowed between two objects.
 * @param name1 The name of the first object
 * @param name2 The name of the second object
 * @param acm The contact allowed function
 * @param verbose If true print debug informaton
 * @return True if contact is allowed between the two object, otherwise false.
 */
inline bool isContactAllowed(const std::string& name1,
                             const std::string& name2,
                             const IsContactAllowedFn& acm,
                             bool verbose = false)
{
  // do not distance check geoms part of the same object / link / attached body
  if (name1 == name2)
    return true;

  if (acm != nullptr && acm(name1, name2))
  {
    if (verbose)
    {
      CONSOLE_BRIDGE_logDebug(
          "Collision between '%s' and '%s' is allowed. No contacts are computed.", name1.c_str(), name2.c_str());
    }
    return true;
  }

  if (verbose)
  {
    CONSOLE_BRIDGE_logDebug("Actually checking collisions between %s and %s", name1.c_str(), name2.c_str());
  }

  return false;
}

inline ContactResult* processResult(ContactTestData& cdata,
                                    ContactResult& contact,
                                    const std::pair<std::string, std::string>& key,
                                    bool found)
{
  if (!found)
  {
    ContactResultVector data;
    if (cdata.type == ContactTestType::FIRST)
    {
      data.emplace_back(contact);
      cdata.done = true;
    }
    else
    {
      data.reserve(100);  // TODO: Need better way to initialize this
      data.emplace_back(contact);
    }

    return &(cdata.res.insert(std::make_pair(key, data)).first->second.back());
  }

  assert(cdata.type != ContactTestType::FIRST);
  ContactResultVector& dr = cdata.res[key];
  if (cdata.type == ContactTestType::ALL)
  {
    dr.emplace_back(contact);
    return &(dr.back());
  }

  if (cdata.type == ContactTestType::CLOSEST)
  {
    if (contact.distance < dr[0].distance)
    {
      dr[0] = contact;
      return &(dr[0]);
    }
  }
  //    else if (cdata.cdata.condition == DistanceRequestType::LIMITED)
  //    {
  //      assert(dr.size() < cdata.req->max_contacts_per_body);
  //      dr.emplace_back(contact);
  //      return &(dr.back());
  //    }

  return nullptr;
}

/**
 * @brief Create a convex hull from vertices using Bullet Convex Hull Computer
 * @param (Output) vertices A vector of vertices
 * @param (Output) faces The first values indicates the number of vertices that define the face followed by the vertice
 * index
 * @param (input) input A vector of point to create a convex hull from
 * @param (input) shrink If positive, the convex hull is shrunken by that amount (each face is moved by "shrink" length
 *                units towards the center along its normal).
 * @param (input) shrinkClamp If positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where
 *                "innerRadius" is the minimum distance of a face to the center of the convex hull.
 * @return The number of faces. If less than zero an error occured when trying to create the convex hull
 */
inline int createConvexHull(tesseract_common::VectorVector3d& vertices,
                            Eigen::VectorXi& faces,
                            const tesseract_common::VectorVector3d& input,
                            double shrink = -1,
                            double shrinkClamp = -1)
{
  vertices.clear();

  btConvexHullComputer conv;
  std::vector<double> points;
  points.reserve(input.size() * 3);
  for (const auto& v : input)
  {
    points.push_back(v[0]);
    points.push_back(v[1]);
    points.push_back(v[2]);
  }

  btScalar val = conv.compute(points.data(),
                              3 * sizeof(double),
                              static_cast<int>(input.size()),
                              static_cast<btScalar>(shrink),
                              static_cast<btScalar>(shrinkClamp));
  if (val < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to create convex hull");
    return -1;
  }

  int num_verts = conv.vertices.size();
  vertices.reserve(static_cast<size_t>(num_verts));
  for (int i = 0; i < num_verts; i++)
  {
    btVector3& v = conv.vertices[i];
    vertices.push_back(
        Eigen::Vector3d(static_cast<double>(v.getX()), static_cast<double>(v.getY()), static_cast<double>(v.getZ())));
  }

  auto num_faces = static_cast<size_t>(conv.faces.size());
  std::vector<int> local_faces;
  local_faces.reserve(3ul * num_faces);
  for (int i = 0; i < conv.faces.size(); i++)
  {
    std::vector<int> face;
    face.reserve(3);

    const btConvexHullComputer::Edge* sourceEdge = &(conv.edges[conv.faces[i]]);
    int a = sourceEdge->getSourceVertex();
    face.push_back(a);

    int b = sourceEdge->getTargetVertex();
    face.push_back(b);

    const btConvexHullComputer::Edge* edge = sourceEdge->getNextEdgeOfFace();
    int c = edge->getTargetVertex();
    face.push_back(c);

    edge = edge->getNextEdgeOfFace();
    c = edge->getTargetVertex();
    while (c != a)
    {
      face.push_back(c);

      edge = edge->getNextEdgeOfFace();
      c = edge->getTargetVertex();
    }
    local_faces.push_back(static_cast<int>(face.size()));
    local_faces.insert(local_faces.end(), face.begin(), face.end());
  }

  faces.resize(static_cast<long>(local_faces.size()));
  for (size_t i = 0; i < local_faces.size(); ++i)
    faces[static_cast<long>(i)] = local_faces[i];

  return conv.faces.size();
}

inline tesseract_geometry::ConvexMesh::Ptr makeConvexMesh(const tesseract_geometry::Mesh& mesh)
{
  std::shared_ptr<tesseract_common::VectorVector3d> ch_vertices = std::make_shared<tesseract_common::VectorVector3d>();
  std::shared_ptr<Eigen::VectorXi> ch_faces = std::make_shared<Eigen::VectorXi>();
  int ch_num_faces = tesseract_collision::createConvexHull(*ch_vertices, *ch_faces, *mesh.getVertices());
  return std::make_shared<tesseract_geometry::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces);
}

/**
 * @brief Write a simple ply file given vertices and faces
 * @param path The file path
 * @param vertices A vector of vertices
 * @param vertices_color The vertices color (0-255,0-255,0-255), if empty uses a default color
 * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
 * @param num_faces The number of faces
 * @return False if failed to write file, otherwise true
 */
inline bool writeSimplePlyFile(const std::string& path,
                               const tesseract_common::VectorVector3d& vertices,
                               const std::vector<Eigen::Vector3i>& vectices_color,
                               const Eigen::VectorXi& faces,
                               int num_faces)
{
  //  ply
  //  format ascii 1.0           { ascii/binary, format version number }
  //  comment made by Greg Turk  { comments keyword specified, like all lines }
  //  comment this file is a cube
  //  element vertex 8           { define "vertex" element, 8 of them in file }
  //  property float x           { vertex contains float "x" coordinate }
  //  property float y           { y coordinate is also a vertex property }
  //  property float z           { z coordinate, too }
  //  property uchar red         { start of vertex color }
  //  property uchar green
  //  property uchar blue
  //  element face 6             { there are 6 "face" elements in the file }
  //  property list uchar int vertex_index { "vertex_indices" is a list of ints }
  //  end_header                 { delimits the end of the header }
  //  0 0 0                      { start of vertex list }
  //  0 0 1
  //  0 1 1
  //  0 1 0
  //  1 0 0
  //  1 0 1
  //  1 1 1
  //  1 1 0
  //  4 0 1 2 3                  { start of face list }
  //  4 7 6 5 4
  //  4 0 4 5 1
  //  4 1 5 6 2
  //  4 2 6 7 3
  //  4 3 7 4 0
  std::ofstream myfile;
  myfile.open(path);
  if (myfile.fail())
  {
    CONSOLE_BRIDGE_logError("Failed to open file: %s", path.c_str());
    return false;
  }

  myfile << "ply\n";
  myfile << "format ascii 1.0\n";
  myfile << "comment made by tesseract\n";
  myfile << "element vertex " << vertices.size() << "\n";
  myfile << "property float x\n";
  myfile << "property float y\n";
  myfile << "property float z\n";
  if (!vectices_color.empty())
  {
    myfile << "property uchar red\n";
    myfile << "property uchar green\n";
    myfile << "property uchar blue\n";
  }
  myfile << "element face " << num_faces << "\n";
  myfile << "property list uchar int vertex_indices\n";
  myfile << "end_header\n";

  // Add vertices
  if (vectices_color.empty())
  {
    for (const auto& v : vertices)
    {
      myfile << std::fixed << std::setprecision(std::numeric_limits<float>::digits10 + 1) << v[0] << " " << v[1] << " "
             << v[2] << "\n";
    }
  }
  else if (vectices_color.size() == 1)
  {
    const Eigen::Vector3i& default_color = vectices_color[0];
    for (const auto& v : vertices)
    {
      myfile << std::fixed << std::setprecision(std::numeric_limits<float>::digits10 + 1) << v[0] << " " << v[1] << " "
             << v[2] << " " << default_color[0] << " " << default_color[1] << " " << default_color[2] << "\n";
    }
  }
  else
  {
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
      const Eigen::Vector3d& v = vertices[i];
      const Eigen::Vector3i& v_color = vectices_color[i];
      myfile << std::fixed << std::setprecision(std::numeric_limits<float>::digits10 + 1) << v[0] << " " << v[1] << " "
             << v[2] << " " << v_color[0] << " " << v_color[1] << " " << v_color[2] << "\n";
    }
  }

  // Add faces
  long idx = 0;
  for (long i = 0; i < num_faces; ++i)
  {
    long num_vert = faces[idx];
    for (long j = 0; j < num_vert; ++j)
    {
      myfile << faces[idx] << " ";
      ++idx;
    }
    myfile << faces[idx] << "\n";
    ++idx;
  }

  myfile.close();
  return true;
}

/**
 * @brief Write a simple ply file given vertices and faces
 * @param path The file path
 * @param vertices A vector of vertices
 * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
 * @param num_faces The number of faces
 * @return False if failed to write file, otherwise true
 */
inline bool writeSimplePlyFile(const std::string& path,
                               const tesseract_common::VectorVector3d& vertices,
                               const Eigen::VectorXi& faces,
                               int num_faces)
{
  std::vector<Eigen::Vector3i> vertices_color;
  return writeSimplePlyFile(path, vertices, vertices_color, faces, num_faces);
}

/**
 * @brief Loads a simple ply file given a path
 * @param path The file path
 * @param vertices A vector of vertices
 * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
 * @param triangles_only Convert to only include triangles
 * @return Number of faces, If returned 0 it failed to load.
 */
inline int loadSimplePlyFile(const std::string& path,
                             tesseract_common::VectorVector3d& vertices,
                             Eigen::VectorXi& faces,
                             bool triangles_only = false)
{
  //  ply
  //  format ascii 1.0           { ascii/binary, format version number }
  //  comment made by Greg Turk  { comments keyword specified, like all lines }
  //  comment this file is a cube
  //  element vertex 8           { define "vertex" element, 8 of them in file }
  //  property float x           { vertex contains float "x" coordinate }
  //  property float y           { y coordinate is also a vertex property }
  //  property float z           { z coordinate, too }
  //  element face 6             { there are 6 "face" elements in the file }
  //  property list uchar int vertex_index { "vertex_indices" is a list of ints }
  //  end_header                 { delimits the end of the header }
  //  0 0 0                      { start of vertex list }
  //  0 0 1
  //  0 1 1
  //  0 1 0
  //  1 0 0
  //  1 0 1
  //  1 1 1
  //  1 1 0
  //  4 0 1 2 3                  { start of face list }
  //  4 7 6 5 4
  //  4 0 4 5 1
  //  4 1 5 6 2
  //  4 2 6 7 3
  //  4 3 7 4 0

  vertices.clear();

  std::ifstream myfile;
  myfile.open(path);
  if (myfile.fail())
  {
    CONSOLE_BRIDGE_logError("Failed to open file: %s", path.c_str());
    return false;
  }
  std::string str;
  std::getline(myfile, str);
  std::getline(myfile, str);
  std::getline(myfile, str);
  std::getline(myfile, str);
  std::vector<std::string> tokens;
  boost::split(tokens, str, boost::is_any_of(" "));
  if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens.back()))
  {
    CONSOLE_BRIDGE_logError("Failed to parse file: %s", path.c_str());
    return false;
  }
  auto num_vertices = static_cast<size_t>(std::stoi(tokens.back()));

  std::getline(myfile, str);
  std::getline(myfile, str);
  std::getline(myfile, str);
  std::getline(myfile, str);

  tokens.clear();
  boost::split(tokens, str, boost::is_any_of(" "));
  if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens.back()))
  {
    CONSOLE_BRIDGE_logError("Failed to parse file: %s", path.c_str());
    return false;
  }

  auto num_faces = static_cast<size_t>(std::stoi(tokens.back()));
  std::getline(myfile, str);
  std::getline(myfile, str);
  if (str != "end_header")
  {
    CONSOLE_BRIDGE_logError("Failed to parse file: %s", path.c_str());
    return false;
  }

  vertices.reserve(num_vertices);
  for (size_t i = 0; i < num_vertices; ++i)
  {
    std::getline(myfile, str);
    tokens.clear();
    boost::split(tokens, str, boost::is_any_of(" "));
    if (tokens.size() != 3)
    {
      CONSOLE_BRIDGE_logError("Failed to parse file: %s", path.c_str());
      return false;
    }

    vertices.push_back(Eigen::Vector3d(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2])));
  }

  std::vector<int> local_faces;
  local_faces.reserve(num_faces * 3);
  size_t copy_num_faces = num_faces;  // Becuase num_faces can change within for loop
  for (size_t i = 0; i < copy_num_faces; ++i)
  {
    std::getline(myfile, str);
    tokens.clear();
    boost::split(tokens, str, boost::is_any_of(" "));
    if (tokens.size() < 3)
    {
      CONSOLE_BRIDGE_logError("Failed to parse file: %s", path.c_str());
      return false;
    }

    auto num_verts = static_cast<int>(tokens.size());
    assert(num_verts >= 3);
    if (triangles_only && num_verts > 3)
    {
      local_faces.push_back(3);
      local_faces.push_back(std::stoi(tokens[0]));
      local_faces.push_back(std::stoi(tokens[1]));
      local_faces.push_back(std::stoi(tokens[2]));
      for (size_t i = 3; i < tokens.size(); ++i)
      {
        num_faces += 1;
        local_faces.push_back(3);
        local_faces.push_back(std::stoi(tokens[0]));
        local_faces.push_back(std::stoi(tokens[i - 1]));
        local_faces.push_back(std::stoi(tokens[i]));
      }
    }
    else
    {
      local_faces.push_back(static_cast<int>(tokens.size()));
      for (const auto& t : tokens)
        local_faces.push_back(std::stoi(t));
    }
  }

  faces.resize(static_cast<long>(local_faces.size()));
  for (size_t i = 0; i < local_faces.size(); ++i)
    faces[static_cast<long>(i)] = local_faces[i];

  myfile.close();
  return static_cast<int>(num_faces);
}

}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_COMMON_H
