/**
 * @file common.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cstdio>
#include <cctype>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <boost/algorithm/string.hpp>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_collision/core/common.h>

namespace tesseract_collision
{
ObjectPairKey getObjectPairKey(const std::string& obj1, const std::string& obj2)
{
  return obj1 < obj2 ? std::make_pair(obj1, obj2) : std::make_pair(obj2, obj1);
}

std::vector<ObjectPairKey> getCollisionObjectPairs(const std::vector<std::string>& active_links,
                                                   const std::vector<std::string>& static_links,
                                                   const IsContactAllowedFn& acm)
{
  std::size_t num_pairs = active_links.size() * (active_links.size() - 1) / 2;
  num_pairs += (active_links.size() * static_links.size());

  std::vector<ObjectPairKey> clp;
  clp.reserve(num_pairs);

  // Create active to active pairs
  for (std::size_t i = 0; i < active_links.size() - 1; ++i)
  {
    const std::string& l1 = active_links[i];
    for (std::size_t j = i + 1; j < active_links.size(); ++j)
    {
      const std::string& l2 = active_links[j];
      if (acm == nullptr || (acm != nullptr && !acm(l1, l2)))
        clp.push_back(tesseract_collision::getObjectPairKey(l1, l2));
    }
  }

  // Create active to static pairs
  for (const auto& l1 : active_links)
  {
    for (const auto& l2 : static_links)
    {
      if (acm == nullptr || (acm != nullptr && !acm(l1, l2)))
        clp.push_back(tesseract_collision::getObjectPairKey(l1, l2));
    }
  }

  return clp;
}

bool isLinkActive(const std::vector<std::string>& active, const std::string& name)
{
  return active.empty() || (std::find(active.begin(), active.end(), name) != active.end());
}

bool isContactAllowed(const std::string& name1, const std::string& name2, const IsContactAllowedFn& acm, bool verbose)
{
  // do not distance check geoms part of the same object / link / attached body
  if (name1 == name2)
    return true;

  if (acm != nullptr && acm(name1, name2))
  {
    if (verbose)
    {
      CONSOLE_BRIDGE_logError(
          "Collision between '%s' and '%s' is allowed. No contacts are computed.", name1.c_str(), name2.c_str());
    }
    return true;
  }

  if (verbose)
  {
    CONSOLE_BRIDGE_logError("Actually checking collisions between %s and %s", name1.c_str(), name2.c_str());
  }

  return false;
}

ContactResult* processResult(ContactTestData& cdata,
                             ContactResult& contact,
                             const std::pair<std::string, std::string>& key,
                             bool found)
{
  if (cdata.req.is_valid && !cdata.req.is_valid(contact))
    return nullptr;

  if ((cdata.req.calculate_distance || cdata.req.calculate_penetration) &&
      (contact.distance > cdata.collision_margin_data.getPairCollisionMargin(key.first, key.second)))
    return nullptr;

  if (!found)
  {
    ContactResultVector data;
    if (cdata.req.type == ContactTestType::FIRST)
    {
      data.emplace_back(contact);
      cdata.done = true;
    }
    else
    {
      data.reserve(100);  // TODO: Need better way to initialize this
      data.emplace_back(contact);
    }

    return &(cdata.res->insert(std::make_pair(key, data)).first->second.back());
  }

  assert(cdata.req.type != ContactTestType::FIRST);
  ContactResultVector& dr = (*cdata.res)[key];
  if (cdata.req.type == ContactTestType::ALL)
  {
    dr.emplace_back(contact);
    return &(dr.back());
  }

  if (cdata.req.type == ContactTestType::CLOSEST)
  {
    if (contact.distance < dr[0].distance)
    {
      dr[0] = contact;
      return dr.data();
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

void scaleVertices(tesseract_common::VectorVector3d& vertices,
                   const Eigen::Vector3d& center,
                   const Eigen::Vector3d& scale)
{
  for (auto& v : vertices)
    v = scale.cwiseProduct(v - center) + center;
}

void scaleVertices(tesseract_common::VectorVector3d& vertices, const Eigen::Vector3d& scale)
{
  Eigen::Vector3d center(0, 0, 0);
  for (const auto& v : vertices)
    center = center + v;

  center = (1.0 / static_cast<double>(vertices.size())) * center;

  scaleVertices(vertices, center, scale);
}

bool writeSimplePlyFile(const std::string& path,
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

bool writeSimplePlyFile(const std::string& path,
                        const tesseract_common::VectorVector3d& vertices,
                        const Eigen::VectorXi& faces,
                        int num_faces)
{
  std::vector<Eigen::Vector3i> vertices_color;
  return writeSimplePlyFile(path, vertices, vertices_color, faces, num_faces);
}

int loadSimplePlyFile(const std::string& path,
                      tesseract_common::VectorVector3d& vertices,
                      Eigen::VectorXi& faces,
                      bool triangles_only)
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
    return 0;
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
    return 0;
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
    return 0;
  }

  auto num_faces = static_cast<size_t>(std::stoi(tokens.back()));
  std::getline(myfile, str);
  std::getline(myfile, str);
  if (str != "end_header")
  {
    CONSOLE_BRIDGE_logError("Failed to parse file: %s", path.c_str());
    return 0;
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
      return 0;
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
      return 0;
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
