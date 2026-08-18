/**
 * @file ply_io.cpp
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

#include <tesseract/common/ply_io.h>
#include <tesseract/common/utils.h>

#include <console_bridge/console.h>

#include <boost/algorithm/string.hpp>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace tesseract::common
{
namespace
{
/**
 * @brief Read one line and split it on runs of whitespace
 * @details Surrounding whitespace, including a trailing carriage return, is dropped, so no token is
 * ever empty. Returns false at end of file.
 */
bool readTokens(std::ifstream& stream, std::string& line, std::vector<std::string>& tokens)
{
  if (!std::getline(stream, line))
    return false;

  trim(line);
  tokens.clear();
  if (!line.empty())
    boost::split(tokens, line, boost::is_any_of(" \t"), boost::token_compress_on);

  return true;
}

/** @brief Convert a whole token to a double, failing if any character is left unconsumed */
bool toDouble(const std::string& token, double& value)
{
  char* end = nullptr;
  value = std::strtod(token.c_str(), &end);
  return !token.empty() && end == token.c_str() + token.size();
}
}  // namespace

bool writeSimplePlyFile(const std::string& path,
                        const tesseract::common::VectorVector3d& vertices,
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
  // A single color is applied to every vertex; otherwise there must be exactly one per vertex
  if (vectices_color.size() > 1 && vectices_color.size() != vertices.size())
  {
    CONSOLE_BRIDGE_logError("Number of vertex colors (%zu) does not match the number of vertices (%zu): %s",
                            vectices_color.size(),
                            vertices.size(),
                            path.c_str());
    return false;
  }

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
                        const tesseract::common::VectorVector3d& vertices,
                        const Eigen::VectorXi& faces,
                        int num_faces)
{
  std::vector<Eigen::Vector3i> vertices_color;
  return writeSimplePlyFile(path, vertices, vertices_color, faces, num_faces);
}

int loadSimplePlyFile(const std::string& path,
                      tesseract::common::VectorVector3d& vertices,
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
  // The header is parsed by keyword rather than by line offset, so optional property lines
  // (vertex colors, for example) do not shift the element counts out from under the parse.
  std::string str;
  std::vector<std::string> tokens;
  size_t num_vertices{ 0 };
  size_t num_faces{ 0 };
  bool found_vertices{ false };
  bool found_faces{ false };
  bool found_end_header{ false };
  while (readTokens(myfile, str, tokens))
  {
    if (str == "end_header")
    {
      found_end_header = true;
      break;
    }

    if (tokens.size() != 3 || tokens[0] != "element" || !isNumeric(tokens.back()))
      continue;

    if (tokens[1] == "vertex")
    {
      num_vertices = static_cast<size_t>(std::stoi(tokens.back()));
      found_vertices = true;
    }
    else if (tokens[1] == "face")
    {
      num_faces = static_cast<size_t>(std::stoi(tokens.back()));
      found_faces = true;
    }
  }

  if (!found_end_header || !found_vertices || !found_faces)
  {
    CONSOLE_BRIDGE_logError("Failed to parse file: %s", path.c_str());
    return 0;
  }

  vertices.reserve(num_vertices);
  for (size_t i = 0; i < num_vertices; ++i)
  {
    // Columns beyond the first three are optional vertex properties (color, for example)
    Eigen::Vector3d vertex;
    bool parsed = readTokens(myfile, str, tokens) && tokens.size() >= 3;
    for (Eigen::Index k = 0; parsed && k < 3; ++k)
      parsed = toDouble(tokens[static_cast<std::size_t>(k)], vertex(k));

    if (!parsed)
    {
      CONSOLE_BRIDGE_logError("Failed to parse file: %s", path.c_str());
      return 0;
    }

    vertices.push_back(vertex);
  }

  std::vector<int> local_faces;
  local_faces.reserve(num_faces * 4);
  size_t copy_num_faces = num_faces;  // Becuase num_faces can change within for loop
  for (size_t i = 0; i < copy_num_faces; ++i)
  {
    if (!readTokens(myfile, str, tokens) || tokens.size() < 4)
    {
      CONSOLE_BRIDGE_logError("Failed to parse file: %s", path.c_str());
      return 0;
    }

    auto num_verts = std::stoi(tokens[0]);
    assert(num_verts == (tokens.size() - 1));
    assert(num_verts >= 3);
    if (triangles_only && num_verts > 3)
    {
      local_faces.push_back(3);
      local_faces.push_back(std::stoi(tokens[1]));
      local_faces.push_back(std::stoi(tokens[2]));
      local_faces.push_back(std::stoi(tokens[3]));
      for (size_t k = 3; k < tokens.size() - 1; ++k)
      {
        num_faces += 1;
        local_faces.push_back(3);
        local_faces.push_back(std::stoi(tokens[1]));
        local_faces.push_back(std::stoi(tokens[k]));
        local_faces.push_back(std::stoi(tokens[k + 1]));
      }
    }
    else
    {
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

}  // namespace tesseract::common
