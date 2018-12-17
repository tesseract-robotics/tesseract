/**
 * @file create_convex_hull.cpp
 * @brief This takes an input file and generates a convex hull ply file
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
#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <geometric_shapes/mesh_operations.h>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_collision/contact_checker_common.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_create_convex_hull_node");
  ros::NodeHandle pnh("~");
  std::string input;
  std::string output;
  double shrink;
  double clamp;

  std::string help = "\nExample:\n"
                     "  create_convex_hull _input:=/home/mesh.stl _output:=/home/mesh.ply\n\n"
                     "Parameters:\n"
                     "  input  (required): File path to mesh used to create a convex hull.\n"
                     "  output (required): File path to save the generated convex hull as a ply.\n"
                     "  shrink (optional): If positive, the convex hull is shrunken by that amount (each face is moved "
                     "by 'shrink' length units towards the center along its normal).\n"
                     "  clamp  (optional): If positive, 'shrink' is clamped to not exceed 'clamp * innerRadius', where "
                     "'innerRadius' is the minimum distance of a face to the center of the convex hull.\n";

  if (!pnh.hasParam("input") || !pnh.hasParam("output"))
  {
    ROS_ERROR(help.c_str());
    return -1;
  }

  pnh.getParam("input", input);
  pnh.getParam("output", output);
  pnh.param<double>("shrink", shrink, -1.0);
  pnh.param<double>("clamp", clamp, -1.0);

  std::ifstream file(input, std::ios::binary | std::ios::ate);
  std::streamsize size = file.tellg();
  if (size < 0)
  {
    ROS_ERROR("Failed to locate input file!");
    return -1;
  }

  file.seekg(0, std::ios::beg);
  std::vector<char> buffer(static_cast<size_t>(size));
  if (!file.read(buffer.data(), size))
  {
    ROS_ERROR("Failed to read input file!");
    return -1;
  }

  std::shared_ptr<shapes::Mesh> mesh;
  mesh.reset(
      shapes::createMeshFromBinary(buffer.data(), static_cast<size_t>(size), Eigen::Vector3d(1.0, 1.0, 1.0), input));
  if (mesh == nullptr)
  {
    ROS_ERROR("Failed to create mesh from binary data!");
    return -1;
  }

  tesseract::VectorVector3d mesh_vertices;
  mesh_vertices.reserve(mesh->vertex_count);

  for (unsigned int i = 0; i < mesh->vertex_count; ++i)
    mesh_vertices.push_back(
        Eigen::Vector3d(mesh->vertices[3 * i + 0], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]));

  tesseract::VectorVector3d convex_hull_vertices;
  std::vector<int> convex_hull_faces;
  int num_faces = tesseract::createConvexHull(convex_hull_vertices, convex_hull_faces, mesh_vertices);

  if (num_faces < 0)
  {
    ROS_ERROR("Failed to create convex hull!");
    return -1;
  }

  if (!tesseract::writeSimplePlyFile(output, convex_hull_vertices, convex_hull_faces, num_faces))
    return -1;

  return 0;
}
