/**
 * @file convex_hull_utils.h
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
#include <console_bridge/console.h>
#include <LinearMath/btConvexHullComputer.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/convex_hull_utils.h>

namespace tesseract_collision
{
int createConvexHull(tesseract_common::VectorVector3d& vertices,
                     Eigen::VectorXi& faces,
                     const tesseract_common::VectorVector3d& input,
                     double shrink,
                     double shrinkClamp)
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
  local_faces.reserve(3UL * num_faces);
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

tesseract_geometry::ConvexMesh::Ptr makeConvexMesh(const tesseract_geometry::Mesh& mesh)
{
  std::shared_ptr<tesseract_common::VectorVector3d> ch_vertices = std::make_shared<tesseract_common::VectorVector3d>();
  std::shared_ptr<Eigen::VectorXi> ch_faces = std::make_shared<Eigen::VectorXi>();
  int ch_num_faces = createConvexHull(*ch_vertices, *ch_faces, *mesh.getVertices());
  return std::make_shared<tesseract_geometry::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces);
}

}  // namespace tesseract_collision
