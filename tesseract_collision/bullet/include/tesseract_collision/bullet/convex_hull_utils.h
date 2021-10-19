#ifndef TESSERACT_COLLISION_BULLET_CONVEX_HULL_UTILS_H
#define TESSERACT_COLLISION_BULLET_CONVEX_HULL_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <LinearMath/btConvexHullComputer.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision
{
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

inline tesseract_geometry::ConvexMesh::Ptr makeConvexMesh(const tesseract_geometry::Mesh& mesh)
{
  std::shared_ptr<tesseract_common::VectorVector3d> ch_vertices = std::make_shared<tesseract_common::VectorVector3d>();
  std::shared_ptr<Eigen::VectorXi> ch_faces = std::make_shared<Eigen::VectorXi>();
  int ch_num_faces = createConvexHull(*ch_vertices, *ch_faces, *mesh.getVertices());
  return std::make_shared<tesseract_geometry::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces);
}

}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_BULLET_CONVEX_HULL_UTILS_H
