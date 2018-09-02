/**
 * @file contact_checker_common.h
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
#ifndef TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H
#define TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H

#include <tesseract_core/basic_types.h>
#include <ros/console.h>

#include <bullet/LinearMath/btConvexHullComputer.h>
#include <cstdio>
#include <Eigen/Geometry>
#include <fstream>

namespace tesseract
{
typedef std::pair<std::string, std::string> ObjectPairKey;

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
 * @brief Determine if contact is allowed between two objects.
 * @param name1 The name of the first object
 * @param name2 The name of the second object
 * @param acm The contact allowed function
 * @param verbose If true print debug informaton
 * @return True if contact is allowed between the two object, otherwise false.
 */
inline bool
isContactAllowed(const std::string& name1, const std::string& name2, const IsContactAllowedFn acm, bool verbose = false)
{
  // do not distance check geoms part of the same object / link / attached body
  if (name1 == name2)
    return true;

  if (acm != nullptr && acm(name1, name2))
  {
    if (verbose)
    {
      ROS_DEBUG("Collision between '%s' and '%s' is allowed. No contacts are computed.", name1.c_str(), name2.c_str());
    }
    return true;
  }

  if (verbose)
  {
    ROS_DEBUG("Actually checking collisions between %s and %s", name1.c_str(), name2.c_str());
  }

  return false;
}

inline ContactResult* processResult(ContactDistanceData& cdata,
                                    ContactResult& contact,
                                    const std::pair<std::string, std::string>& key,
                                    bool found)
{
  if (!found)
  {
    ContactResultVector data;
    if (cdata.req->type == ContactRequestType::FIRST)
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
  else
  {
    assert(cdata.req->type != ContactRequestType::FIRST);
    ContactResultVector& dr = cdata.res->at(key);
    if (cdata.req->type == ContactRequestType::ALL)
    {
      dr.emplace_back(contact);
      return &(dr.back());
    }
    else if (cdata.req->type == ContactRequestType::CLOSEST)
    {
      if (contact.distance < dr[0].distance)
      {
        dr[0] = contact;
        return &(dr[0]);
      }
    }
    //    else if (cdata.req->type == DistanceRequestType::LIMITED)
    //    {
    //      assert(dr.size() < cdata.req->max_contacts_per_body);
    //      dr.emplace_back(contact);
    //      return &(dr.back());
    //    }
  }

  return nullptr;
}

/**
 * @brief Create a convex hull from vertices using Bullet Convex Hull Computer
 * @param (Output) vertices A vector of vertices
 * @param (Output) faces The first values indicates the number of vertices that define the face followed by the vertice index
 * @param (input) input A vector of point to create a convex hull from
 * @param (input) shrink If positive, the convex hull is shrunken by that amount (each face is moved by "shrink" length
 *                units towards the center along its normal).
 * @param (input) shrinkClamp If positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where
 *                "innerRadius" is the minimum distance of a face to the center of the convex hull.
 * @return The number of faces. If less than zero an error occured when trying to create the convex hull
 */
inline int createConvexHull(VectorVector3d& vertices, std::vector<int>& faces, const VectorVector3d& input, double shrink = -1, double shrinkClamp = -1)
{
  vertices.clear();
  faces.clear();

  btConvexHullComputer conv;
  btAlignedObjectArray<btVector3> points;
  points.reserve(input.size());
  for (const auto& v : input)
  {
    points.push_back(btVector3(v[0], v[1], v[2]));
  }

  btScalar val = conv.compute(&points[0].getX(), sizeof(btVector3), points.size(), shrink, shrinkClamp);
  if (val < 0)
  {
    ROS_ERROR("Failed to create convex hull");
    return -1;
  }

  int num_verts = conv.vertices.size();
  vertices.reserve(num_verts);
  for (int i = 0; i < num_verts; i++)
  {
    btVector3& v = conv.vertices[i];
    vertices.push_back(Eigen::Vector3d(v.getX(), v.getY(), v.getZ()));
  }

  int num_faces = conv.faces.size();
  faces.reserve(3 * num_faces);
  for (int i = 0; i < num_faces; i++)
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
    faces.push_back(face.size());
    faces.insert(faces.end(), face.begin(), face.end());
  }

  return num_faces;
}

/**
 * @brief Write a simple ply file given vertices and faces
 * @param path The file path
 * @param vertices A vector of vertices
 * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
 * @param num_faces The number of faces
 */
inline void writeSimplePlyFile(const std::string& path, const VectorVector3d vertices, const std::vector<int> &faces, int num_faces)
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
  std::ofstream myfile;
  myfile.open (path);
  myfile << "ply\n";
  myfile << "format ascii 1.0\n";
  myfile << "comment made by tesseract\n";
  myfile << "element vertex " << vertices.size() << "\n";
  myfile << "property float x\n";
  myfile << "property float y\n";
  myfile << "property float z\n";
  myfile << "element face " << num_faces << "\n";
  myfile << "property list uchar uint vertex_indices\n";
  myfile << "end_header\n";

  // Add vertices
  for (const auto& v : vertices)
  {
    myfile << std::fixed << v[0] << " " << v[1] << " " << v[2] << "\n";
  }

  // Add faces
  int idx = 0;
  for (int i = 0; i < num_faces; ++i)
  {
    int num_vert = faces[idx];
    for (int j = 0; j < num_vert; ++j)
    {
      myfile << faces[idx] << " ";
      ++idx;
    }
    myfile << faces[idx] << "\n";
    ++idx;
  }

  myfile.close();
}

}

#endif  // TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H

///**
// * @brief Create a convex hull from points using QHull
// * @param (Output) vertices A vector of vertices
// * @param (Output) faces The first values indicates the number of vertices that define the face followed by the vertice index
// * @param (input) input A vector of point to create a convex hull from
// * @param (input) flags A string representing QHull flags. Recommends sending at least Tv.
// * @return The number of faces. If less than zero an error occured when trying to create the convex hull
// */
//inline int createConvexHullQHull(VectorVector3d& vertices, std::vector<int>& faces, const VectorVector3d& input, const std::string& flags = "Tv")
//{
//  vertices.clear();
//  faces.clear();

//  /* compute convex hull */
//  coordT* points = (coordT*)calloc(input.size() * 3, sizeof(coordT));
//  for (unsigned int i = 0; i < input.size(); ++i)
//  {
//    points[3 * i + 0] = (coordT)input[i][0];
//    points[3 * i + 1] = (coordT)input[i][1];
//    points[3 * i + 2] = (coordT)input[i][2];
//  }

//  static FILE* null = fopen("/dev/null", "w");

//  std::string s = "qhull " + flags;
//  char cmd[s.length() + 1];
//  strcpy(cmd, s.c_str());
//  int exitcode = qh_new_qhull(3, input.size(), points, true, cmd, null, null);

//  if (exitcode != 0)
//  {
//    ROS_ERROR("Failed to create convex hull");
//    qh_freeqhull(!qh_ALL);
//    int curlong, totlong;
//    qh_memfreeshort(&curlong, &totlong);
//    return -1;
//  }

//  int num_facets = qh num_facets;

//  int num_vertices = qh num_vertices;
//  vertices.reserve(num_vertices);

//  // necessary for FORALLvertices
//  std::map<unsigned int, unsigned int> qhull_vertex_table;
//  vertexT* vertex, **vertexp;
//  FORALLvertices
//  {
//    Eigen::Vector3d vert(vertex->point[0], vertex->point[1], vertex->point[2]);
//    qhull_vertex_table[vertex->id] = vertices.size();
//    vertices.push_back(vert);
//  }

//  faces.reserve(num_facets * 3);

//  // neccessary for qhull macro
//  facetT* facet;
//  setT *v_set;
//  FORALLfacets
//  {
//    v_set = qh_facet3vertex(facet);

//    int num_face_verts = qh_setsize(v_set);
//    std::vector<int> face;
//    face.reserve(num_face_verts);
//    faces.push_back(num_face_verts);

//    FOREACHvertex_(v_set)
//    {
//      face.push_back(qhull_vertex_table[vertex->id]);
//    }

//    // The order creates normals pointing into the shape so need to reverse the order.
//    faces.insert(faces.end(), face.rbegin(), face.rend());
//  }

//  qh_freeqhull(!qh_ALL);
//  int curlong, totlong;
//  qh_memfreeshort(&curlong, &totlong);

//  return num_facets;
//}
