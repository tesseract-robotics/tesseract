/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef TESSERACT_SCENE_GRAPH_GEOMETRY_H
#define TESSERACT_SCENE_GRAPH_GEOMETRY_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <Eigen/Eigen>

namespace tesseract
{

class Geometry
{
public:
  enum {SPHERE, BOX, CYLINDER, MESH, CONVEX, OCTOMAP, SDF} type;

  virtual ~Geometry(void)
  {
  }
};
typedef std::shared_ptr<Geometry> GeometryPtr;
typedef std::shared_ptr<const Geometry> GeometryConstPtr;

class Sphere : public Geometry
{
public:
  Sphere() { this->clear(); type = SPHERE; }
  double radius;

  void clear()
  {
    radius = 0;
  }
};
typedef std::shared_ptr<Sphere> SpherePtr;
typedef std::shared_ptr<const Sphere> SphereConstPtr;

class Box : public Geometry
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Box() { this->clear(); type = BOX; }

  Eigen::Vector3d dim;

  void clear()
  {
    this->dim.setZero();
  }
};
typedef std::shared_ptr<Box> BoxPtr;
typedef std::shared_ptr<const Box> BoxConstPtr;

class Cylinder : public Geometry
{
public:
  Cylinder() { this->clear(); type = CYLINDER; }

  double length;
  double radius;

  void clear()
  {
    length = 0;
    radius = 0;
  }
};
typedef std::shared_ptr<Cylinder> CylinderPtr;
typedef std::shared_ptr<const Cylinder> CylinderConstPtr;

class Mesh : public Geometry
{
public:
  Mesh() { this->clear(); type = MESH; }

  std::vector<Eigen::Vector3d> verticies;
  std::vector<int> faces;

  void clear()
  {
    verticies.clear();
    faces.clear();
  }
};
typedef std::shared_ptr<Mesh> MeshPtr;
typedef std::shared_ptr<const Mesh> MeshConstPtr;

class ConvexMesh : public Geometry
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMesh() { this->clear(); type = CONVEX; }

  std::vector<Eigen::Vector3d> verticies;
  std::vector<int> faces;

  void clear()
  {
    verticies.clear();
    faces.clear();
  }
};
typedef std::shared_ptr<ConvexMesh> ConvexMeshPtr;
typedef std::shared_ptr<const ConvexMesh> ConvexMeshConstPtr;
}
#endif // TESSERACT_SCENE_GRAPH_GEOMETRY_H
