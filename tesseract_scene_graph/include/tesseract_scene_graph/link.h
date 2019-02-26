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

#ifndef TESSERACT_SCENE_GRAPH_LINK_H
#define TESSERACT_SCENE_GRAPH_LINK_H

#include <tesseract_scene_graph/macros.h>
TESSERACT_SCENE_GRAPH_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <Eigen/Eigen>
TESSERACT_SCENE_GRAPH_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>
#include <tesseract_geometry/geometry.h>

namespace tesseract_scene_graph
{

class Link;
typedef std::shared_ptr<Link> LinkPtr;
typedef std::shared_ptr<const Link> LinkConstPtr;


class Material
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Material() { this->clear(); }
  std::string name;
  std::string texture_filename;
  Eigen::Vector4d color;

  void clear()
  {
    color.setZero();
    texture_filename.clear();
    name.clear();
  }
};
typedef std::shared_ptr<Material> MaterialPtr;
typedef std::shared_ptr<const Material> MaterialConstPtr;

class Inertial
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Inertial() { this->clear(); }
  Eigen::Isometry3d origin;
  double mass;
  double ixx,ixy,ixz,iyy,iyz,izz;

  void clear()
  {
    origin.setIdentity();
    mass = 0;
    ixx = ixy = ixz = iyy = iyz = izz = 0;
  }
};
typedef std::shared_ptr<Inertial> InertialPtr;
typedef std::shared_ptr<const Inertial> InertialConstPtr;

class Visual
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Visual() { this->clear(); }
  Eigen::Isometry3d origin;
  tesseract_geometry::GeometryPtr geometry;

  std::string material_name;
  MaterialPtr material;

  void clear()
  {
    origin.setIdentity();
    material_name.clear();
    material.reset();
    geometry.reset();
    name.clear();
  }

  std::string name;
};
typedef std::shared_ptr<Visual> VisualPtr;
typedef std::shared_ptr<const Visual> VisualConstPtr;

class Collision
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Collision() { this->clear(); }
  Eigen::Isometry3d origin;
  tesseract_geometry::GeometryPtr geometry;

  void clear()
  {
    origin.setIdentity();
    geometry.reset();
    name.clear();
  }

  std::string name;

};
typedef std::shared_ptr<Collision> CollisionPtr;
typedef std::shared_ptr<const Collision> CollisionConstPtr;

class Link
{
public:
  Link(std::string name) : name_(name) { this->clear(); }

  const std::string& getName() const { return name_; }

  /// inertial element
  InertialPtr inertial;

  /// Visual Elements
  std::vector<VisualPtr> visual;

  /// Collision Elements
  std::vector<CollisionPtr> collision;

  void clear()
  {
    this->inertial.reset();
    this->collision.clear();
    this->visual.clear();
  }

private:
  const std::string name_;

};

}

#endif // TESSERACT_SCENE_GRAPH_LINK_H
