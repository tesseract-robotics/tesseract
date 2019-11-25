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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <Eigen/Eigen>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>
#include <tesseract_geometry/geometry.h>

namespace tesseract_scene_graph
{
class Material
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Material>;
  using ConstPtr = std::shared_ptr<const Material>;

  Material(std::string name) : name_(std::move(name)) { this->clear(); }

  const std::string& getName() const { return name_; }

  std::string texture_filename;
  Eigen::Vector4d color;

  void clear()
  {
    color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
    texture_filename.clear();
  }

private:
  std::string name_;
};

static auto DEFAULT_TESSERACT_MATERIAL = std::make_shared<Material>("default_tesseract_material");

class Inertial
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Inertial>;
  using ConstPtr = std::shared_ptr<const Inertial>;

  Inertial() { this->clear(); }
  Eigen::Isometry3d origin;
  double mass;
  double ixx, ixy, ixz, iyy, iyz, izz;

  void clear()
  {
    origin.setIdentity();
    mass = 0;
    ixx = ixy = ixz = iyy = iyz = izz = 0;
  }
};

class Visual
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Visual>;
  using ConstPtr = std::shared_ptr<const Visual>;

  Visual() { this->clear(); }
  Eigen::Isometry3d origin;
  tesseract_geometry::Geometry::Ptr geometry;

  Material::Ptr material;

  void clear()
  {
    origin.setIdentity();
    material = DEFAULT_TESSERACT_MATERIAL;
    geometry.reset();
    name.clear();
  }

  std::string name;
};

class Collision
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Collision>;
  using ConstPtr = std::shared_ptr<const Collision>;

  Collision() { this->clear(); }
  Eigen::Isometry3d origin;
  tesseract_geometry::Geometry::Ptr geometry;

  void clear()
  {
    origin.setIdentity();
    geometry.reset();
    name.clear();
  }

  std::string name;
};

class Link
{
public:
  using Ptr = std::shared_ptr<Link>;
  using ConstPtr = std::shared_ptr<const Link>;

  Link(std::string name) : name_(std::move(name)) { this->clear(); }

  const std::string& getName() const { return name_; }

  /// inertial element
  Inertial::Ptr inertial;

  /// Visual Elements
  std::vector<Visual::Ptr> visual;

  /// Collision Elements
  std::vector<Collision::Ptr> collision;

  void clear()
  {
    this->inertial.reset();
    this->collision.clear();
    this->visual.clear();
  }

private:
  const std::string name_;
};

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_LINK_H
