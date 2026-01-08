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
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_geometry/fwd.h>

namespace tesseract_scene_graph
{
class Material;
template <class Archive>
void serialize(Archive& ar, Material& obj);

class Link;
template <class Archive>
void serialize(Archive& ar, Link& obj);

class Material
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Material>;
  using ConstPtr = std::shared_ptr<const Material>;

  Material() = default;
  explicit Material(std::string name);
  ~Material() = default;

  Material(const Material&) = default;
  Material& operator=(const Material&) = default;
  Material(Material&&) = default;
  Material& operator=(Material&&) = default;

  const std::string& getName() const;

  static std::shared_ptr<Material> getDefaultMaterial();

  std::string texture_filename;
  Eigen::Vector4d color;

  void clear();

  bool operator==(const Material& rhs) const;
  bool operator!=(const Material& rhs) const;

private:
  std::string name_;

  template <class Archive>
  friend void ::tesseract_scene_graph::serialize(Archive& ar, Material& obj);
};

class Inertial
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Inertial>;
  using ConstPtr = std::shared_ptr<const Inertial>;

  Inertial() = default;
  ~Inertial() = default;

  Inertial(const Inertial&) = default;
  Inertial& operator=(const Inertial&) = default;
  Inertial(Inertial&&) = default;
  Inertial& operator=(Inertial&&) = default;

  Eigen::Isometry3d origin{ Eigen::Isometry3d::Identity() };
  double mass{ 0 };
  double ixx{ 0 };
  double ixy{ 0 };
  double ixz{ 0 };
  double iyy{ 0 };
  double iyz{ 0 };
  double izz{ 0 };

  void clear();

  bool operator==(const Inertial& rhs) const;
  bool operator!=(const Inertial& rhs) const;
};

class Visual
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Visual>;
  using ConstPtr = std::shared_ptr<const Visual>;

  Visual();
  ~Visual() = default;
  Visual(const Visual&) = default;
  Visual& operator=(const Visual&) = default;
  Visual(Visual&&) = default;
  Visual& operator=(Visual&&) = default;

  Eigen::Isometry3d origin;
  std::shared_ptr<const tesseract_geometry::Geometry> geometry;

  Material::Ptr material;
  std::string name;

  void clear();

  bool operator==(const Visual& rhs) const;
  bool operator!=(const Visual& rhs) const;
};

class Collision
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Collision>;
  using ConstPtr = std::shared_ptr<const Collision>;

  Collision();
  ~Collision() = default;
  Collision(const Collision&) = default;
  Collision& operator=(const Collision&) = default;
  Collision(Collision&&) = default;
  Collision& operator=(Collision&&) = default;

  Eigen::Isometry3d origin;
  std::shared_ptr<const tesseract_geometry::Geometry> geometry;
  std::string name;

  void clear();

  bool operator==(const Collision& rhs) const;
  bool operator!=(const Collision& rhs) const;
};

class Link
{
public:
  using Ptr = std::shared_ptr<Link>;
  using ConstPtr = std::shared_ptr<const Link>;

  Link(const std::string& name);
  Link() = default;
  ~Link() = default;
  // Links are non-copyable as their name must be unique
  Link(const Link& other) = delete;
  Link& operator=(const Link& other) = delete;

  Link(Link&& other) = default;
  Link& operator=(Link&& other) = default;

  const std::string& getName() const;

  const std::size_t& getHash() const;

  /// inertial element
  Inertial::Ptr inertial;

  /// Visual Elements
  std::vector<Visual::Ptr> visual;

  /// Collision Elements
  std::vector<Collision::Ptr> collision;

  /// Visibility Enable
  bool visible{ true };

  /// Collision Enable
  bool collision_enabled{ true };

  void clear();

  bool operator==(const Link& rhs) const;
  bool operator!=(const Link& rhs) const;

  /**
   * @brief Clone the link keeping the name.
   * @return Cloned link
   */
  Link clone() const;

  /** Perform a copy of link, changing its name **/
  Link clone(const std::string& name) const;

private:
  std::string name_;

  /** @brief The hash of the link name */
  std::size_t hash_{};

  template <class Archive>
  friend void ::tesseract_scene_graph::serialize(Archive& ar, Link& obj);
};

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_LINK_H
