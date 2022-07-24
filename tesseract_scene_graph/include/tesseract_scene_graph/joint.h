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

#ifndef TESSERACT_SCENE_GRAPH_JOINT_H
#define TESSERACT_SCENE_GRAPH_JOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Eigen>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_scene_graph
{
class Link;

class JointDynamics
{
public:
  using Ptr = std::shared_ptr<JointDynamics>;
  using ConstPtr = std::shared_ptr<const JointDynamics>;

  JointDynamics() = default;
  JointDynamics(double damping, double friction) : damping(damping), friction(friction) {}
  double damping{ 0 };
  double friction{ 0 };

  void clear()
  {
    damping = 0;
    friction = 0;
  }
  bool operator==(const JointDynamics& rhs) const;
  bool operator!=(const JointDynamics& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class JointLimits
{
public:
  using Ptr = std::shared_ptr<JointLimits>;
  using ConstPtr = std::shared_ptr<const JointLimits>;

  JointLimits() = default;
  JointLimits(double l, double u, double e, double v, double a)
    : lower(l), upper(u), effort(e), velocity(v), acceleration(a)
  {
  }

  double lower{ 0 };
  double upper{ 0 };
  double effort{ 0 };
  double velocity{ 0 };
  double acceleration{ 0 };

  void clear()
  {
    lower = 0;
    upper = 0;
    effort = 0;
    velocity = 0;
    acceleration = 0;
  }

  friend std::ostream& operator<<(std::ostream& os, const JointLimits& limits)
  {
    os << "lower=" << limits.lower << " upper=" << limits.upper << " effort=" << limits.effort
       << " velocity=" << limits.velocity << " acceleration=" << limits.acceleration;
    return os;
  };
  bool operator==(const JointLimits& rhs) const;
  bool operator!=(const JointLimits& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/// \brief Parameters for Joint Safety Controllers
class JointSafety
{
public:
  using Ptr = std::shared_ptr<JointSafety>;
  using ConstPtr = std::shared_ptr<const JointSafety>;

  JointSafety() = default;
  JointSafety(double soft_upper_limit, double soft_lower_limit, double k_position, double k_velocity)
    : soft_upper_limit(soft_upper_limit)
    , soft_lower_limit(soft_lower_limit)
    , k_position(k_position)
    , k_velocity(k_velocity)
  {
  }

  ///
  /// IMPORTANT:  The safety controller support is very much PR2 specific, not intended for generic usage.
  ///
  /// Basic safety controller operation is as follows
  ///
  /// current safety controllers will take effect on joints outside the position range below:
  ///
  /// position range: [JointSafety::soft_lower_limit  + JointLimits::velocity / JointSafety::k_position,
  ///                  JointSafety::soft_uppper_limit - JointLimits::velocity / JointSafety::k_position]
  ///
  /// if (joint_position is outside of the position range above)
  ///     velocity_limit_min = -JointLimits::velocity + JointSafety::k_position * (joint_position -
  ///     JointSafety::soft_lower_limit) velocity_limit_max =  JointLimits::velocity + JointSafety::k_position *
  ///     (joint_position - JointSafety::soft_upper_limit)
  /// else
  ///     velocity_limit_min = -JointLimits::velocity
  ///     velocity_limit_max =  JointLimits::velocity
  ///
  /// velocity range: [velocity_limit_min + JointLimits::effort / JointSafety::k_velocity,
  ///                  velocity_limit_max - JointLimits::effort / JointSafety::k_velocity]
  ///
  /// if (joint_velocity is outside of the velocity range above)
  ///     effort_limit_min = -JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_min)
  ///     effort_limit_max =  JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_max)
  /// else
  ///     effort_limit_min = -JointLimits::effort
  ///     effort_limit_max =  JointLimits::effort
  ///
  /// Final effort command sent to the joint is saturated by [effort_limit_min,effort_limit_max]
  ///
  /// Please see wiki for more details: http://www.ros.org/wiki/pr2_controller_manager/safety_limits
  ///
  double soft_upper_limit{ 0 };
  double soft_lower_limit{ 0 };
  double k_position{ 0 };
  double k_velocity{ 0 };

  void clear()
  {
    soft_upper_limit = 0;
    soft_lower_limit = 0;
    k_position = 0;
    k_velocity = 0;
  }
  bool operator==(const JointSafety& rhs) const;
  bool operator!=(const JointSafety& rhs) const;

  friend std::ostream& operator<<(std::ostream& os, const JointSafety& safety)
  {
    os << "soft_upper_limit=" << safety.soft_upper_limit << " soft_lower_limit=" << safety.soft_lower_limit
       << " k_position=" << safety.k_position << " k_velocity=" << safety.k_velocity;
    return os;
  };

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class JointCalibration
{
public:
  using Ptr = std::shared_ptr<JointCalibration>;
  using ConstPtr = std::shared_ptr<const JointCalibration>;

  JointCalibration() = default;
  JointCalibration(double reference_position, double rising, double falling)
    : reference_position(reference_position), rising(rising), falling(falling)
  {
  }
  double reference_position{ 0 };
  double rising{ 0 };
  double falling{ 0 };

  void clear()
  {
    reference_position = 0;
    rising = 0;
    falling = 0;
  }
  bool operator==(const JointCalibration& rhs) const;
  bool operator!=(const JointCalibration& rhs) const;

  friend std::ostream& operator<<(std::ostream& os, const JointCalibration& calibration)
  {
    os << "reference_position=" << calibration.reference_position << " rising=" << calibration.rising
       << " falling=" << calibration.falling;
    return os;
  };

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class JointMimic
{
public:
  using Ptr = std::shared_ptr<JointMimic>;
  using ConstPtr = std::shared_ptr<const JointMimic>;

  JointMimic() = default;
  JointMimic(double offset, double multiplier, std::string joint_name)
    : offset(offset), multiplier(multiplier), joint_name(std::move(joint_name))
  {
  }
  double offset{ 0 };
  double multiplier{ 1.0 };
  std::string joint_name;

  void clear()
  {
    offset = 0.0;
    multiplier = 1.0;
    joint_name.clear();
  }
  bool operator==(const JointMimic& rhs) const;
  bool operator!=(const JointMimic& rhs) const;

  friend std::ostream& operator<<(std::ostream& os, const JointMimic& mimic)
  {
    os << "joint_name=" << mimic.joint_name << " offset=" << mimic.offset << " multiplier=" << mimic.multiplier;
    return os;
  };

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

enum class JointType
{
  UNKNOWN,
  REVOLUTE,
  CONTINUOUS,
  PRISMATIC,
  FLOATING,
  PLANAR,
  FIXED
};

class Joint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Joint>;
  using ConstPtr = std::shared_ptr<const Joint>;

  Joint(std::string name) : name_(std::move(name)) { this->clear(); }
  Joint() = default;
  ~Joint() = default;
  // Joints are non-copyable as their name must be unique
  Joint(const Joint& other) = delete;
  Joint& operator=(const Joint& other) = delete;

  Joint(Joint&& other) = default;
  Joint& operator=(Joint&& other) = default;

  const std::string& getName() const { return name_; }

  /// The type of joint
  JointType type{ JointType::UNKNOWN };

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            UNKNOWN     unknown type
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            FLOATING    N/A
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  Eigen::Vector3d axis;

  /// child Link element
  ///   child link frame is the same as the Joint frame
  std::string child_link_name;

  /// parent Link element
  ///   origin specifies the transform from Parent Link to Joint Frame
  std::string parent_link_name;

  /// transform from Parent Link frame to Joint frame
  Eigen::Isometry3d parent_to_joint_origin_transform{ Eigen::Isometry3d::Identity() };

  /// Joint Dynamics
  JointDynamics::Ptr dynamics;

  /// Joint Limits
  JointLimits::Ptr limits;

  /// Unsupported Hidden Feature
  JointSafety::Ptr safety;

  /// Unsupported Hidden Feature
  JointCalibration::Ptr calibration;

  /// Option to Mimic another Joint
  JointMimic::Ptr mimic;

  void clear()
  {
    this->axis = Eigen::Vector3d(1, 0, 0);
    this->child_link_name.clear();
    this->parent_link_name.clear();
    this->parent_to_joint_origin_transform.setIdentity();
    this->dynamics.reset();
    this->limits.reset();
    this->safety.reset();
    this->calibration.reset();
    this->mimic.reset();
    this->type = JointType::UNKNOWN;
  }

  /**
   * @brief Clone the joint keeping the name
   * @return Cloned joint
   */
  Joint clone() const { return clone(name_); }

  /* Create a clone of current joint, with a new name. Child link name and parent link name are unchanged.
   * All underlying properties, such as dynamics, limits... are copied as well.*/
  Joint clone(const std::string& name) const
  {
    Joint ret(name);
    ret.axis = this->axis;
    ret.child_link_name = this->child_link_name;
    ret.parent_link_name = this->parent_link_name;
    ret.parent_to_joint_origin_transform = this->parent_to_joint_origin_transform;
    ret.type = this->type;
    if (this->dynamics)
    {
      ret.dynamics = std::make_shared<JointDynamics>(*(this->dynamics));
    }
    if (this->limits)
    {
      ret.limits = std::make_shared<JointLimits>(*(this->limits));
    }
    if (this->safety)
    {
      ret.safety = std::make_shared<JointSafety>(*(this->safety));
    }
    if (this->calibration)
    {
      ret.calibration = std::make_shared<JointCalibration>(*(this->calibration));
    }
    if (this->mimic)
    {
      ret.mimic = std::make_shared<JointMimic>(*(this->mimic));
    }
    return ret;
  }
  bool operator==(const Joint& rhs) const;
  bool operator!=(const Joint& rhs) const;

private:
  std::string name_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

inline std::ostream& operator<<(std::ostream& os, const JointType& type)
{
  switch (type)
  {
    case JointType::FIXED:
    {
      os << "Fixed";
      break;
    }
    case JointType::PLANAR:
    {
      os << "Planar";
      break;
    }
    case JointType::FLOATING:
    {
      os << "Floating";
      break;
    }
    case JointType::REVOLUTE:
    {
      os << "Revolute";
      break;
    }
    case JointType::PRISMATIC:
    {
      os << "Prismatic";
      break;
    }
    case JointType::CONTINUOUS:
    {
      os << "Continuous";
      break;
    }
    default:
    {
      os << "Unknown";
      break;
    }
  }
  return os;
}
}  // namespace tesseract_scene_graph

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_scene_graph::JointDynamics, "JointDynamics")
BOOST_CLASS_EXPORT_KEY2(tesseract_scene_graph::JointLimits, "JointLimits")
BOOST_CLASS_EXPORT_KEY2(tesseract_scene_graph::JointSafety, "JointSafety")
BOOST_CLASS_EXPORT_KEY2(tesseract_scene_graph::JointCalibration, "JointCalibration")
BOOST_CLASS_EXPORT_KEY2(tesseract_scene_graph::JointMimic, "JointMimic")
BOOST_CLASS_EXPORT_KEY2(tesseract_scene_graph::Joint, "Joint")

#endif  // TESSERACT_SCENE_GRAPH_JOINT_H
