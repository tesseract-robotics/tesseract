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

  JointDynamics() { this->clear(); }
  double damping;
  double friction;

  void clear()
  {
    damping = 0;
    friction = 0;
  }
};

class JointLimits
{
public:
  using Ptr = std::shared_ptr<JointLimits>;
  using ConstPtr = std::shared_ptr<const JointLimits>;

  JointLimits() { this->clear(); }
  double lower;
  double upper;
  double effort;
  double velocity;

  void clear()
  {
    lower = 0;
    upper = 0;
    effort = 0;
    velocity = 0;
  }
};

/// \brief Parameters for Joint Safety Controllers
class JointSafety
{
public:
  using Ptr = std::shared_ptr<JointSafety>;
  using ConstPtr = std::shared_ptr<const JointSafety>;

  /// clear variables on construction
  JointSafety() { this->clear(); }

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
  double soft_upper_limit;
  double soft_lower_limit;
  double k_position;
  double k_velocity;

  void clear()
  {
    soft_upper_limit = 0;
    soft_lower_limit = 0;
    k_position = 0;
    k_velocity = 0;
  }
};

class JointCalibration
{
public:
  using Ptr = std::shared_ptr<JointCalibration>;
  using ConstPtr = std::shared_ptr<const JointCalibration>;

  JointCalibration() { this->clear(); }
  double reference_position;
  double rising, falling;

  void clear()
  {
    reference_position = 0;
    rising = 0;
    falling = 0;
  }
};

class JointMimic
{
public:
  using Ptr = std::shared_ptr<JointMimic>;
  using ConstPtr = std::shared_ptr<const JointMimic>;

  JointMimic() { this->clear(); }
  double offset;
  double multiplier;
  std::string joint_name;

  void clear()
  {
    offset = 0.0;
    multiplier = 1.0;
    joint_name.clear();
  }
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

  const std::string& getName() const { return name_; }

  /// The type of joint
  JointType type;

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
  Eigen::Isometry3d parent_to_joint_origin_transform;

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

private:
  const std::string name_;
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

#endif  // TESSERACT_SCENE_GRAPH_JOINT_H
