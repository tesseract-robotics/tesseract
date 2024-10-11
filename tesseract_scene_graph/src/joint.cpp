/**
 * @file joint.cpp
 * @brief TesseractJoint
 *
 * @author Levi Armstrong
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/joint.h>

namespace tesseract_scene_graph
{
/*********************************************************/
/******              JointDynamics                   *****/
/*********************************************************/
JointDynamics::JointDynamics(double damping, double friction) : damping(damping), friction(friction) {}

void JointDynamics::clear()
{
  damping = 0;
  friction = 0;
}

bool JointDynamics::operator==(const JointDynamics& rhs) const
{
  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(damping, rhs.damping);
  equal &= tesseract_common::almostEqualRelativeAndAbs(friction, rhs.friction);

  return equal;
}
bool JointDynamics::operator!=(const JointDynamics& rhs) const { return !operator==(rhs); }

template <class Archive>
void JointDynamics::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(damping);
  ar& BOOST_SERIALIZATION_NVP(friction);
}

std::ostream& operator<<(std::ostream& os, const JointDynamics& dynamics)
{
  os << "damping=" << dynamics.damping << " friction=" << dynamics.friction;
  return os;
}

/*********************************************************/
/******              JointLimits                     *****/
/*********************************************************/

JointLimits::JointLimits(double l, double u, double e, double v, double a, double j)
  : lower(l), upper(u), effort(e), velocity(v), acceleration(a), jerk(j)
{
}

void JointLimits::clear()
{
  lower = 0;
  upper = 0;
  effort = 0;
  velocity = 0;
  acceleration = 0;
  jerk = 0;
}

bool JointLimits::operator==(const JointLimits& rhs) const
{
  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(lower, rhs.lower);
  equal &= tesseract_common::almostEqualRelativeAndAbs(upper, rhs.upper);
  equal &= tesseract_common::almostEqualRelativeAndAbs(effort, rhs.effort);
  equal &= tesseract_common::almostEqualRelativeAndAbs(velocity, rhs.velocity);
  equal &= tesseract_common::almostEqualRelativeAndAbs(acceleration, rhs.acceleration);
  equal &= tesseract_common::almostEqualRelativeAndAbs(jerk, rhs.jerk);

  return equal;
}
bool JointLimits::operator!=(const JointLimits& rhs) const { return !operator==(rhs); }

template <class Archive>
void JointLimits::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(lower);
  ar& BOOST_SERIALIZATION_NVP(upper);
  ar& BOOST_SERIALIZATION_NVP(effort);
  ar& BOOST_SERIALIZATION_NVP(velocity);
  ar& BOOST_SERIALIZATION_NVP(acceleration);
  ar& BOOST_SERIALIZATION_NVP(jerk);
}

std::ostream& operator<<(std::ostream& os, const JointLimits& limits)
{
  os << "lower=" << limits.lower << " upper=" << limits.upper << " effort=" << limits.effort
     << " velocity=" << limits.velocity << " acceleration=" << limits.acceleration << " jerk=" << limits.jerk;
  ;
  return os;
}

/*********************************************************/
/******              JointSafety                     *****/
/*********************************************************/

JointSafety::JointSafety(double soft_upper_limit, double soft_lower_limit, double k_position, double k_velocity)
  : soft_upper_limit(soft_upper_limit)
  , soft_lower_limit(soft_lower_limit)
  , k_position(k_position)
  , k_velocity(k_velocity)
{
}

void JointSafety::clear()
{
  soft_upper_limit = 0;
  soft_lower_limit = 0;
  k_position = 0;
  k_velocity = 0;
}

bool JointSafety::operator==(const JointSafety& rhs) const
{
  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(soft_upper_limit, rhs.soft_upper_limit);
  equal &= tesseract_common::almostEqualRelativeAndAbs(soft_lower_limit, rhs.soft_lower_limit);
  equal &= tesseract_common::almostEqualRelativeAndAbs(k_position, rhs.k_position);
  equal &= tesseract_common::almostEqualRelativeAndAbs(k_velocity, rhs.k_velocity);

  return equal;
}
bool JointSafety::operator!=(const JointSafety& rhs) const { return !operator==(rhs); }

template <class Archive>
void JointSafety::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(soft_upper_limit);
  ar& BOOST_SERIALIZATION_NVP(soft_lower_limit);
  ar& BOOST_SERIALIZATION_NVP(k_position);
  ar& BOOST_SERIALIZATION_NVP(k_velocity);
}

std::ostream& operator<<(std::ostream& os, const JointSafety& safety)
{
  os << "soft_upper_limit=" << safety.soft_upper_limit << " soft_lower_limit=" << safety.soft_lower_limit
     << " k_position=" << safety.k_position << " k_velocity=" << safety.k_velocity;
  return os;
}

/*********************************************************/
/******              JointCalibration                *****/
/*********************************************************/
JointCalibration::JointCalibration(double reference_position, double rising, double falling)
  : reference_position(reference_position), rising(rising), falling(falling)
{
}

void JointCalibration::clear()
{
  reference_position = 0;
  rising = 0;
  falling = 0;
}

bool JointCalibration::operator==(const JointCalibration& rhs) const
{
  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(reference_position, rhs.reference_position);
  equal &= tesseract_common::almostEqualRelativeAndAbs(rising, rhs.rising);
  equal &= tesseract_common::almostEqualRelativeAndAbs(falling, rhs.falling);

  return equal;
}
bool JointCalibration::operator!=(const JointCalibration& rhs) const { return !operator==(rhs); }

template <class Archive>
void JointCalibration::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(reference_position);
  ar& BOOST_SERIALIZATION_NVP(rising);
  ar& BOOST_SERIALIZATION_NVP(falling);
}

std::ostream& operator<<(std::ostream& os, const JointCalibration& calibration)
{
  os << "reference_position=" << calibration.reference_position << " rising=" << calibration.rising
     << " falling=" << calibration.falling;
  return os;
}

/*********************************************************/
/******                  JointMimic                  *****/
/*********************************************************/
JointMimic::JointMimic(double offset, double multiplier, std::string joint_name)
  : offset(offset), multiplier(multiplier), joint_name(std::move(joint_name))
{
}

void JointMimic::clear()
{
  offset = 0.0;
  multiplier = 1.0;
  joint_name.clear();
}

bool JointMimic::operator==(const JointMimic& rhs) const
{
  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(offset, rhs.offset);
  equal &= tesseract_common::almostEqualRelativeAndAbs(multiplier, rhs.multiplier);
  equal &= joint_name == rhs.joint_name;

  return equal;
}
bool JointMimic::operator!=(const JointMimic& rhs) const { return !operator==(rhs); }

template <class Archive>
void JointMimic::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(offset);
  ar& BOOST_SERIALIZATION_NVP(multiplier);
  ar& BOOST_SERIALIZATION_NVP(joint_name);
}

std::ostream& operator<<(std::ostream& os, const JointMimic& mimic)
{
  os << "joint_name=" << mimic.joint_name << " offset=" << mimic.offset << " multiplier=" << mimic.multiplier;
  return os;
}

/*********************************************************/
/******                     Joint                    *****/
/*********************************************************/
Joint::Joint(std::string name) : name_(std::move(name)) { this->clear(); }

const std::string& Joint::getName() const { return name_; }

void Joint::clear()
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

Joint Joint::clone() const { return clone(name_); }

Joint Joint::clone(const std::string& name) const
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

bool Joint::operator==(const Joint& rhs) const
{
  bool equal = true;
  equal &= type == rhs.type;
  equal &= tesseract_common::almostEqualRelativeAndAbs(axis, rhs.axis);
  equal &= child_link_name == rhs.child_link_name;
  equal &= parent_link_name == rhs.parent_link_name;
  equal &= parent_to_joint_origin_transform.isApprox(rhs.parent_to_joint_origin_transform, 1e-5);
  equal &= tesseract_common::pointersEqual(dynamics, rhs.dynamics);
  equal &= tesseract_common::pointersEqual(limits, rhs.limits);
  equal &= tesseract_common::pointersEqual(safety, rhs.safety);
  equal &= tesseract_common::pointersEqual(calibration, rhs.calibration);
  equal &= tesseract_common::pointersEqual(mimic, rhs.mimic);
  equal &= name_ == rhs.name_;
  return equal;
}
bool Joint::operator!=(const Joint& rhs) const { return !operator==(rhs); }

template <class Archive>
void Joint::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(type);
  ar& BOOST_SERIALIZATION_NVP(axis);
  ar& BOOST_SERIALIZATION_NVP(child_link_name);
  ar& BOOST_SERIALIZATION_NVP(parent_link_name);
  ar& BOOST_SERIALIZATION_NVP(parent_to_joint_origin_transform);
  ar& BOOST_SERIALIZATION_NVP(dynamics);
  ar& BOOST_SERIALIZATION_NVP(limits);
  ar& BOOST_SERIALIZATION_NVP(safety);
  ar& BOOST_SERIALIZATION_NVP(calibration);
  ar& BOOST_SERIALIZATION_NVP(mimic);
  ar& BOOST_SERIALIZATION_NVP(name_);
}

std::ostream& operator<<(std::ostream& os, const JointType& type)
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

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointDynamics)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointLimits)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointSafety)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointCalibration)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointMimic)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::Joint)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointDynamics)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointLimits)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointSafety)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointCalibration)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointMimic)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::Joint)
