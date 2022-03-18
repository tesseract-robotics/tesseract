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
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/joint.h>

namespace tesseract_scene_graph
{
/*********************************************************/
/******              JointDynamics                   *****/
/*********************************************************/
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

/*********************************************************/
/******              JointLimits                     *****/
/*********************************************************/
bool JointLimits::operator==(const JointLimits& rhs) const
{
  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(lower, rhs.lower);
  equal &= tesseract_common::almostEqualRelativeAndAbs(upper, rhs.upper);
  equal &= tesseract_common::almostEqualRelativeAndAbs(effort, rhs.effort);
  equal &= tesseract_common::almostEqualRelativeAndAbs(velocity, rhs.velocity);
  equal &= tesseract_common::almostEqualRelativeAndAbs(acceleration, rhs.acceleration);

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
}

/*********************************************************/
/******              JointSafety                     *****/
/*********************************************************/
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

/*********************************************************/
/******              JointCalibration                *****/
/*********************************************************/
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

/*********************************************************/
/******                  JointMimic                  *****/
/*********************************************************/
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

/*********************************************************/
/******                     Joint                    *****/
/*********************************************************/
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

}  // namespace tesseract_scene_graph

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointDynamics)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointDynamics)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointLimits)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointLimits)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointSafety)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointSafety)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointCalibration)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointCalibration)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::JointMimic)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::JointMimic)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::Joint)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::Joint)
