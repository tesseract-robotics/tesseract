/**
 * @file manipulator_info.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <stdexcept>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>

namespace tesseract_common
{
ToolCenterPoint::ToolCenterPoint(const std::string& name, bool external) : type_(1), name_(name), external_(external) {}

ToolCenterPoint::ToolCenterPoint(const Eigen::Isometry3d& transform, bool external, std::string external_frame)
  : type_(2), transform_(transform), external_(external), external_frame_(std::move(external_frame))
{
}

bool ToolCenterPoint::empty() const { return (type_ == 0); }
bool ToolCenterPoint::isString() const { return (type_ == 1); }
bool ToolCenterPoint::isTransform() const { return (type_ == 2); }
bool ToolCenterPoint::isExternal() const { return external_; }

const std::string& ToolCenterPoint::getExternalFrame() const
{
  if (isTransform() && external_)
    return external_frame_;

  throw std::runtime_error("ToolCenterPoint: Called getExternalFrame for invalid type.");
}
void ToolCenterPoint::setExternal(bool value, std::string external_frame)
{
  external_ = value;

  // External frame is only valid when a Isometry3d tcp is provided.
  if (isTransform() && value)
    external_frame_ = std::move(external_frame);
  else
    external_frame_.clear();
}

const std::string& ToolCenterPoint::getString() const
{
  if (type_ != 1)
    throw std::runtime_error("ToolCenterPoint: Called getString for invalid type.");

  return name_;
}
const Eigen::Isometry3d& ToolCenterPoint::getTransform() const
{
  if (type_ != 2)
    throw std::runtime_error("ToolCenterPoint: Called getTransform for invalid type.");

  return transform_;
}

bool ToolCenterPoint::operator==(const ToolCenterPoint& rhs) const
{
  bool ret_val = true;
  ret_val &= (type_ == rhs.type_);
  ret_val &= (name_ == rhs.name_);
  ret_val &= (transform_.isApprox(rhs.transform_, 1e-5));
  ret_val &= (external_ == rhs.external_);
  ret_val &= (external_frame_ == rhs.external_frame_);
  return ret_val;
}
bool ToolCenterPoint::operator!=(const ToolCenterPoint& rhs) const { return !operator==(rhs); }

template <class Archive>
void ToolCenterPoint::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("type", type_);
  ar& boost::serialization::make_nvp("name", name_);
  ar& boost::serialization::make_nvp("transform", transform_);
  ar& boost::serialization::make_nvp("external", external_);
  ar& boost::serialization::make_nvp("external_frame", external_frame_);
}

ManipulatorInfo::ManipulatorInfo(std::string manipulator) : manipulator(std::move(manipulator)) {}

ManipulatorInfo ManipulatorInfo::getCombined(const ManipulatorInfo& manip_info_override) const
{
  ManipulatorInfo combined = *this;

  if (!manip_info_override.manipulator.empty())
    combined.manipulator = manip_info_override.manipulator;

  if (!manip_info_override.manipulator_ik_solver.empty())
    combined.manipulator_ik_solver = manip_info_override.manipulator_ik_solver;

  if (!manip_info_override.working_frame.empty())
    combined.working_frame = manip_info_override.working_frame;

  if (!manip_info_override.tcp.empty())
    combined.tcp = manip_info_override.tcp;

  return combined;
}

bool ManipulatorInfo::empty() const
{
  if (!manipulator.empty())
    return false;

  if (!manipulator_ik_solver.empty())
    return false;

  if (!working_frame.empty())
    return false;

  if (!tcp.empty())
    return false;

  return true;
}

bool ManipulatorInfo::operator==(const ManipulatorInfo& rhs) const
{
  bool ret_val = true;
  ret_val &= (manipulator == rhs.manipulator);
  ret_val &= (manipulator_ik_solver == rhs.manipulator_ik_solver);
  ret_val &= (tcp == rhs.tcp);
  ret_val &= (working_frame == rhs.working_frame);
  return ret_val;
}
bool ManipulatorInfo::operator!=(const ManipulatorInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void ManipulatorInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("manipulator", manipulator);
  ar& boost::serialization::make_nvp("manipulator_ik_solver", manipulator_ik_solver);
  ar& boost::serialization::make_nvp("tcp", tcp);
  ar& boost::serialization::make_nvp("working_frame", working_frame);
}

}  // namespace tesseract_common

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

template void tesseract_common::ToolCenterPoint::serialize(boost::archive::xml_oarchive& ar,
                                                           const unsigned int version);
template void tesseract_common::ToolCenterPoint::serialize(boost::archive::xml_iarchive& ar,
                                                           const unsigned int version);

template void tesseract_common::ManipulatorInfo::serialize(boost::archive::xml_oarchive& ar,
                                                           const unsigned int version);
template void tesseract_common::ManipulatorInfo::serialize(boost::archive::xml_iarchive& ar,
                                                           const unsigned int version);
