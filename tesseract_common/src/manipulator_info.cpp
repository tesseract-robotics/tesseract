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
ManipulatorInfo::ManipulatorInfo(std::string manipulator_,
                                 std::string working_frame_,
                                 std::string tcp_frame_,
                                 const Eigen::Isometry3d& tcp_offset_)
  : manipulator(std::move(manipulator_))
  , working_frame(std::move(working_frame_))
  , tcp_frame(std::move(tcp_frame_))
  , tcp_offset(tcp_offset_)
{
}

ManipulatorInfo ManipulatorInfo::getCombined(const ManipulatorInfo& manip_info_override) const
{
  ManipulatorInfo combined = *this;

  if (!manip_info_override.manipulator.empty())
    combined.manipulator = manip_info_override.manipulator;

  if (!manip_info_override.manipulator_ik_solver.empty())
    combined.manipulator_ik_solver = manip_info_override.manipulator_ik_solver;

  if (!manip_info_override.working_frame.empty())
    combined.working_frame = manip_info_override.working_frame;

  if (!manip_info_override.tcp_frame.empty())
  {
    combined.tcp_frame = manip_info_override.tcp_frame;
    combined.tcp_offset = manip_info_override.tcp_offset;
  }

  return combined;
}

bool ManipulatorInfo::empty() const
{
  // This struct is empty if either the manipulator or tcp_frame members are empty since they are required
  return manipulator.empty() || working_frame.empty() || tcp_frame.empty();
}

bool ManipulatorInfo::operator==(const ManipulatorInfo& rhs) const
{
  bool ret_val = true;
  ret_val &= (manipulator == rhs.manipulator);
  ret_val &= (manipulator_ik_solver == rhs.manipulator_ik_solver);
  ret_val &= (working_frame == rhs.working_frame);
  ret_val &= (tcp_frame == rhs.tcp_frame);
  ret_val &= (tcp_offset.index() == rhs.tcp_offset.index());
  if (ret_val)
  {
    if (tcp_offset.index() == 0)
      ret_val &= (std::get<std::string>(tcp_offset) == std::get<std::string>(rhs.tcp_offset));
    else
      ret_val &= std::get<Eigen::Isometry3d>(tcp_offset).isApprox(std::get<Eigen::Isometry3d>(rhs.tcp_offset));
  }

  return ret_val;
}
bool ManipulatorInfo::operator!=(const ManipulatorInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void ManipulatorInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("manipulator", manipulator);
  ar& boost::serialization::make_nvp("manipulator_ik_solver", manipulator_ik_solver);
  ar& boost::serialization::make_nvp("working_frame", working_frame);
  ar& boost::serialization::make_nvp("tcp_frame", tcp_frame);
  ar& boost::serialization::make_nvp("tcp_offset", tcp_offset);
}

}  // namespace tesseract_common

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

template void tesseract_common::ManipulatorInfo::serialize(boost::archive::xml_oarchive& ar,
                                                           const unsigned int version);
template void tesseract_common::ManipulatorInfo::serialize(boost::archive::xml_iarchive& ar,
                                                           const unsigned int version);
