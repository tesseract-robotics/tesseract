/**
 * @file manipulator_info.h
 * @brief
 *
 * @author Levi Armstrong
 * @date July 22, 2020
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
#ifndef TESSERACT_COMMON_MANIPULATOR_INFO_H
#define TESSERACT_COMMON_MANIPULATOR_INFO_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <variant>
#include <Eigen/Geometry>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/**
 * @brief Contains information about a robot manipulator
 */
struct ManipulatorInfo
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  ManipulatorInfo() = default;
  ManipulatorInfo(std::string manipulator_,
                  std::string working_frame_,
                  std::string tcp_frame_,
                  const Eigen::Isometry3d& tcp_offset_ = Eigen::Isometry3d::Identity());

  /** @brief Name of the manipulator group */
  std::string manipulator;

  /**
   * @brief The working frame to which waypoints are relative.
   * @details If the tcp_frame is external to manipulator then the working frame must be an active frame on the
   * manipulator
   */
  std::string working_frame;

  /**
   * @brief The coordinate frame within to the environment to use as the reference frame for the tool center
   * point (TCP) which is defined by an offset transform relative to this frame
   */
  std::string tcp_frame;

  /** @brief (Optional) Offset transform applied to the tcp_frame link to represent the manipulator TCP */
  std::variant<std::string, Eigen::Isometry3d> tcp_offset{ Eigen::Isometry3d::Identity() };

  /** @brief (Optional) IK Solver to be used */
  std::string manipulator_ik_solver;

  /**
   * @brief If the provided manipulator information member is not empty it will override this and return a
   * new manipulator information with the combined results
   * @param manip_info_override The manipulator information to check for overrides
   * @return The combined manipulator information
   */
  ManipulatorInfo getCombined(const ManipulatorInfo& manip_info_override) const;

  /**
   * @brief Check if any data is current being stored
   * @return True if empty otherwise false
   */
  bool empty() const;

  bool operator==(const ManipulatorInfo& rhs) const;
  bool operator!=(const ManipulatorInfo& rhs) const;

private:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_MANIPULATOR_INFO_H
