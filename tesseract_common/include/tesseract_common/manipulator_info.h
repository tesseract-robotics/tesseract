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
#include <Eigen/Geometry>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/** @brief Manipulator Info Tool Center Point Definition */
class ToolCenterPoint
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  ToolCenterPoint() = default;

  /**
   * @brief Tool Center Point Defined by name
   * @param name The tool center point name
   * @param external The external TCP is used when the robot is holding the part versus the tool.
   * @note External should also be set to true when your kinematic object includes a robot and
   * positioner, where the positioner has the tool and the robot is holding the part. Basically
   * anytime the tool is not attached to the tip link of the kinematic chain.
   */
  ToolCenterPoint(const std::string& name, bool external = false);

  /**
   * @brief Tool Center Point Defined by transform
   * @param transform The tool center point transform
   * @param external The external TCP is used when the robot is holding the part versus the tool.
   * @param external_frame If empty assumed relative to world, otherwise the provided tcp is relative to this link.
   * @note External should also be set to true when your kinematic object includes a robot and
   * positioner, where the positioner has the tool and the robot is holding the part. Basically
   * anytime the tool is not attached to the tip link of the kinematic chain.
   */
  ToolCenterPoint(const Eigen::Isometry3d& transform, bool external = false, std::string external_frame = "");

  /**
   * @brief The Tool Center Point is empty
   * @return True if empty otherwise false
   */
  bool empty() const;

  /**
   * @brief Check if tool center point is defined by name
   * @return True if constructed with name otherwise false
   */
  bool isString() const;

  /**
   * @brief Check if tool center point is defined by transform
   * @return True if constructed with transform otherwise false
   */
  bool isTransform() const;

  /**
   * @brief Check if tool center point is and external tcp which mean it is not defined
   * @details The external TCP is used when the robot is holding the part versus the tool.
   * External should also be set to true when your kinematic object includes a robot and
   * positioner, where the positioner has the tool and the robot is holding the part. Basically
   * anytime the tool is not attached to the tip link of the kinematic chain.
   * @return True if and external TCP, otherwise the false
   */
  bool isExternal() const;

  /**
   * @brief Set the external property
   * @param value True if external tool center point, otherwise false
   * @param external_frame If an external tcp was defined as an Isometry then an external can be provided. If empty
   * assumed relative to world.
   */
  void setExternal(bool value, std::string external_frame = "");

  /**
   * @brief If an external tcp was defined as an Isometry then an external frame can be provided.
   * If empty assumed relative to world.
   */
  const std::string& getExternalFrame() const;

  /**
   * @brief Get the tool center point name
   * @return Name
   */
  const std::string& getString() const;

  /**
   * @brief Get the tool center point transform
   * @return Transform
   */
  const Eigen::Isometry3d& getTransform() const;

  bool operator==(const ToolCenterPoint& rhs) const;
  bool operator!=(const ToolCenterPoint& rhs) const;

protected:
  int type_{ 0 };
  std::string name_;
  Eigen::Isometry3d transform_{ Eigen::Isometry3d::Identity() };

  /**
   * @brief The external TCP is used when the robot is holding the part versus the tool.
   * @details External should also be set to true when your kinematic object includes a robot and
   * positioner, where the positioner has the tool and the robot is holding the part. Basically
   * anytime the tool is not attached to the tip link of the kinematic chain.
   */
  bool external_{ false };

  /**
   * @brief If an external tcp was defined as an Isometry then an external can be provided. If empty assumed relative to
   * world.
   */
  std::string external_frame_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/**
 * @brief Contains information about a robot manipulator
 */
struct ManipulatorInfo
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  ManipulatorInfo() = default;
  ManipulatorInfo(std::string manipulator);

  /** @brief Name of the manipulator group */
  std::string manipulator;

  /** @brief (Optional) IK Solver to be used */
  std::string manipulator_ik_solver;

  /** @brief (Optional) The tool center point, if of type bool it is empty */
  ToolCenterPoint tcp;

  /** @brief (Optional) The working frame to which waypoints are relative. If empty the base link of the environment is
   * used*/
  std::string working_frame;

  /**
   * @brief If the provided manipulator information member is not empty it will override this and return a
   * new manipualtor information with the combined results
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
