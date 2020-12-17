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
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>

namespace tesseract_common
{
/** @brief Manipulator Info Tool Center Point Definition */
class ToolCenterPoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ToolCenterPoint() = default;

  /**
   * @brief Tool Center Point Defined by name
   * @param name The tool center point name
   * @param external The external TCP is used when the robot is holding the part verus the tool.
   */
  ToolCenterPoint(const std::string& name, bool external = false);

  /**
   * @brief Tool Center Point Defined by transform
   * @param transform The tool center point transform
   * @param external The external TCP is used when the robot is holding the part verus the tool.
   */
  ToolCenterPoint(const Eigen::Isometry3d& transform, bool external = false);

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
   * @details The external TCP is used when the robot is holding the part verus the tool.
   * @return True if and external TCP, otherwise the false
   */
  bool isExternal() const;

  /**
   * @brief Set the external property
   * @param value True if external tool center point, otherwise false
   */
  void setExternal(bool value);

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

  bool operator==(const ToolCenterPoint& other) const
  {
    bool ret_val = true;
    ret_val &= (type_ == other.type_);
    ret_val &= (name_ == other.name_);
    ret_val &= (transform_.isApprox(transform_, 1e-5));
    ret_val &= (external_ == other.external_);
    return ret_val;
  }

protected:
  int type_{ 0 };
  std::string name_;
  Eigen::Isometry3d transform_;

  /** @brief The external TCP is used when the robot is holding the part verus the tool. */
  bool external_{ false };
};

/**
 * @brief Contains information about a robot manipulator
 */
struct ManipulatorInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ManipulatorInfo() = default;
  ManipulatorInfo(std::string manipulator);
  ManipulatorInfo(const tinyxml2::XMLElement& xml_element);

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

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const;

  bool operator==(const ManipulatorInfo& other) const
  {
    bool ret_val = true;
    ret_val &= (manipulator == other.manipulator);
    ret_val &= (manipulator_ik_solver == other.manipulator_ik_solver);
    ret_val &= (tcp == other.tcp);
    ret_val &= (working_frame == other.working_frame);
    return ret_val;
  }
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_MANIPULATOR_INFO_H
