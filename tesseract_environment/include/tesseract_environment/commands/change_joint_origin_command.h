/**
 * @file change_joint_origin_command.h
 * @brief Used to change a joints origin in environment
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ENVIRONMENT_CHANGE_JOINT_ORIGIN_COMMAND_H
#define TESSERACT_ENVIRONMENT_CHANGE_JOINT_ORIGIN_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>

namespace tesseract_environment
{
class ChangeJointOriginCommand : public Command
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<ChangeJointOriginCommand>;
  using ConstPtr = std::shared_ptr<const ChangeJointOriginCommand>;

  ChangeJointOriginCommand();

  /**
   * @brief Changes the origin associated with the joint
   *
   * Note: This is the origin as in the "origin" tag in the URDF. This is the location of the
   * joint in the frame of the parent link.
   * @param joint_name Name of the joint to be updated
   * @param new_origin New transform to be set as the origin
   */
  ChangeJointOriginCommand(std::string joint_name, const Eigen::Isometry3d& origin);

  const std::string& getJointName() const;
  const Eigen::Isometry3d& getOrigin() const;

  bool operator==(const ChangeJointOriginCommand& rhs) const;
  bool operator!=(const ChangeJointOriginCommand& rhs) const;

private:
  std::string joint_name_;
  Eigen::Isometry3d origin_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_environment

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_environment::ChangeJointOriginCommand, "ChangeJointOriginCommand")
#endif  // TESSERACT_ENVIRONMENT_CHANGE_JOINT_ORIGIN_COMMAND_H
