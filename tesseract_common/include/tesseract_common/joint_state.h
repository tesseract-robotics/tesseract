/**
 * @file joint_state.h
 * @brief Tesseract Joint State
 *
 * @author Levi Armstrong
 * @date January 18, 2018
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
#ifndef TESSERACT_COMMON_JOINT_STATE_H
#define TESSERACT_COMMON_JOINT_STATE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
#include <vector>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
struct JointState
{
  JointState() = default;
  JointState(std::vector<std::string> joint_names, Eigen::VectorXd position);

  /** @brief The joint corresponding to the position vector. */
  std::vector<std::string> joint_names;

  /** @brief The joint position at the waypoint */
  Eigen::VectorXd position;

  /** @brief The velocity at the waypoint (optional) */
  Eigen::VectorXd velocity;

  /** @brief The Acceleration at the waypoint (optional) */
  Eigen::VectorXd acceleration;

  /** @brief The Effort at the waypoint (optional) */
  Eigen::VectorXd effort;

  /** @brief The Time from start at the waypoint (optional) */
  double time{ 0 };

  bool operator==(const JointState& other) const;

  bool operator!=(const JointState& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/** @brief Represents a joint trajectory */
using JointTrajectory = std::vector<JointState>;
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_JOINT_STATE_H
