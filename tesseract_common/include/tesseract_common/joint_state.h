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
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/serialization.h>

namespace tesseract_common
{
struct JointState
{
  JointState() = default;
  JointState(std::vector<std::string> joint_names, Eigen::VectorXd position)
    : joint_names(std::move(joint_names)), position(std::move(position))
  {
  }

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

  bool operator==(const JointState& other) const
  {
    bool ret_val = true;
    ret_val &= (joint_names == other.joint_names);
    ret_val &= (position.isApprox(other.position, 1e-5));
    ret_val &= (velocity.isApprox(other.velocity, 1e-5));
    ret_val &= (acceleration.isApprox(other.acceleration, 1e-5));
    ret_val &= (effort.isApprox(other.effort, 1e-5));
    ret_val &= (tesseract_common::almostEqualRelativeAndAbs(time, other.time, 1e-5));
    return ret_val;
  }

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_NVP(joint_names);
    ar& BOOST_SERIALIZATION_NVP(position);
    ar& BOOST_SERIALIZATION_NVP(velocity);
    ar& BOOST_SERIALIZATION_NVP(acceleration);
    ar& BOOST_SERIALIZATION_NVP(effort);
    ar& BOOST_SERIALIZATION_NVP(time);
  }
};

/** @brief Represents a joint trajectory */
using JointTrajectory = std::vector<JointState>;
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_JOINT_STATE_H
