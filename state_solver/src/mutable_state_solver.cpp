/**
 * @file mutable_state_solver.cpp
 * @brief Default implementations of MutableStateSolver ID overloads.
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
#include <tesseract/state_solver/mutable_state_solver.h>

namespace tesseract::scene_graph
{
bool MutableStateSolver::removeLink(const tesseract::common::LinkId& link_id) { return removeLink(link_id.name()); }

bool MutableStateSolver::removeJoint(const tesseract::common::JointId& joint_id)
{
  return removeJoint(joint_id.name());
}

bool MutableStateSolver::moveJoint(const tesseract::common::JointId& joint_id,
                                   const tesseract::common::LinkId& parent_link_id)
{
  return moveJoint(joint_id.name(), parent_link_id.name());
}

bool MutableStateSolver::changeJointOrigin(const tesseract::common::JointId& joint_id,
                                           const Eigen::Isometry3d& new_origin)
{
  return changeJointOrigin(joint_id.name(), new_origin);
}

bool MutableStateSolver::changeJointPositionLimits(const tesseract::common::JointId& joint_id,
                                                   double lower,
                                                   double upper)
{
  return changeJointPositionLimits(joint_id.name(), lower, upper);
}

bool MutableStateSolver::changeJointVelocityLimits(const tesseract::common::JointId& joint_id, double limit)
{
  return changeJointVelocityLimits(joint_id.name(), limit);
}

bool MutableStateSolver::changeJointAccelerationLimits(const tesseract::common::JointId& joint_id, double limit)
{
  return changeJointAccelerationLimits(joint_id.name(), limit);
}

bool MutableStateSolver::changeJointJerkLimits(const tesseract::common::JointId& joint_id, double limit)
{
  return changeJointJerkLimits(joint_id.name(), limit);
}
}  // namespace tesseract::scene_graph
