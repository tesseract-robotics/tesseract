/**
 * @file replace_joint_command.cpp
 * @brief Used to replace a joint in the environment
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 18, 2022
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

#include <tesseract_environment/commands/replace_joint_command.h>
#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/joint.h>

#include <memory>

namespace tesseract_environment
{
ReplaceJointCommand::ReplaceJointCommand() : Command(CommandType::REPLACE_JOINT) {}

ReplaceJointCommand::ReplaceJointCommand(const tesseract_scene_graph::Joint& joint)
  : Command(CommandType::REPLACE_JOINT), joint_(std::make_shared<tesseract_scene_graph::Joint>(joint.clone()))
{
  if (joint_->type != tesseract_scene_graph::JointType::FIXED)
  {
    //      if ()
    /** @todo check limits */
  }
}

const tesseract_scene_graph::Joint::ConstPtr& ReplaceJointCommand::getJoint() const { return joint_; }

bool ReplaceJointCommand::operator==(const ReplaceJointCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= tesseract_common::pointersEqual(joint_, rhs.joint_);
  return equal;
}
bool ReplaceJointCommand::operator!=(const ReplaceJointCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_environment
