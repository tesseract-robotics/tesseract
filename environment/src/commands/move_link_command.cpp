/**
 * @file move_link_command.cpp
 * @brief Used to move a link in the environment
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 18, 2022
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

#include <tesseract/environment/commands/move_link_command.h>
#include <tesseract/common/utils.h>
#include <tesseract/scene_graph/joint.h>

#include <memory>

namespace tesseract::environment
{
MoveLinkCommand::MoveLinkCommand() : Command(CommandType::MOVE_LINK) {}

MoveLinkCommand::MoveLinkCommand(const tesseract::scene_graph::Joint& joint)
  : Command(CommandType::MOVE_LINK), joint_(std::make_shared<tesseract::scene_graph::Joint>(joint.clone()))
{
}

const tesseract::scene_graph::Joint::ConstPtr& MoveLinkCommand::getJoint() const { return joint_; }

bool MoveLinkCommand::operator==(const MoveLinkCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= tesseract::common::pointersEqual(joint_, rhs.joint_);
  return equal;
}
bool MoveLinkCommand::operator!=(const MoveLinkCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::environment
