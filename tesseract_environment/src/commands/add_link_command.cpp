/**
 * @file add_link_command.cpp
 * @brief Used to add a link to the environment
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

#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

#include <memory>

namespace tesseract_environment
{
AddLinkCommand::AddLinkCommand() : Command(CommandType::ADD_LINK) {}

AddLinkCommand::AddLinkCommand(const tesseract_scene_graph::Link& link, bool replace_allowed)
  : Command(CommandType::ADD_LINK)
  , link_(std::make_shared<tesseract_scene_graph::Link>(link.clone()))
  , joint_(nullptr)
  , replace_allowed_(replace_allowed)
{
}

AddLinkCommand::AddLinkCommand(const tesseract_scene_graph::Link& link,
                               const tesseract_scene_graph::Joint& joint,
                               bool replace_allowed)
  : Command(CommandType::ADD_LINK)
  , link_(std::make_shared<tesseract_scene_graph::Link>(link.clone()))
  , joint_(std::make_shared<tesseract_scene_graph::Joint>(joint.clone()))
  , replace_allowed_(replace_allowed)
{
  if (joint_->child_link_name != link.getName())
    throw std::runtime_error("AddLinkCommand: The provided joint child link name must equal the name of the provided "
                             "link.");

  /** @todo if joint is not fixed we should verify that limits are provided */
}

const std::shared_ptr<const tesseract_scene_graph::Link>& AddLinkCommand::getLink() const { return link_; }
const std::shared_ptr<const tesseract_scene_graph::Joint>& AddLinkCommand::getJoint() const { return joint_; }
bool AddLinkCommand::replaceAllowed() const { return replace_allowed_; }

bool AddLinkCommand::operator==(const AddLinkCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= tesseract_common::pointersEqual(link_, rhs.link_);
  equal &= tesseract_common::pointersEqual(joint_, rhs.joint_);
  equal &= replace_allowed_ == rhs.replace_allowed_;
  return equal;
}
bool AddLinkCommand::operator!=(const AddLinkCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_environment
