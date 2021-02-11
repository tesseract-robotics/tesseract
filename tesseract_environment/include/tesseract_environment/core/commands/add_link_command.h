/**
 * @file add_link_command.h
 * @brief Used to add link and joint to environment
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
#ifndef TESSERACT_ENVIRONMENT_ADD_COMMAND_H
#define TESSERACT_ENVIRONMENT_ADD_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/command.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>

namespace tesseract_environment
{
class AddLinkCommand : public Command
{
public:
  using Ptr = std::shared_ptr<AddLinkCommand>;
  using ConstPtr = std::shared_ptr<const AddLinkCommand>;

  /**
   * @brief Adds a link to the environment
   *
   *        This method should attach the link to the root link with a fixed joint
   *        with a joint name of joint_{link name}".
   *
   * @param link The link to be added to the graph
   */
  AddLinkCommand(const tesseract_scene_graph::Link& link)
    : link_(std::make_shared<tesseract_scene_graph::Link>(link.clone())), joint_(nullptr)
  {
  }

  /**
   * @brief Adds a link to the environment
   * @param link The link to be added to the graph
   * @param joint The joint to be used to attach link to environment
   */
  AddLinkCommand(const tesseract_scene_graph::Link& link, const tesseract_scene_graph::Joint& joint)
    : link_(std::make_shared<tesseract_scene_graph::Link>(link.clone()))
    , joint_(std::make_shared<tesseract_scene_graph::Joint>(joint.clone()))
  {
  }

  CommandType getType() const final { return CommandType::ADD_LINK; }
  const tesseract_scene_graph::Link::ConstPtr& getLink() const { return link_; }
  const tesseract_scene_graph::Joint::ConstPtr& getJoint() const { return joint_; }

private:
  tesseract_scene_graph::Link::ConstPtr link_;
  tesseract_scene_graph::Joint::ConstPtr joint_;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_ADD_COMMAND_H
