/**
 * @file move_link_command.cpp
 * @brief Used to move a link in the environment
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/commands/move_link_command.h>
#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/joint.h>

namespace tesseract_environment
{
MoveLinkCommand::MoveLinkCommand() : Command(CommandType::MOVE_LINK) {}

MoveLinkCommand::MoveLinkCommand(const tesseract_scene_graph::Joint& joint)
  : Command(CommandType::MOVE_LINK), joint_(std::make_shared<tesseract_scene_graph::Joint>(joint.clone()))
{
}

const tesseract_scene_graph::Joint::ConstPtr& MoveLinkCommand::getJoint() const { return joint_; }

bool MoveLinkCommand::operator==(const MoveLinkCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= tesseract_common::pointersEqual(joint_, rhs.joint_);
  return equal;
}
bool MoveLinkCommand::operator!=(const MoveLinkCommand& rhs) const { return !operator==(rhs); }

template <class Archive>
void MoveLinkCommand::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  ar& BOOST_SERIALIZATION_NVP(joint_);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::MoveLinkCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::MoveLinkCommand)
