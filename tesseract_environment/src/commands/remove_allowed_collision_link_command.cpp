/**
 * @file remove_allowed_collision_link_command.cpp
 * @brief Used to remove an allowed collision from the acm for a link
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
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_environment/commands/remove_allowed_collision_link_command.h>

namespace tesseract_environment
{
RemoveAllowedCollisionLinkCommand::RemoveAllowedCollisionLinkCommand()
  : Command(CommandType::REMOVE_ALLOWED_COLLISION_LINK)
{
}

RemoveAllowedCollisionLinkCommand::RemoveAllowedCollisionLinkCommand(std::string link_name)
  : Command(CommandType::REMOVE_ALLOWED_COLLISION_LINK), link_name_(std::move(link_name))
{
}

const std::string& RemoveAllowedCollisionLinkCommand::getLinkName() const { return link_name_; }

bool RemoveAllowedCollisionLinkCommand::operator==(const RemoveAllowedCollisionLinkCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= link_name_ == rhs.link_name_;
  return equal;
}
bool RemoveAllowedCollisionLinkCommand::operator!=(const RemoveAllowedCollisionLinkCommand& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void RemoveAllowedCollisionLinkCommand::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  ar& BOOST_SERIALIZATION_NVP(link_name_);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::RemoveAllowedCollisionLinkCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::RemoveAllowedCollisionLinkCommand)
