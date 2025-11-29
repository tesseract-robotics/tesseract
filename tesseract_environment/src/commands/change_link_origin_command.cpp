/**
 * @file change_link_origin_command.cpp
 * @brief Used to change a link origin
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

#include <tesseract_common/utils.h>
#include <tesseract_environment/commands/change_link_origin_command.h>

#include <string>

namespace tesseract_environment
{
ChangeLinkOriginCommand::ChangeLinkOriginCommand() : Command(CommandType::CHANGE_LINK_ORIGIN) {}

// NOLINTNEXTLINE(modernize-pass-by-value)
ChangeLinkOriginCommand::ChangeLinkOriginCommand(std::string link_name, const Eigen::Isometry3d& origin)
  : Command(CommandType::CHANGE_LINK_ORIGIN), link_name_(std::move(link_name)), origin_(origin)
{
}

const std::string& ChangeLinkOriginCommand::getLinkName() const { return link_name_; }
const Eigen::Isometry3d& ChangeLinkOriginCommand::getOrigin() const { return origin_; }

bool ChangeLinkOriginCommand::operator==(const ChangeLinkOriginCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= link_name_ == rhs.link_name_;
  equal &= origin_.isApprox(rhs.origin_, 1e-5);
  return equal;
}
bool ChangeLinkOriginCommand::operator!=(const ChangeLinkOriginCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_environment
