/**
 * @file change_link_origin_command.h
 * @brief Used to change a links origin in environment
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
#ifndef TESSERACT_ENVIRONMENT_CHANGE_LINK_ORIGIN_COMMAND_H
#define TESSERACT_ENVIRONMENT_CHANGE_LINK_ORIGIN_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>

namespace tesseract_environment
{
class ChangeLinkOriginCommand : public Command
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<ChangeLinkOriginCommand>;
  using ConstPtr = std::shared_ptr<const ChangeLinkOriginCommand>;

  ChangeLinkOriginCommand(std::string link_name, const Eigen::Isometry3d& origin)
    : link_name_(std::move(link_name)), origin_(origin)
  {
  }

  CommandType getType() const final { return CommandType::CHANGE_LINK_ORIGIN; }
  const std::string& getLinkName() const { return link_name_; }
  const Eigen::Isometry3d& getOrigin() const { return origin_; }

private:
  std::string link_name_;
  Eigen::Isometry3d origin_;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_CHANGE_LINK_ORIGIN_COMMAND_H
