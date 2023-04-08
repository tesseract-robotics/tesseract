/**
 * @file add_trajectory_link_command.h
 * @brief Used to add trajectory link and joint to environment
 *
 * @author Levi Armstrong
 * @date March 29, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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
#ifndef TESSERACT_ENVIRONMENT_ADD_TRAJECTORY_LINK_COMMAND_H
#define TESSERACT_ENVIRONMENT_ADD_TRAJECTORY_LINK_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_common/joint_state.h>

namespace tesseract_environment
{
class AddTrajectoryLinkCommand : public Command
{
public:
  using Ptr = std::shared_ptr<AddTrajectoryLinkCommand>;
  using ConstPtr = std::shared_ptr<const AddTrajectoryLinkCommand>;

  AddTrajectoryLinkCommand();
  /**
   * @brief Adds or replace a trajectory link to the environment
   *
   * If the link exists and replace_allowed equals true:
   *
   *        This command should replace the current link with the new link
   *
   * If the link exists and replace_allowed equals false:
   *
   *        This command should result in an error
   *
   * If the link does not exist:
   *
   *        This command should attach the link to the parent link with a fixed joint
   *        with a joint name of joint_{link name}".
   *
   * @param link_name The link name
   * @param parent_link_name The parent link name
   * @param trajectory The trajectory to used for generating link
   * @param replace_allowed If true then if the link exists it will be replaced, otherwise if false it will fail.
   */
  AddTrajectoryLinkCommand(std::string link_name,
                           std::string parent_link_name,
                           tesseract_common::JointTrajectory trajectory,
                           bool replace_allowed = false);

  const std::string& getLinkName() const;
  const std::string& getParentLinkName() const;
  const tesseract_common::JointTrajectory& getTrajectory() const;
  bool replaceAllowed() const;

  bool operator==(const AddTrajectoryLinkCommand& rhs) const;
  bool operator!=(const AddTrajectoryLinkCommand& rhs) const;

private:
  std::string link_name_;
  std::string parent_link_name_;
  tesseract_common::JointTrajectory trajectory_;
  bool replace_allowed_{ false };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_environment

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_environment::AddTrajectoryLinkCommand, "AddTrajectoryLinkCommand")

#endif  // TESSERACT_ENVIRONMENT_ADD_TRAJECTORY_LINK_COMMAND_H
