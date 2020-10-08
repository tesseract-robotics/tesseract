/**
 * @file state_waypoint.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_common/utils.h>
#include <tesseract_command_language/visibility_control.h>

namespace tesseract_planning
{
class TESSERACT_COMMAND_LANGUAGE_PUBLIC StateWaypoint
{
public:
  StateWaypoint() = default;
  StateWaypoint(std::vector<std::string> joint_names, const Eigen::Ref<const Eigen::VectorXd>& position);
  StateWaypoint(const tinyxml2::XMLElement& xml_element);

  int getType() const;

  void print(const std::string& prefix = "") const;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const;

  /** @brief The joint corresponding to the position vector. */
  std::vector<std::string> joint_names;

  /**
   * @brief The joint position at the waypoint
   *
   * This is different from waypoint because it can be cartesian or joint and this stores the joint position solved
   * for the planned waypoint. This also can be used for determining the robot configuration if provided a cartesian
   * waypoint for generating a native robot program.
   */
  Eigen::VectorXd position;

  /** @brief The velocity at the waypoint */
  Eigen::VectorXd velocity;

  /** @brief The Acceleration at the waypoint */
  Eigen::VectorXd acceleration;

  /** @brief The Effort at the waypoint */
  Eigen::VectorXd effort;

  /** @brief The Time from start at the waypoint */
  double time{ 0 };
};
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H
