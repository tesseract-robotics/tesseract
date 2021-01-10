/**
 * @file waypoint_type.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_WAYPOINT_TYPE_H
#define TESSERACT_COMMAND_LANGUAGE_WAYPOINT_TYPE_H

namespace tesseract_planning
{
class Waypoint;

enum class WaypointType : int
{
  // NULL Waypoint
  NULL_WAYPOINT = 0,

  // Cartesian Waypoint
  CARTESIAN_WAYPOINT = 1,

  // Joint Waypoint
  JOINT_WAYPOINT = 2,

  // State Waypoint
  STATE_WAYPOINT = 3,

  // User defined types must be larger than this
  USER_DEFINED = 1000
};

bool isCartesianWaypoint(const Waypoint& waypoint);

bool isJointWaypoint(const Waypoint& waypoint);

bool isStateWaypoint(const Waypoint& waypoint);

bool isNullWaypoint(const Waypoint& waypoint);

}  // namespace tesseract_planning
#endif  // TESSERACT_COMMAND_LANGUAGE_WAYPOINT_TYPE_H
