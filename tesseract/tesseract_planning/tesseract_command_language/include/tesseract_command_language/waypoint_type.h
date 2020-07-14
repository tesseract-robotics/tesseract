#ifndef TESSERACT_COMMAND_LANGUAGE_WAYPOINT_TYPE_H
#define TESSERACT_COMMAND_LANGUAGE_WAYPOINT_TYPE_H

namespace tesseract_planning
{
class Waypoint;

enum class WaypointType : int
{
  // NULL Waypoint
  NULL_WAYPOINT,

  // Cartesian Waypoint
  CARTESIAN_WAYPOINT,

  // Joint Waypoint
  JOINT_WAYPOINT,

  // User defined types must be larger than this
  USER_DEFINED = 1000
};

bool isCartesianWaypoint(const Waypoint& waypoint);

bool isJointWaypoint(const Waypoint& waypoint);

bool isNullWaypoint(const Waypoint& waypoint);

}  // namespace tesseract_planning
#endif  // TESSERACT_COMMAND_LANGUAGE_WAYPOINT_TYPE_H
