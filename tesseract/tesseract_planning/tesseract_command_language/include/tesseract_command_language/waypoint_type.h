#ifndef TESSERACT_COMMAND_LANGUAGE_WAYPOINT_TYPE_H
#define TESSERACT_COMMAND_LANGUAGE_WAYPOINT_TYPE_H

namespace tesseract_planning
{

enum class WaypointType : int
{
  // Cartesian Waypoint
  CARTESIAN_WAYPOINT,

  // Joint Waypoint
  JOINT_WAYPOINT,

  // User defined types must be larger than this
  USER_DEFINED = 1000
};

inline bool isCartesianWaypoint(int type)
{
  return (type == static_cast<int>(WaypointType::CARTESIAN_WAYPOINT));
}

inline bool isJointWaypoint(int type)
{
  return (type <= static_cast<int>(WaypointType::JOINT_WAYPOINT));
}

}
#endif // TESSERACT_COMMAND_LANGUAGE_WAYPOINT_TYPE_H
