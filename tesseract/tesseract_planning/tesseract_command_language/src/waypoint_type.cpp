#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_command_language/core/waypoint.h>

namespace tesseract_planning
{
bool isCartesianWaypoint(const Waypoint& waypoint)
{
  return (waypoint.getType() == static_cast<int>(WaypointType::CARTESIAN_WAYPOINT));
}

bool isJointWaypoint(const Waypoint& waypoint)
{
  return (waypoint.getType() == static_cast<int>(WaypointType::JOINT_WAYPOINT));
}

bool isNullWaypoint(const Waypoint& waypoint)
{
  return (waypoint.getType() == static_cast<int>(WaypointType::NULL_WAYPOINT));
}

}  // namespace tesseract_planning
