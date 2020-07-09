#ifndef TESSERACT_COMMAND_LANGUAGE_NULL_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_NULL_WAYPOINT_H

#include <tesseract_command_language/waypoint_type.h>

namespace tesseract_planning
{
class NullWaypoint
{
public:
  int getType() const { return static_cast<int>(WaypointType::NULL_WAYPOINT); }
};
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_NULL_WAYPOINT_H
