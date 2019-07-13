#ifndef TESSERACT_PLANNING_PASSTHROUGH_PROCESS_GENERATOR_H
#define TESSERACT_PLANNING_PASSTHROUGH_PROCESS_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>

namespace tesseract_process_planners
{

class PassthroughProcessGenerator : public ProcessStepGenerator
{
public:
  PassthroughProcessGenerator() {}
  ~PassthroughProcessGenerator() = default;

  std::vector<tesseract_motion_planners::WaypointPtr> generate(const std::vector<tesseract_motion_planners::WaypointPtr>& waypoints,
                                                               const ProcessDefinitionConfig& config) const override
  {
    std::vector<tesseract_motion_planners::WaypointPtr> new_waypoints;
    new_waypoints.reserve(waypoints.size());
    for(const auto& waypoint : waypoints)
    {
      assert(waypoint->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
      const tesseract_motion_planners::CartesianWaypointPtr& cur_waypoint = std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(waypoint);
      tesseract_motion_planners::CartesianWaypointPtr new_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>();
      new_waypoint->cartesian_position_ = config.world_offset_direction * cur_waypoint->cartesian_position_ * config.local_offset_direction;
      new_waypoint->coeffs_ = cur_waypoint->coeffs_;
      new_waypoint->is_critical_ = cur_waypoint->is_critical_;
      new_waypoints.push_back(new_waypoint);
    }
    return new_waypoints;
  }
};

}
#endif // TESSERACT_PLANNING_PASSTHROUGH_PROCESS_GENERATOR_H
