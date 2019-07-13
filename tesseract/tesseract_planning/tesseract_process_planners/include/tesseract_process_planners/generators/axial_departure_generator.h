#ifndef TESSERACT_PLANNING_AXIAL_DEPARTURE_GENERATOR_H
#define TESSERACT_PLANNING_AXIAL_DEPARTURE_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <Eigen/Core>

namespace tesseract_process_planners
{

class AxialDepartureGenerator : public ProcessStepGenerator
{
public:
  AxialDepartureGenerator(const Eigen::Isometry3d& departure, int step_count) : departure_(departure), step_count_(step_count) {}
  ~AxialDepartureGenerator() override = default;

  std::vector<tesseract_motion_planners::Waypoint::Ptr> generate(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints,
                                                                   const ProcessDefinitionConfig& config) const override
  {
    assert(waypoints.back()->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);

    std::vector<tesseract_motion_planners::Waypoint::Ptr> departure;
    departure.reserve(step_count_ + 1);

    const tesseract_motion_planners::CartesianWaypoint::Ptr& cur_waypoint = std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(waypoints.back());
    for (int i = 0; i <= step_count_; i++)
    {
      tesseract_motion_planners::CartesianWaypoint::Ptr new_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>();
      Eigen::Isometry3d scaled = departure_;
      scaled.translation() = (i * 1.0 / step_count_) * departure_.translation();
      new_waypoint->cartesian_position_ = config.world_offset_direction * cur_waypoint->cartesian_position_ * config.local_offset_direction * scaled;
      new_waypoint->coeffs_ = cur_waypoint->coeffs_;
      new_waypoint->is_critical_ = cur_waypoint->is_critical_;
      departure.push_back(new_waypoint);
    }
    return departure;
  }

private:
  Eigen::Isometry3d departure_;
  int step_count_;
};

}

#endif // TESSERACT_PLANNING_AXIAL_DEPARTURE_GENERATOR_H
