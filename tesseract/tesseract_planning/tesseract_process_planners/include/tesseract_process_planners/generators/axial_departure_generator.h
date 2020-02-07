#ifndef TESSERACT_PLANNING_AXIAL_DEPARTURE_GENERATOR_H
#define TESSERACT_PLANNING_AXIAL_DEPARTURE_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <Eigen/Core>

namespace tesseract_process_planners
{
class AxialDepartureGenerator : public ProcessStepGenerator
{
public:
  AxialDepartureGenerator(const Eigen::Isometry3d& departure, int step_count)
    : departure_(departure), step_count_(step_count)
  {
  }
  ~AxialDepartureGenerator() override = default;
  AxialDepartureGenerator(const AxialDepartureGenerator&) = default;
  AxialDepartureGenerator& operator=(const AxialDepartureGenerator&) = default;
  AxialDepartureGenerator(AxialDepartureGenerator&&) = default;
  AxialDepartureGenerator& operator=(AxialDepartureGenerator&&) = default;

  std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints,
           const ProcessDefinitionConfig& config) const override
  {
    assert(waypoints.back()->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
    const tesseract_motion_planners::CartesianWaypoint::Ptr& cur_waypoint =
        std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(waypoints.back());

    Eigen::Isometry3d alt_departure = departure_;
    // Ensure that negative x/y 'double back' and positive x/y move in the direction of the path
    if (waypoints.size() >= 2)
    {
      assert(waypoints[waypoints.end() - 2].getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
      const tesseract_motion_planners::CartesianWaypoint::Ptr& prev_waypoint =
          std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(*(waypoints.end() - 2));
      Eigen::Isometry3d penultimate_in_final = cur_waypoint->getTransform().inverse() * prev_waypoint->getTransform();

      // If the final waypoint x-axis is pointing 'towards' the previous waypoint,
      // reverse the requested x-translation.  Likewise for y.

      if (penultimate_in_final.translation().x() > 0)
      {
        alt_departure.translation().x() *= -1;
      }
      if (penultimate_in_final.translation().y() > 0)
      {
        alt_departure.translation().y() *= -1;
      }
    }

    std::vector<tesseract_motion_planners::Waypoint::Ptr> departure;
    departure.reserve(static_cast<size_t>(step_count_) + 1);

    for (int i = 0; i < step_count_; i++)
    {
      Eigen::Isometry3d scaled = alt_departure;

      // Interpolate the transform
      double progress = static_cast<double>(i + 1) / static_cast<double>(step_count_);
      scaled.translation() = progress * alt_departure.translation();
      scaled.linear() = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)
                            .slerp(progress, Eigen::Quaterniond(alt_departure.rotation()))
                            .toRotationMatrix();

      tesseract_motion_planners::CartesianWaypoint::Ptr new_waypoint =
          std::make_shared<tesseract_motion_planners::CartesianWaypoint>(
              config.world_offset_direction * cur_waypoint->getTransform() * config.local_offset_direction * scaled,
              cur_waypoint->getParentLinkName());
      new_waypoint->setCoefficients(cur_waypoint->getCoefficients());
      new_waypoint->setIsCritical(cur_waypoint->isCritical());
      departure.push_back(new_waypoint);
    }
    return departure;
  }

private:
  Eigen::Isometry3d departure_;
  int step_count_;
};

}  // namespace tesseract_process_planners

#endif  // TESSERACT_PLANNING_AXIAL_DEPARTURE_GENERATOR_H
