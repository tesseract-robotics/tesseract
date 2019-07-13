#ifndef TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H
#define TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <tesseract_motion_planners/core/utils.h>
#include <Eigen/Core>

namespace tesseract_process_planners
{

class LinearTransitionGenerator : public ProcessTransitionGenerator
{
public:
  LinearTransitionGenerator(int step_count) : step_count_(step_count) {}
  ~LinearTransitionGenerator() override = default;

  std::vector<tesseract_motion_planners::WaypointPtr> generate(const tesseract_motion_planners::WaypointPtr& start_waypoint,
                                                               const tesseract_motion_planners::WaypointPtr& end_waypoint) const override
  {
    assert(start_waypoint->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
    assert(end_waypoint->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);

    const tesseract_motion_planners::CartesianWaypointPtr& w1 = std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(start_waypoint);
    const tesseract_motion_planners::CartesianWaypointPtr& w2 = std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(end_waypoint);

    return tesseract_motion_planners::interpolate(*w1, *w2, step_count_);
  }

private:
  int step_count_;
};

}

#endif // TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H
