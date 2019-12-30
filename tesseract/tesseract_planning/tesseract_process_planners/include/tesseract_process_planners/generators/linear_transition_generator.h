#ifndef TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H
#define TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_planners/process_definition.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_process_planners
{
class TESSERACT_PUBLIC LinearTransitionGenerator : public ProcessTransitionGenerator
{
public:
  LinearTransitionGenerator(int step_count) : step_count_(step_count) {}
  ~LinearTransitionGenerator() override = default;
  LinearTransitionGenerator(const LinearTransitionGenerator&) = default;
  LinearTransitionGenerator& operator=(const LinearTransitionGenerator&) = default;
  LinearTransitionGenerator(LinearTransitionGenerator&&) = default;
  LinearTransitionGenerator& operator=(LinearTransitionGenerator&&) = default;

  std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const tesseract_motion_planners::Waypoint::Ptr& start_waypoint,
           const tesseract_motion_planners::Waypoint::Ptr& end_waypoint) const override
  {
    assert(start_waypoint->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
    assert(end_waypoint->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);

    const tesseract_motion_planners::CartesianWaypoint::Ptr& w1 =
        std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(start_waypoint);
    const tesseract_motion_planners::CartesianWaypoint::Ptr& w2 =
        std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(end_waypoint);

    return tesseract_motion_planners::interpolate(*w1, *w2, step_count_);
  }

private:
  int step_count_;
};

}  // namespace tesseract_process_planners

#endif  // TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H
