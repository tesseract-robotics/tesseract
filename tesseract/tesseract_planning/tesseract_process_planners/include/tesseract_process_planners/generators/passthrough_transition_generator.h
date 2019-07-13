#ifndef TESSERACT_PLANNING_PASSTHROUGH_TRANSITION_GENERATOR_H
#define TESSERACT_PLANNING_PASSTHROUGH_TRANSITION_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <Eigen/Core>

namespace tesseract_process_planners
{

class PassthroughTransitionGenerator : public ProcessTransitionGenerator
{
public:
  PassthroughTransitionGenerator() {}
  ~PassthroughTransitionGenerator() override = default;

  std::vector<tesseract_motion_planners::WaypointPtr> generate(const tesseract_motion_planners::WaypointPtr& start_waypoint,
                                                               const tesseract_motion_planners::WaypointPtr& end_waypoint) const override
  {
    std::vector<tesseract_motion_planners::WaypointPtr> transition;
    transition.reserve(2);
    transition.push_back(start_waypoint);
    transition.push_back(end_waypoint);

    return transition;
  }

};

}
#endif // TESSERACT_PLANNING_PASSTHROUGH_TRANSITION_GENERATOR_H
