#ifndef TESSERACT_PLANNING_PASSTHROUGH_TRANSITION_GENERATOR_H
#define TESSERACT_PLANNING_PASSTHROUGH_TRANSITION_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <Eigen/Core>

namespace tesseract_process_planners
{
class PassthroughTransitionGenerator : public ProcessTransitionGenerator
{
public:
  PassthroughTransitionGenerator() = default;
  ~PassthroughTransitionGenerator() override = default;
  PassthroughTransitionGenerator(const PassthroughTransitionGenerator&) = default;
  PassthroughTransitionGenerator& operator=(const PassthroughTransitionGenerator&) = default;
  PassthroughTransitionGenerator(PassthroughTransitionGenerator&&) = default;
  PassthroughTransitionGenerator& operator=(PassthroughTransitionGenerator&&) = default;

  std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const tesseract_motion_planners::Waypoint::Ptr& start_waypoint,
           const tesseract_motion_planners::Waypoint::Ptr& end_waypoint) const override
  {
    std::vector<tesseract_motion_planners::Waypoint::Ptr> transition;
    transition.reserve(2);
    transition.push_back(start_waypoint);
    transition.push_back(end_waypoint);

    return transition;
  }
};

}  // namespace tesseract_process_planners
#endif  // TESSERACT_PLANNING_PASSTHROUGH_TRANSITION_GENERATOR_H
