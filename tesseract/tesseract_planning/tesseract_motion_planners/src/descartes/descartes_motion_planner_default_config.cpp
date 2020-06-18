#include <tesseract_motion_planners/descartes/impl/descartes_motion_planner_default_config.hpp>

namespace tesseract_planning
{
// Explicit template instantiation
template struct DescartesMotionPlannerDefaultConfig<float>;
template struct DescartesMotionPlannerDefaultConfig<double>;

}  // namespace tesseract_planning
