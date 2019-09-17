#include "tesseract_motion_planners/trajopt/config/trajopt_planner_config.h"

namespace tesseract_motion_planners
{
TrajOptPlannerConfig::TrajOptPlannerConfig(trajopt::TrajOptProb::Ptr prob_) { prob = prob_;}

bool TrajOptPlannerConfig::generate()
{
  if (prob)
    return true;
  else
    return false;
}

} // namespace tesseract_motion_planners
