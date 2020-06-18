#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_MOTION_PLANNER_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_MOTION_PLANNER_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/interface/edge_evaluator.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/descartes_light.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_problem.h>

namespace tesseract_planning
{

template <typename FloatType>
struct DescartesMotionPlannerConfig
{
  using Ptr = std::shared_ptr<DescartesMotionPlannerConfig<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesMotionPlannerConfig<FloatType>>;

  /** @brief The descartes problem to solve */
  DescartesProblem<FloatType> prob;

  /**
   * @brief This gets called by the planner. This must be called by the inheriting class
   * @return True if successful, otherwise false.
   */
  virtual bool generate()
  {
    // Check that parameters are valid
    if (prob.edge_evaluators.empty() || prob.edge_evaluators.size() != (prob.samplers.size() - 1))
    {
      CONSOLE_BRIDGE_logError("In DescartesMotionPlannerConfig: edge_evaluators is a required parameter and has not been set");
      return false;
    }

    if (prob.timing_constraints.empty())
    {
      CONSOLE_BRIDGE_logError("In DescartesMotionPlannerConfig: timing_constraint is a required parameter and has not been set");
      return false;
    }

    if (prob.samplers.empty())
    {
      CONSOLE_BRIDGE_logError("In DescartesMotionPlannerConfig: samplers is a required parameter and has not been set");
      return false;
    }

    if (prob.timing_constraints.size() != prob.samplers.size())
    {
      CONSOLE_BRIDGE_logError("In DescartesMotionPlannerConfig: timing_constraint and samplers must be the same size");
      return false;
    }

    if (!(prob.dof > 0))
    {
      CONSOLE_BRIDGE_logError("In DescartesMotionPlannerConfig: dof must be greater than zero");
      return false;
    }

    return true;
  }

};

using DescartesMotionPlannerConfigD = DescartesMotionPlannerConfig<double>;
using DescartesMotionPlannerConfigF = DescartesMotionPlannerConfig<float>;
}
#endif // TESSERACT_MOTION_PLANNERS_DESCARTES_PLANNER_CONFIG_H
