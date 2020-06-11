#ifndef TESSERACT_COMMAND_LANGUAGE_DESCARTES_MOTION_PLANNER_CONFIG_H
#define TESSERACT_COMMAND_LANGUAGE_DESCARTES_MOTION_PLANNER_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/interface/edge_evaluator.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/descartes_light.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
template <typename FloatType>
struct DescartesMotionPlannerConfig
{
  using Ptr = std::shared_ptr<DescartesMotionPlannerConfig<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesMotionPlannerConfig<FloatType>>;

  // These are required by descartes
  typename descartes_light::EdgeEvaluator<FloatType>::Ptr edge_evaluator;
  std::vector<descartes_core::TimingConstraint<FloatType>> timing_constraint;
  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> samplers;
  int num_threads = descartes_light::Solver<double>::getMaxThreads();
  int dof {0};

  /**
   * @brief This gets called by the planner. This must be called by the inheriting class
   * @return True if successful, otherwise false.
   */
  virtual bool generate()
  {
    // Check that parameters are valid
    if (edge_evaluator == nullptr)
    {
      CONSOLE_BRIDGE_logError("In DescartesMotionPlannerConfig: edge_evaluator is a required parameter and has not been set");
      return false;
    }

    if (timing_constraint.empty())
    {
      CONSOLE_BRIDGE_logError("In DescartesMotionPlannerConfig: timing_constraint is a required parameter and has not been set");
      return false;
    }

    if (samplers.empty())
    {
      CONSOLE_BRIDGE_logError("In DescartesMotionPlannerConfig: samplers is a required parameter and has not been set");
      return false;
    }

    if (timing_constraint.size() != samplers.size())
    {
      CONSOLE_BRIDGE_logError("In DescartesMotionPlannerConfig: timing_constraint and samplers must be the same size");
      return false;
    }

    if (!(dof > 0))
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
