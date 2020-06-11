#ifndef TESSERACT_COMMAND_LANGUAGE_DECARTES_MOTION_PLANNER_H
#define TESSERACT_COMMAND_LANGUAGE_DECARTES_MOTION_PLANNER_H

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/interface/edge_evaluator.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/descartes_light.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner_status_category.h>
#include <tesseract_command_language/descartes/descartes_motion_planner_config.h>

namespace tesseract_planning
{

template <typename FloatType>
class DescartesMotionPlanner : public tesseract_motion_planners::MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  DescartesMotionPlanner(std::string name = "DESCARTES");

  ~DescartesMotionPlanner() override = default;
  DescartesMotionPlanner(const DescartesMotionPlanner&) = default;
  DescartesMotionPlanner& operator=(const DescartesMotionPlanner&) = default;
  DescartesMotionPlanner(DescartesMotionPlanner&&) noexcept = default;
  DescartesMotionPlanner& operator=(DescartesMotionPlanner&&) noexcept = default;

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(typename DescartesMotionPlannerConfig<FloatType>::Ptr config);

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param check_type The type of validation check to be performed on the planned trajectory
   * @param verbose Boolean indicating whether logging information about the motion planning solution should be printed
   * to console
   * @return true if optimization complete
   */
  tesseract_common::StatusCode solve(tesseract_motion_planners::PlannerResponse& response,
                                     tesseract_motion_planners::PostPlanCheckType check_type = tesseract_motion_planners::PostPlanCheckType::DISCRETE_CONTINUOUS_COLLISION,
                                     bool verbose = false) override;

  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  tesseract_common::StatusCode isConfigured() const override;

private:
  /** @brief The planners configuration */
  typename DescartesMotionPlannerConfig<FloatType>::Ptr config_;

  /** @brief The planners status codes */
  std::shared_ptr<const tesseract_motion_planners::DescartesMotionPlannerStatusCategory> status_category_;
};

using DescartesMotionPlannerD = DescartesMotionPlanner<double>;
using DescartesMotionPlannerF = DescartesMotionPlanner<float>;

}  // namespace tesseract_motion_planners
#endif // TESSERACT_COMMAND_LANGUAGE_DECARTES_MOTION_PLANNER_H
