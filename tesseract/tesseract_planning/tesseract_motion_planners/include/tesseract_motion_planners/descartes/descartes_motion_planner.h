#ifndef TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H

#include <tesseract_common/macros.h>
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
#include <tesseract_motion_planners/descartes/descartes_problem.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>

namespace tesseract_planning
{
template <typename FloatType>
using DescartesProblemGeneratorFn =
    std::function<std::shared_ptr<DescartesProblem<FloatType>>(const std::string&,
                                                               const PlannerRequest&,
                                                               const DescartesPlanProfileMap<FloatType>&)>;

using DescartesProblemGeneratorFnD = DescartesProblemGeneratorFn<double>;
using DescartesProblemGeneratorFnF = DescartesProblemGeneratorFn<float>;

}  // namespace tesseract_planning

#ifdef SWIG
%template(DescartesProblemGeneratorFnD) tesseract_planning::DescartesProblemGeneratorFn<double>;
%shared_ptr(tesseract_planning::DescartesMotionPlannerD);
%shared_ptr(tesseract_planning::DescartesMotionPlanner<double>);
#endif  // SWIG

namespace tesseract_planning
{
template <typename FloatType>
class DescartesMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  DescartesMotionPlanner();
  ~DescartesMotionPlanner() override = default;
  DescartesMotionPlanner(const DescartesMotionPlanner&) = delete;
  DescartesMotionPlanner& operator=(const DescartesMotionPlanner&) = delete;
  DescartesMotionPlanner(DescartesMotionPlanner&&) noexcept = delete;
  DescartesMotionPlanner& operator=(DescartesMotionPlanner&&) noexcept = delete;

  const std::string& getName() const override;

#ifndef SWIG
  DescartesProblemGeneratorFn<FloatType> problem_generator;
#else
  DescartesProblemGeneratorFnD problem_generator;
#endif

  /**
   * @brief The available plan profiles
   *
   * Plan instruction profiles are used to control waypoint specific information like fixed waypoint, toleranced
   * waypoint, corner distance waypoint, etc.
   */
#ifndef SWIG
  DescartesPlanProfileMap<FloatType> plan_profiles;
#else
  DescartesPlanProfileMapD plan_profiles;
#endif

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param check_type The type of validation check to be performed on the planned trajectory
   * @param verbose Boolean indicating whether logging information about the motion planning solution should be printed
   * to console
   * @return true if optimization complete
   */
  tesseract_common::StatusCode solve(const PlannerRequest& request,
                                     PlannerResponse& response,
                                     bool verbose = false) const override;

  bool checkUserInput(const PlannerRequest& request);

  bool terminate() override;

  void clear() override;

  MotionPlanner::Ptr clone() const override;

private:
  /** @brief The planners status codes */
  std::string name_{ "DESCARTES" };
  std::shared_ptr<const DescartesMotionPlannerStatusCategory> status_category_;
};

using DescartesMotionPlannerD = DescartesMotionPlanner<double>;
using DescartesMotionPlannerF = DescartesMotionPlanner<float>;

}  // namespace tesseract_planning

#ifdef SWIG
%template(DescartesMotionPlannerD) tesseract_planning::DescartesMotionPlanner<double>;
#endif  // SWIG

#endif  // TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
