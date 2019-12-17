#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_TRAJOPT_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_TRAJOPT_PLANNER_H

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_common/status_code.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
class DescartesTrajOptArrayPlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  DescartesTrajOptArrayPlanner(std::string name = "DESCARTES_TRAJOPT_ARRAY");

  ~DescartesTrajOptArrayPlanner() override = default;
  DescartesTrajOptArrayPlanner(const DescartesTrajOptArrayPlanner&) = default;
  DescartesTrajOptArrayPlanner& operator=(const DescartesTrajOptArrayPlanner&) = default;
  DescartesTrajOptArrayPlanner(DescartesTrajOptArrayPlanner&&) noexcept = default;
  DescartesTrajOptArrayPlanner& operator=(DescartesTrajOptArrayPlanner&&) noexcept = default;

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(const DescartesMotionPlannerConfig<FloatType>& descartes_config,
                        const TrajOptPlannerConfig::Ptr& trajopt_config);

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param verbose Boolean indicating whether logging information about the motion planning solution should be printed
   * to console
   * @return true if optimization complete
   */
  tesseract_common::StatusCode solve(PlannerResponse& response, bool verbose = false) override;

  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  tesseract_common::StatusCode isConfigured() const override;

protected:
  TrajOptMotionPlanner trajopt_planner_;
  DescartesMotionPlanner<FloatType> descartes_planner_;
  std::shared_ptr<const tesseract_common::GeneralStatusCategory> status_category_; /** @brief The plannsers status codes
                                                                                    */
  TrajOptPlannerConfig::Ptr trajopt_config_;
};

using DescartesTrajOptArrayPlannerD = DescartesTrajOptArrayPlanner<double>;
using DescartesTrajOptArrayPlannerF = DescartesTrajOptArrayPlanner<float>;

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_TRAJOPT_PLANNER_H
