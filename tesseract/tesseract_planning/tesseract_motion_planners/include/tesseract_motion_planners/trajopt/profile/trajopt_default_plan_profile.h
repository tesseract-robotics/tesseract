#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H

#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <trajopt_sco/modeling_utils.hpp>
#include <Eigen/Geometry>

namespace tesseract_planning
{

class TrajOptDefaultPlanProfile : public TrajOptPlanProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptDefaultPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptDefaultPlanProfile>;

  Eigen::VectorXd cartesian_coeff { Eigen::VectorXd::Constant(1,1,5) };
  Eigen::VectorXd joint_coeff { Eigen::VectorXd::Constant(1,1,5) };
  trajopt::TermType term_type { trajopt::TermType::TT_CNT };

  /** @brief Error function that is set as a constraint for each timestep.
   *
   * This is a vector of std::tuple<Error Function, Error Function Jacobian, Constraint Type, Coeff>, the error
   * function, constraint type, and coeff is required, but the jacobian is optional (nullptr).
   *
   * Error Function:
   *   arg: VectorXd will be all of the joint values for one timestep.
   *   return: VectorXd of violations for each joint. Anything != 0 will be a violation
   *
   * Error Function Jacobian:
   *   arg: VectorXd will be all of the joint values for one timestep.
   *   return: Eigen::MatrixXd that represents the change in the error function with respect to joint values
   *
   * Error Constraint Type
   *
   * Coefficients/Weights
   *
   */
  std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
      constraint_error_functions;

  void apply(trajopt::ProblemConstructionInfo& pci,
             const Eigen::Isometry3d& cartesian_waypoint,
             const PlanInstruction& parent_instruction,
             const std::vector<std::string> &active_links,
             int index) override;

  void apply(trajopt::ProblemConstructionInfo& pci,
             const Eigen::VectorXd& joint_waypoint,
             const PlanInstruction& parent_instruction,
             const std::vector<std::string> &active_links,
             int index) override;
protected:

  void addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci,
                                   const std::vector<int>& fixed_steps) const;

  void addAvoidSingularity(trajopt::ProblemConstructionInfo& pci,
                           const std::vector<int>& fixed_steps) const;
};
}

#endif // TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H
