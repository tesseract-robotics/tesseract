#ifndef TESSERACT_COMMAND_LANGUAGE_TRAJOPT_DEFAULT_PROFILE_H
#define TESSERACT_COMMAND_LANGUAGE_TRAJOPT_DEFAULT_PROFILE_H

#include <tesseract_motion_planners/trajopt/config/trajopt_collision_config.h>
#include <tesseract_command_language/planners/trajopt/trajopt_profile.h>

#include <Eigen/Geometry>

namespace tesseract_planning
{

class TrajOptDefaultProfile : public TrajOptProfile
{
public:

  /** @brief The type of contact test to perform: FIRST, CLOSEST, ALL */
  tesseract_collision::ContactTestType contact_test_type = tesseract_collision::ContactTestType::ALL;
  /** @brief Configuration info for collisions that are modeled as costs */
  tesseract_motion_planners::CollisionCostConfig collision_cost_config;
  /** @brief Configuration info for collisions that are modeled as constraints */
  tesseract_motion_planners::CollisionConstraintConfig collision_constraint_config;
  /** @brief If true, a joint velocity cost with a target of 0 will be applied for all timesteps Default: true*/
  bool smooth_velocities = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd velocity_coeff;
  /** @brief If true, a joint acceleration cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_accelerations = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd acceleration_coeff;
  /** @brief If true, a joint jerk cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_jerks = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd jerk_coeff;
  /** @brief If true, applies a cost to avoid kinematic singularities */
  bool avoid_singularity = false;
  /** @brief Optimization weight associated with kinematic singularity avoidance */
  double avoid_singularity_coeff = 5.0;

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

  TrajOptProfileResults generate(const TrajOptPlannerUniversalConfig& config) override;
protected:
  bool checkUserInput() const;

  std::pair<std::vector<std::size_t>,std::vector<int>>
  addInstructions(trajopt::ProblemConstructionInfo& pci,
                  const TrajOptPlannerUniversalConfig& config) const;

  void addCollisionCost(trajopt::ProblemConstructionInfo& pci,
                        const std::vector<int>& fixed_steps,
                        const TrajOptPlannerUniversalConfig& config) const;

  void addCollisionConstraint(trajopt::ProblemConstructionInfo& pci,
                              const std::vector<int>& fixed_steps,
                              const TrajOptPlannerUniversalConfig& config) const;

  void addVelocitySmoothing(trajopt::ProblemConstructionInfo& pci,
                            const std::vector<int>& fixed_steps,
                            const TrajOptPlannerUniversalConfig& config) const;

  void addAccelerationSmoothing(trajopt::ProblemConstructionInfo& pci,
                                const std::vector<int>& fixed_steps,
                                const TrajOptPlannerUniversalConfig& config) const;

  void addJerkSmoothing(trajopt::ProblemConstructionInfo& pci,
                        const std::vector<int>& fixed_steps,
                        const TrajOptPlannerUniversalConfig& config) const;

  void addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci,
                                   const std::vector<int>& fixed_steps,
                                   const TrajOptPlannerUniversalConfig& config) const;

  void addAvoidSingularity(trajopt::ProblemConstructionInfo& pci,
                           const std::vector<int>& fixed_steps,
                           const TrajOptPlannerUniversalConfig& config) const;
};

}

#endif // TESSERACT_COMMAND_LANGUAGE_TRAJOPT_DEFAULT_PROFILE_H
